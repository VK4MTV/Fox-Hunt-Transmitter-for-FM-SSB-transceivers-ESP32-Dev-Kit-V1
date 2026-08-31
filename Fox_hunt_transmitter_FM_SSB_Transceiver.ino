#include <Arduino.h>
#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <LittleFS.h>
#include <SD.h>
#include <SPI.h>
#include <ArduinoJson.h>
#include <vector>
#include <utility>
#include <cstring>
#include <esp_task_wdt.h>
#include <math.h>

// --- RADIO / FIELD CONFIG ---
const char* ssid     = "FOX_HUNT_AM_TRANSMITTER";
const char* password = "foxhunt123";   // change this; the AP can key PTT

#define DAC_PIN   25
#define PTT_PIN   27
#define PLAY_PIN  14
#define STOP_PIN  13
#define SD_CS_PIN  5

#define BUFFER_SIZE 512

// Morse: 200 ms dit ~ 6 wpm. ITU: dah=3, element=1, letter=3, word=7.
static const uint32_t DIT_MS = 200;

// Software LPF. Analog RC/LC after the DAC is still mandatory.
static const float AUDIO_LPF_HZ     = 2700.0f;  // FM/SSB-ish; AM mask wants ~2.5 kHz
static const float LIMITER_THRESH   = 0.85f;    // peak, relative to full scale
static const float GAIN_BOOST       = 1.10f;    // AC gain around mid-scale, not * sample
static const bool  DROP_PTT_ON_SILENCE = true;  // false = keep carrier for DF during pauses

static const char* MORSE_PREFIX   = "M:";
static const char* SILENCE_PREFIX = "S:";

uint8_t audioBuffer[BUFFER_SIZE];

AsyncWebServer server(80);
std::vector<String> playlist;
int currentIndex = 0;
volatile bool isRunning = false;
bool sdMounted = false;
int dacLevel = 128;
SemaphoreHandle_t playlistMutex = nullptr;
TaskHandle_t AudioTaskHandle = nullptr;

const char* morse_table[] = {
  "-----", ".----", "..---", "...--", "....-", ".....", "-....", "--...", "---..", "----.",
  "", "", "", "", "", "", "",
  ".-", "-...", "-.-.", "-..", ".", "..-.", "--.", "....", "..", ".---", "-.-", ".-..",
  "--", "-.", "---", ".--.", "--.-", ".-.", "...", "-", "..-", "...-", ".--", "-..-", "-.--", "--.."
};
const int MORSE_LETTER_OFFSET = 17;

#define SINE_SAMPLE_US  31
#define DOT_SAMPLES     ((DIT_MS * 1000UL) / SINE_SAMPLE_US)
#define DASH_SAMPLES    ((3 * DIT_MS * 1000UL) / SINE_SAMPLE_US)

const uint8_t sine_table[32] = {
  128, 148, 166, 184, 199, 211, 220, 226, 228, 226, 220, 211,
  199, 184, 166, 148, 128, 108,  90,  72,  57,  45,  36,  30,
   28,  30,  36,  45,  57,  72,  90, 108
};

const int step_table[] = {
  7,8,9,10,11,12,13,14,16,17,19,21,23,25,28,31,34,37,41,45,50,55,60,66,73,80,88,97,107,118,130,143,
  157,173,190,209,230,253,279,307,337,371,408,449,494,544,598,658,724,796,876,963,1060,1166,1282,1411,
  1552,1707,1878,2066,2272,2499,2749,3024,3327,3660,4026,4428,4871,5358,5894,6484,7132,7845,8630,9493,
  10442,11487,12635,13899,15289,16818,18500,20350,22385,24623,27086,29794,32767
};
const int8_t index_table[16] = { -1,-1,-1,-1,2,4,6,8, -1,-1,-1,-1,2,4,6,8 };

int predictor = 0;
int step_index = 0;

// --- tiny DSP: two Butterworth biquads + peak limiter ---
struct Biquad {
  float b0 = 1, b1 = 0, b2 = 0, a1 = 0, a2 = 0;
  float z1 = 0, z2 = 0;
  void setLowpass(float fs, float fc) {
    if (fs < 100.0f) fs = 8000.0f;
    if (fc < 100.0f) fc = 100.0f;
    if (fc > fs * 0.45f) fc = fs * 0.45f;  // 5 kHz files: don't ask for 2.7 kHz past Nyquist
    const float w0 = 2.0f * PI * fc / fs;
    const float cosw = cosf(w0);
    const float sinw = sinf(w0);
    const float alpha = sinw / (2.0f * 0.70710678f);
    const float a0 = 1.0f + alpha;
    b0 = ((1.0f - cosw) * 0.5f) / a0;
    b1 = (1.0f - cosw) / a0;
    b2 = b0;
    a1 = (-2.0f * cosw) / a0;
    a2 = (1.0f - alpha) / a0;
    z1 = z2 = 0;
  }
  float process(float x) {
    float y = b0 * x + z1;
    z1 = b1 * x - a1 * y + z2;
    z2 = b2 * x - a2 * y;
    return y;
  }
};

struct AudioPath {
  Biquad lp1, lp2;
  float env = 0;
  void setup(float fs) {
    lp1.setLowpass(fs, AUDIO_LPF_HZ);
    lp2.setLowpass(fs, AUDIO_LPF_HZ);
    env = 0;
  }
  int processUnsigned8(int s) {
    float x = (s - 128) / 128.0f;
    x = lp2.process(lp1.process(x));
    float ax = fabsf(x);
    env = (ax > env) ? ax : env * 0.9995f;          // ~fast attack, slow decay
    float g = 1.0f;
    if (env > LIMITER_THRESH) g = LIMITER_THRESH / env;
    x *= g * GAIN_BOOST;
    if (x >  0.97f) x =  0.97f;
    if (x < -0.97f) x = -0.97f;
    return constrain(128 + (int)(x * 128.0f), 0, 255);
  }
} audioPath;

static bool waitMs(uint32_t ms) {
  uint32_t start = millis();
  while ((millis() - start) < ms) {
    if (!isRunning) return false;
    esp_task_wdt_reset();
    vTaskDelay(pdMS_TO_TICKS(10));
  }
  return isRunning;
}

static String normalizeFsName(const String& n) {
  if (n.startsWith("/")) return n.substring(1);
  return n;
}

File openAnyFile(const String& item) {
  if (item.startsWith("SD:")) return SD.open(item.substring(3));
  return LittleFS.open("/" + normalizeFsName(item));
}

bool fileExistsAny(const String& item) {
  if (item.startsWith("SD:")) return SD.exists(item.substring(3));
  return LittleFS.exists("/" + normalizeFsName(item));
}

static bool isSafeFilename(const String& n) {
  if (n.length() == 0 || n.length() > 64) return false;
  if (n.indexOf("..") >= 0 || n.indexOf('/') >= 0 || n.indexOf('\\') >= 0) return false;
  if (n.equalsIgnoreCase("playlist.json")) return false;
  return true;
}

String getStorageInfo() {
  String info = "LittleFS: ";
  size_t total = LittleFS.totalBytes();
  size_t used  = LittleFS.usedBytes();
  if (total == 0) {
    info += "unmounted";
  } else {
    info += String((float)used / (float)total * 100.0f, 1) + "% (" +
            String(used / 1024) + "KB/" + String(total / 1024) + "KB)";
  }
  if (sdMounted) {
    uint64_t sdTotal = SD.totalBytes() / 1024 / 1024;
    uint64_t sdUsed  = SD.usedBytes()  / 1024 / 1024;
    info += " | SD: " + String((uint32_t)sdUsed) + "MB/" + String((uint32_t)sdTotal) + "MB";
  } else {
    info += " | SD: not mounted";
  }
  return info;
}

void savePlaylistToDisk() {
  File file = LittleFS.open("/playlist.json", "w");
  if (!file) return;
  JsonDocument doc;
  JsonArray arr = doc.to<JsonArray>();
  xSemaphoreTake(playlistMutex, portMAX_DELAY);
  for (const auto& s : playlist) arr.add(s);
  xSemaphoreGive(playlistMutex);
  serializeJson(doc, file);
  file.close();
}

void loadPlaylistFromDisk() {
  if (!LittleFS.exists("/playlist.json")) return;
  File file = LittleFS.open("/playlist.json", "r");
  if (!file) return;
  JsonDocument doc;
  DeserializationError err = deserializeJson(doc, file);
  file.close();
  if (err) return;
  xSemaphoreTake(playlistMutex, portMAX_DELAY);
  playlist.clear();
  for (JsonVariant v : doc.as<JsonArray>()) playlist.push_back(v.as<String>());
  xSemaphoreGive(playlistMutex);
}

void apply_ramp(int target, int ms) {
  const int steps = 20;
  if (ms < 1) ms = 5;
  int diff = target - dacLevel;
  for (int i = 1; i <= steps; ++i) {
    int val = constrain(dacLevel + (diff * i / steps), 0, 255);
    dacWrite(DAC_PIN, val);
    delayMicroseconds((ms * 1000) / steps);
  }
  dacLevel = target;
}

void decodeAndOutput(uint8_t nibble) {
  int step = step_table[step_index];
  int diff = step >> 3;
  if (nibble & 4) diff += step;
  if (nibble & 2) diff += (step >> 1);
  if (nibble & 1) diff += (step >> 2);
  if (nibble & 8) predictor -= diff;
  else            predictor += diff;
  predictor  = constrain(predictor, -32768, 32767);
  step_index = constrain(step_index + index_table[nibble & 0x0F], 0, 88);
  int sample = (predictor + 32768) >> 8;
  int out = audioPath.processUnsigned8(sample);
  dacWrite(DAC_PIN, (uint8_t)out);
  dacLevel = out;
}

static bool parseWav(File& f, uint32_t& sampleRate, bool& isPcm8) {
  uint8_t riff[12];
  if (f.read(riff, 12) != 12) return false;
  if (memcmp(riff, "RIFF", 4) || memcmp(riff + 8, "WAVE", 4)) return false;

  sampleRate = 8000;
  isPcm8 = false;
  bool gotFmt = false, gotData = false;
  uint16_t format = 0, bits = 0, ch = 0;

  while (f.available()) {
    uint8_t hdr[8];
    if (f.read(hdr, 8) != 8) break;
    uint32_t sz;
    memcpy(&sz, hdr + 4, 4);
    if (!memcmp(hdr, "fmt ", 4)) {
      uint8_t fmt[16];
      uint32_t n = sz < 16 ? sz : 16;
      if (f.read(fmt, n) != (int)n) return false;
      if (sz > n) f.seek(f.position() + (sz - n));
      memcpy(&format,     fmt + 0, 2);
      memcpy(&ch,         fmt + 2, 2);
      memcpy(&sampleRate, fmt + 4, 4);
      memcpy(&bits,       fmt + 14, 2);
      gotFmt = true;
    } else if (!memcmp(hdr, "data", 4)) {
      gotData = true;
      break;
    } else {
      f.seek(f.position() + sz);
    }
    if (sz & 1) f.seek(f.position() + 1);
  }
  isPcm8 = gotFmt && format == 1 && bits == 8 && ch == 1;
  return gotData;
}

void playWavFile(const String& item) {
  File f = openAnyFile(item);
  if (!f) return;

  uint32_t sampleRate = 8000;
  bool isPcm8 = false;
  if (!parseWav(f, sampleRate, isPcm8)) {
    f.close();
    return;
  }

  predictor = 0;
  step_index = 0;
  audioPath.setup((float)(sampleRate ? sampleRate : 8000));
  const uint32_t sampleDelayUs = sampleRate ? (1000000UL / sampleRate) : 125;

  digitalWrite(PTT_PIN, HIGH);
  apply_ramp(128, 12);

  while (f.available() && isRunning) {
    int n = f.read(audioBuffer, BUFFER_SIZE);
    if (n <= 0) break;
    for (int i = 0; i < n && isRunning; ++i) {
      uint8_t b = audioBuffer[i];
      if (isPcm8) {
        int out = audioPath.processUnsigned8(b);
        dacWrite(DAC_PIN, (uint8_t)out);
        dacLevel = out;
        delayMicroseconds(sampleDelayUs);
      } else {
        // Legacy raw IMA nibbles (NOT Microsoft WAV IMA blocks).
        decodeAndOutput(b & 0x0F);
        delayMicroseconds(sampleDelayUs);
        if (!isRunning) break;
        decodeAndOutput(b >> 4);
        delayMicroseconds(sampleDelayUs);
      }
    }
    esp_task_wdt_reset();
  }

  f.close();
  apply_ramp(128, 15);
  digitalWrite(PTT_PIN, LOW);
}

static void toneDitOrDah(int samples) {
  for (int s = 0; s < samples; ++s) {
    if (!isRunning) return;
    dacWrite(DAC_PIN, sine_table[s & 31]);
    delayMicroseconds(SINE_SAMPLE_US);
    if ((s & 127) == 0) esp_task_wdt_reset();
  }
  dacWrite(DAC_PIN, 128);
  dacLevel = 128;
}

void sendMorse(const String& text) {
  String upper = text;
  upper.toUpperCase();

  digitalWrite(PTT_PIN, HIGH);
  apply_ramp(128, 12);

  for (size_t i = 0; i < upper.length() && isRunning; ++i) {
    char c = upper[i];
    if (c == ' ') {
      if (!waitMs(7 * DIT_MS)) break;
      continue;
    }

    const char* pattern = nullptr;
    if (c >= '0' && c <= '9') pattern = morse_table[c - '0'];
    else if (c >= 'A' && c <= 'Z') pattern = morse_table[c - 'A' + MORSE_LETTER_OFFSET];
    else if (c == '/') pattern = "-..-.";
    if (!pattern || !pattern[0]) continue;

    for (size_t j = 0; pattern[j] && isRunning; ++j) {
      toneDitOrDah(pattern[j] == '.' ? DOT_SAMPLES : DASH_SAMPLES);
      if (pattern[j + 1]) waitMs(DIT_MS);          // element gap
    }
    waitMs(3 * DIT_MS);                            // letter gap
  }

  apply_ramp(128, 15);
  digitalWrite(PTT_PIN, LOW);
}

void playNextItem() {
  String item;
  int idx = 0;

  xSemaphoreTake(playlistMutex, portMAX_DELAY);
  if (playlist.empty() || !isRunning) {
    xSemaphoreGive(playlistMutex);
    return;
  }
  idx = currentIndex;
  if (idx < 0 || idx >= (int)playlist.size()) idx = 0;
  item = playlist[idx];
  xSemaphoreGive(playlistMutex);

  if (item.startsWith(MORSE_PREFIX)) {
    sendMorse(item.substring(2));
  } else if (item.startsWith(SILENCE_PREFIX)) {
    dacWrite(DAC_PIN, 128);
    dacLevel = 128;
    if (DROP_PTT_ON_SILENCE) digitalWrite(PTT_PIN, LOW);
    waitMs((uint32_t)item.substring(2).toInt() * 1000UL);
  } else if (fileExistsAny(item)) {
    playWavFile(item);
  }

  // Only advance if the item completed (STOP should resume the same slot).
  if (isRunning) {
    xSemaphoreTake(playlistMutex, portMAX_DELAY);
    if (!playlist.empty()) currentIndex = (idx + 1) % (int)playlist.size();
    else currentIndex = 0;
    xSemaphoreGive(playlistMutex);
  }
}

void audioTask(void*) {
  esp_task_wdt_add(nullptr);
  for (;;) {
    bool run = isRunning;
    int n;
    xSemaphoreTake(playlistMutex, portMAX_DELAY);
    n = (int)playlist.size();
    xSemaphoreGive(playlistMutex);

    if (run && n > 0) playNextItem();
    else {
      esp_task_wdt_reset();
      vTaskDelay(pdMS_TO_TICKS(50));
    }
  }
}

const char index_html[] PROGMEM = R"HTML(
<!DOCTYPE HTML><html><head><title>Fox Hunt Transmitter</title>
<style>
  body { background:#000; color:#0f0; font-family:'Courier New', monospace; padding:20px; }
  h2 { border-bottom: 2px solid #040; }
  .box { margin:15px 0; padding:15px; border:1px solid #0f0; background:#010; }
  .item { display:flex; align-items:center; background:#010; border:1px solid #030; margin:5px 0; padding:5px; }
  button { background:#020; color:#0f0; border:1px solid #0f0; cursor:pointer; padding:6px; margin:2px; }
  button:hover { background:#0f0; color:#000; }
  input { background:#000; color:#0f0; border:1px solid #0f0; padding:5px; }
  .save-btn { background:#040; border-color:#0f0; font-weight:bold; width:100%; font-size:16px; margin:10px 0; }
</style></head><body>
  <div id="mem" style="border:1px solid #040; padding:10px;">STORAGE: ...</div>
  <h2>[ PLAYLIST ]</h2><div id="playlist"></div>
  <button class="save-btn" onclick="saveAndPlay()">SAVE & START BROADCAST</button>
  <div class="box">[ MORSE ] <input id="mmsg" placeholder="VK4MTV FOX 1"><button onclick="addM()">ADD</button></div>
  <div class="box">[ UPLOAD WAV ] <input type="file" id="fi" accept=".wav"><button onclick="up()">UPLOAD (LittleFS)</button><div id="prg"></div></div>
  <div class="box">[ SILENCE sec ] <input type="number" id="sec" min="1"><button onclick="addS()">ADD</button></div>
  <h2>[ FILES ]</h2><div id="files"></div>
  <button style="color:red; border-color:red;" onclick="clearPL()">CLEAR PLAYLIST</button>
<script>
var esc = function(s) {
  return String(s).replace(/[&<>"']/g, function(c) {
    return {'&':'&amp;','<':'&lt;','>':'&gt;','"':'&quot;',"'":'&#39;'}[c];
  });
};
var load = function() {
  fetch('/api/status').then(function(r){ return r.json(); }).then(function(d) {
    document.getElementById('mem').innerText = d.storage;
    var p = '';
    d.playlist.forEach(function(f,i) {
      p += '<div class="item">' +
        '<button onclick="moveItem(' + i + ',-1)">▲</button>' +
        '<button onclick="moveItem(' + i + ',1)">▼</button>' +
        '<button onclick="removeItem(' + i + ')">X</button> ' + esc(f) +
        '</div>';
    });
    document.getElementById('playlist').innerHTML = p || 'Empty';
    var fstr = '';
    d.files.forEach(function(n) {
      var q = encodeURIComponent(n);
      var prefix = n.indexOf('SD:') === 0 ? '💾 ' : '';
      fstr += '<div class="item">' +
        '<button onclick="addToPlaylist(\'' + q + '\')">ADD</button> ' + prefix + esc(n) +
        ' <button onclick="deleteFile(\'' + q + '\')">DEL</button></div>';
    });
    document.getElementById('files').innerHTML = fstr;
  });
};
var addToPlaylist = function(filename) { fetch('/api/add?f=' + filename).then(load); };
var removeItem = function(i) { fetch('/api/remove?i=' + i).then(load); };
var moveItem = function(i,d) { fetch('/api/move?i=' + i + '&d=' + d).then(load); };
var deleteFile = function(filename) { fetch('/api/del-disk?f=' + filename).then(load); };
var addM = function() {
  var m = document.getElementById('mmsg').value;
  if (m) fetch('/api/add?f=M:' + encodeURIComponent(m)).then(load);
};
var addS = function() {
  var s = document.getElementById('sec').value;
  if (s) fetch('/api/add?f=S:' + encodeURIComponent(s)).then(load);
};
var saveAndPlay = function() {
  fetch('/api/save-play').then(function(){ load(); alert('Playlist saved and broadcast started!'); });
};
var up = function() {
  var fi = document.getElementById('fi');
  if (!fi.files.length) return;
  var fd = new FormData();
  fd.append('data', fi.files[0], fi.files[0].name);
  var x = new XMLHttpRequest();
  x.open('POST', '/api/upload');
  x.upload.onprogress = function(e) {
    document.getElementById('prg').innerText = Math.round(e.loaded / e.total * 100) + '%';
  };
  x.onload = function() { load(); document.getElementById('prg').innerText = 'Done'; };
  x.send(fd);
};
var clearPL = function() { if (confirm('Wipe playlist?')) fetch('/api/clear').then(load); };
load();
</script></body></html>
)HTML";


static void withPlaylistLock(void (*fn)()) {
  xSemaphoreTake(playlistMutex, portMAX_DELAY);
  fn();
  xSemaphoreGive(playlistMutex);
}

void setup() {
  Serial.begin(115200);
  setCpuFrequencyMhz(240);

  playlistMutex = xSemaphoreCreateMutex();

  pinMode(PLAY_PIN, INPUT_PULLUP);
  pinMode(STOP_PIN, INPUT_PULLUP);
  pinMode(PTT_PIN, OUTPUT);
  digitalWrite(PTT_PIN, LOW);
  dacWrite(DAC_PIN, 128);

  if (!LittleFS.begin(false)) {
    Serial.println("LittleFS mount failed (not formatting).");
  } else {
    loadPlaylistFromDisk();
  }

  sdMounted = SD.begin(SD_CS_PIN);
  Serial.println(sdMounted ? "SD mounted" : "SD not detected");

  WiFi.softAP(ssid, password);
  Serial.print("AP started: ");
  Serial.println(ssid);

  server.on("/", HTTP_GET, [](AsyncWebServerRequest *r) {
    r->send(200, "text/html", index_html);
  });

  server.on("/api/status", HTTP_GET, [](AsyncWebServerRequest *r) {
    JsonDocument doc;
    doc["storage"] = getStorageInfo();
    JsonArray pl = doc["playlist"].to<JsonArray>();
    xSemaphoreTake(playlistMutex, portMAX_DELAY);
    for (const auto& s : playlist) pl.add(s);
    xSemaphoreGive(playlistMutex);

    JsonArray fs = doc["files"].to<JsonArray>();
    File root = LittleFS.open("/");
    File file = root.openNextFile();
    while (file) {
      String n = normalizeFsName(file.name());
      if (!n.endsWith(".json")) fs.add(n);
      file.close();
      file = root.openNextFile();
    }
    root.close();

    if (sdMounted) {
      root = SD.open("/");
      file = root.openNextFile();
      while (file) {
        if (!file.isDirectory()) fs.add("SD:" + normalizeFsName(String(file.name())));
        file.close();
        file = root.openNextFile();
      }
      root.close();
    }

    String out;
    serializeJson(doc, out);
    r->send(200, "application/json", out);
  });

  server.on("/api/add", HTTP_GET, [](AsyncWebServerRequest *r) {
    if (!r->hasParam("f")) { r->send(400); return; }
    String v = r->getParam("f")->value();
    xSemaphoreTake(playlistMutex, portMAX_DELAY);
    playlist.push_back(v);
    xSemaphoreGive(playlistMutex);
    r->send(200);
  });

  server.on("/api/move", HTTP_GET, [](AsyncWebServerRequest *r) {
    if (!r->hasParam("i") || !r->hasParam("d")) { r->send(400); return; }
    int i = r->getParam("i")->value().toInt();
    int d = r->getParam("d")->value().toInt();
    xSemaphoreTake(playlistMutex, portMAX_DELAY);
    if (i >= 0 && i < (int)playlist.size() && i + d >= 0 && i + d < (int)playlist.size())
      std::swap(playlist[i], playlist[i + d]);
    xSemaphoreGive(playlistMutex);
    r->send(200);
  });

  server.on("/api/remove", HTTP_GET, [](AsyncWebServerRequest *r) {
    if (!r->hasParam("i")) { r->send(400); return; }
    int i = r->getParam("i")->value().toInt();
    xSemaphoreTake(playlistMutex, portMAX_DELAY);
    if (i >= 0 && i < (int)playlist.size()) playlist.erase(playlist.begin() + i);
    if (currentIndex >= (int)playlist.size()) currentIndex = 0;
    xSemaphoreGive(playlistMutex);
    r->send(200);
  });

  server.on("/api/del-disk", HTTP_GET, [](AsyncWebServerRequest *r) {
    if (!r->hasParam("f")) { r->send(400); return; }
    String fname = r->getParam("f")->value();
    if (fname.startsWith("SD:")) SD.remove(fname.substring(3));
    else {
      fname = normalizeFsName(fname);
      if (isSafeFilename(fname)) LittleFS.remove("/" + fname);
    }
    r->send(200);
  });

  server.on("/api/clear", HTTP_GET, [](AsyncWebServerRequest *r) {
    xSemaphoreTake(playlistMutex, portMAX_DELAY);
    playlist.clear();
    currentIndex = 0;
    xSemaphoreGive(playlistMutex);
    r->send(200);
  });

  server.on("/api/save-play", HTTP_GET, [](AsyncWebServerRequest *r) {
    savePlaylistToDisk();
    xSemaphoreTake(playlistMutex, portMAX_DELAY);
    currentIndex = 0;
    xSemaphoreGive(playlistMutex);
    isRunning = true;
    r->send(200);
  });

  server.on("/api/upload", HTTP_POST,
    [](AsyncWebServerRequest *r) { r->send(200); },
    [](AsyncWebServerRequest *request, String filename, size_t index, uint8_t *data, size_t len, bool final) {
      filename = normalizeFsName(filename);
      if (!isSafeFilename(filename)) return;
      if (!index) request->_tempFile = LittleFS.open("/" + filename, "w");
      if (request->_tempFile) request->_tempFile.write(data, len);
      if (final && request->_tempFile) request->_tempFile.close();
    });

  server.begin();

#if defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 3)
  esp_task_wdt_config_t wdt_config = {
    .timeout_ms = 8000, .idle_core_mask = 0, .trigger_panic = true
  };
  esp_task_wdt_init(&wdt_config);
#else
  esp_task_wdt_init(8, true);
#endif

  xTaskCreatePinnedToCore(audioTask, "AudioTask", 12288, nullptr, 3, &AudioTaskHandle, 1);
  Serial.println("Fox Hunt Transmitter ready");
}

void loop() {
  static bool playPrev = true;
  static bool stopPrev = true;
  bool playNow = digitalRead(PLAY_PIN);
  bool stopNow = digitalRead(STOP_PIN);

  if (playPrev && !playNow) {          // falling edge, pulled-up
    xSemaphoreTake(playlistMutex, portMAX_DELAY);
    currentIndex = 0;
    xSemaphoreGive(playlistMutex);
    isRunning = true;
  }
  if (stopPrev && !stopNow) {
    isRunning = false;
    dacWrite(DAC_PIN, 128);
    dacLevel = 128;
    digitalWrite(PTT_PIN, LOW);
  }
  playPrev = playNow;
  stopPrev = stopNow;
  vTaskDelay(pdMS_TO_TICKS(10));
}
