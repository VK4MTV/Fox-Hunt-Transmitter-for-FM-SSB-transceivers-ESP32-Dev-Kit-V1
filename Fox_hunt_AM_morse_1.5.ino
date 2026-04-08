#include <Arduino.h>
#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <LittleFS.h>
#include <ArduinoJson.h>
#include <vector>
#include "esp_system.h"
#include <esp_task_wdt.h>

// --- CONFIGURATION ---
const char* ssid = "FOX_HUNT_AM_TRANSMITTER";
const char* password = "foxhunt123";
#define DAC_PIN 25      // GPIO25 = DAC1 on ESP32
#define PTT_PIN 27      // PTT enable: HIGH during TX, LOW when idle
#define PLAY_PIN 14     
#define STOP_PIN 13
#define BUFFER_SIZE 512
uint8_t audioBuffer[BUFFER_SIZE];
int bufferPointer = 0;
int bytesInBuffer = 0;

// --- GLOBALS ---
AsyncWebServer server(80);
std::vector<String> playlist;
int currentIndex = 0;
bool isRunning = false;
float gain_booster = 1.2; 
int last_sample = 128; 
TaskHandle_t AudioTaskHandle;

const char* morse_table[] = {
  "-----", ".----", "..---", "...--", "....-", ".....", "-....", "--...", "---..", "----.", 
  "", "", "", "", "", "", "", 
  ".-", "-...", "-.-.", "-..", ".", "..-.", "--.", "....", "..", ".---", "-.-", ".-..", 
  "--", "-.", "---", ".--.", "--.-", ".-.", "...", "-", "..-", "...-", ".--", "-..-", "-.--", "--.." 
};

const int index_table[] = { -1, -1, -1, -1, 2, 4, 6, 8, -1, -1, -1, -1, 2, 4, 6, 8 };
const int step_table[] = { 
  7, 8, 9, 10, 11, 12, 13, 14, 16, 17, 19, 21, 23, 25, 28, 31, 34, 37, 41, 45, 
  50, 55, 60, 66, 73, 80, 88, 97, 107, 118, 130, 143, 157, 173, 190, 209, 230, 
  253, 279, 307, 337, 371, 408, 449, 494, 544, 598, 658, 724, 796, 876, 963, 
  1060, 1166, 1282, 1411, 1552, 1707, 1878, 2066, 2272, 2499, 2749, 3024, 3327, 
  3660, 4026, 4428, 4871, 5358, 5894, 6484, 7132, 7845, 8630, 9493, 10442, 11487, 
  12635, 13899, 15289, 16818, 18500, 20350, 22385, 24623, 27086, 29794, 32767 
};

// morse_table layout: indices 0-9 = digits '0'-'9',
//                     indices 10-16 = (unused/special chars),
//                     indices 17-42 = letters 'A'-'Z'
#define MORSE_LETTER_OFFSET 17
// 1000 Hz sine-wave: 32 samples/cycle, one sample every 31.25 µs → ~1000 Hz
#define SINE_SAMPLE_US      31
// Pre-computed dot/dash sample counts (avoids division inside time-critical loop)
#define DOT_SAMPLES  ((200 * 1000) / SINE_SAMPLE_US)   // ~6451
#define DASH_SAMPLES ((600 * 1000) / SINE_SAMPLE_US)   // ~19354
const uint8_t sine_table[32] = {
  128, 148, 166, 184, 199, 211, 220, 226, 228, 226, 220, 211,
  199, 184, 166, 148, 128, 108,  90,  72,  57,  45,  36,  30,
   28,  30,  36,  45,  57,  72,  90, 108
};

int predictor = 0;
int step_index = 0;

// --- HELPERS ---

String getStorageInfo() {
  size_t total = LittleFS.totalBytes();
  size_t used = LittleFS.usedBytes();
  float p = ((float)used / (float)total) * 100.0;
  return String(p, 1) + "% Used (" + String(used/1024) + "KB / " + String(total/1024) + "KB)";
}

void savePlaylistToDisk() {
  File file = LittleFS.open("/playlist.json", "w");
  if (!file) return;
  JsonDocument doc;
  JsonArray arr = doc.to<JsonArray>();
  for (String s : playlist) arr.add(s);
  serializeJson(doc, file);
  file.close();
}

void loadPlaylistFromDisk() {
  if (!LittleFS.exists("/playlist.json")) return;
  File file = LittleFS.open("/playlist.json", "r");
  JsonDocument doc;
  deserializeJson(doc, file);
  playlist.clear();
  for (JsonVariant v : doc.as<JsonArray>()) playlist.push_back(v.as<String>());
  file.close();
}

void apply_ramp(int target, int ms) {
  static int current = 0;
  int steps = 30; // More steps for smoother transition
  if (ms <= 0) ms = 1; 
  int total_diff = target - current;
  for (int i = 1; i <= steps; i++) {
    int next_val = current + (total_diff * i / steps);
    dacWrite(DAC_PIN, (uint8_t)constrain(next_val, 0, 255));
    // Use delayMicroseconds for sub-millisecond ramps to avoid 8Hz pulse
    delayMicroseconds((ms * 1000) / steps);
  }
  current = target;
}

void decodeAndOutput(uint8_t n) {
  int step = step_table[step_index];
  int diff = step >> 3;
  if (n & 4) diff += step; 
  if (n & 2) diff += (step >> 1); 
  if (n & 1) diff += (step >> 2);
  if (n & 8) predictor -= diff; else predictor += diff;
  predictor = constrain(predictor, -32768, 32767);
  step_index = constrain(step_index + index_table[n & 0x0F], 0, 88);
  
  int target_sample = (predictor + 32768) >> 8;
  int start_sample = last_sample;
  int total_change = target_sample - start_sample;

  for (int i = 1; i <= 4; i++) {
    int intermediate = start_sample + ((total_change * i) >> 2);
    int final_out = (int)(intermediate * gain_booster);
    dacWrite(DAC_PIN, (uint8_t)constrain(final_out, 0, 255));
    delayMicroseconds(42); 
  }
  last_sample = target_sample;
}

void sendMorse(String text) {
  text.toUpperCase();
  digitalWrite(PTT_PIN, HIGH);

  for (int i = 0; i < (int)text.length(); i++) {
    char c = text[i];
    int index;

    if (c == ' ') {
      // Word gap – extra silence (inter-char gap already follows last letter)
      dacWrite(DAC_PIN, 128);
      vTaskDelay(pdMS_TO_TICKS(600));
      continue;
    } else if (c >= '0' && c <= '9') {
      index = c - '0';
    } else if (c >= 'A' && c <= 'Z') {
      index = c - 'A' + MORSE_LETTER_OFFSET;
    } else {
      continue; // skip unsupported characters
    }

    const char* pattern = morse_table[index];
    for (int j = 0; j < (int)strlen(pattern); j++) {

      // Generate 1000 Hz sine-wave tone for dot/dash duration
      // 32 samples/cycle × SINE_SAMPLE_US µs/sample ≈ 1000 Hz
      int total_samples = (pattern[j] == '.') ? DOT_SAMPLES : DASH_SAMPLES;
      for (int s = 0; s < total_samples; s++) {
        dacWrite(DAC_PIN, sine_table[s % 32]);
        delayMicroseconds(SINE_SAMPLE_US);
      }

      // Inter-element silence
      dacWrite(DAC_PIN, 128);
      vTaskDelay(pdMS_TO_TICKS(200));
    }

    // Inter-character gap (additional silence after last element gap)
    dacWrite(DAC_PIN, 128);
    vTaskDelay(pdMS_TO_TICKS(400));
  }

  dacWrite(DAC_PIN, 128);
  digitalWrite(PTT_PIN, LOW);
}

void playNextItem() {
  if (playlist.empty() || !isRunning) return;
  String item = playlist[currentIndex];

  if (item.startsWith("M:")) {
    sendMorse(item.substring(2));   // sendMorse handles PTT internally
  } else if (item.toInt() > 0) {
    dacWrite(DAC_PIN, 128);         // Silence – no PTT
    // Use delay() which is an alias for vTaskDelay to let Core 0 work
    delay(item.toInt() * 1000);
  } else {
    File f = LittleFS.open("/" + item, "r");
    if (f) {
      digitalWrite(PTT_PIN, HIGH);  // Assert PTT before audio
      apply_ramp(128, 15);
      f.seek(44);
      predictor = 0;
      step_index = 0;

      while (f.available() && isRunning) {
        // Refill the buffer if it's empty
        if (bufferPointer >= bytesInBuffer) {
          bytesInBuffer = f.read(audioBuffer, BUFFER_SIZE);
          bufferPointer = 0;
          if (bytesInBuffer == 0) break; // End of file
        }

        uint8_t b = audioBuffer[bufferPointer++];
        
        decodeAndOutput(b & 0x0F); 
        decodeAndOutput(b >> 4);   

        // Stability: Poke the watchdog every few samples without a delay
        static int wdtCounter = 0;
        if (wdtCounter++ > 100) {
          esp_task_wdt_reset(); 
          wdtCounter = 0;
        }
      }
      f.close();
      dacWrite(DAC_PIN, 128);        // Return to silence before releasing PTT
      digitalWrite(PTT_PIN, LOW);   // Release PTT after audio
    }
  }
  currentIndex = (currentIndex + 1) % playlist.size();
}


void audioTask(void * pvParameters) {
  for(;;) {
    if (isRunning) {
      playNextItem();
    } else {
      // Longer delay when idle to let WebUI be snappy
      vTaskDelay(100 / portTICK_PERIOD_MS); 
    }
    // Final safety yield
    yield();
  }
}

void handleUpload(AsyncWebServerRequest *request, String filename, size_t index, uint8_t *data, size_t len, bool final) {
  if (!index) request->_tempFile = LittleFS.open("/" + filename, "w");
  if (len && request->_tempFile) request->_tempFile.write(data, len);
  if (final && request->_tempFile) request->_tempFile.close();
}

const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE HTML><html><head><title>Fox Hunt 1.4</title>
<style>
  body { background:#000; color:#0f0; font-family:'Courier New', monospace; padding:20px; }
  h2 { border-bottom: 2px solid #040; }
  .box { margin:15px 0; padding:15px; border:1px solid #0f0; background:#010; }
  .item { display:flex; align-items:center; background:#010; border:1px solid #030; margin:5px 0; padding:5px; }
  button { background:#020; color:#0f0; border:1px solid #0f0; cursor:pointer; padding:6px; margin: 2px; }
  button:hover { background:#0f0; color:#000; }
  input { background:#000; color:#0f0; border:1px solid #0f0; padding:5px; }
  .save-btn { background:#040; border-color:#0f0; font-weight:bold; width:100%; font-size:16px; margin:10px 0; }
</style></head><body>
  <div id="mem" style="border:1px solid #040; padding:10px;">DISK: ...</div>
  <h2>[ PLAYLIST ]</h2><div id="playlist"></div>
  <button class="save-btn" onclick="saveAndPlay()">SAVE & BROADCAST</button>
  <div class="box">[ MORSE ] <input id="mmsg"><button onclick="addM()">ADD</button></div>
  <div class="box">[ UPLOAD ] <input type="file" id="fi"><button onclick="up()">GO</button><div id="prg"></div></div>
  <div class="box">[ SILENCE ] <input type="number" id="sec"><button onclick="addS()">ADD</button></div>
  <h2>[ FILES ]</h2><div id="files"></div>
  <button style="color:red; border-color:red;" onclick="clearPL()">WIPE PLAYLIST</button>
<script>
  function load() {
    fetch('/api/status').then(r=>r.json()).then(d=>{
      document.getElementById('mem').innerText = d.storage;
      let p = ''; d.playlist.forEach((f,i)=> p += `<div class="item"><button onclick="act('move',${i},-1)">▲</button><button onclick="act('move',${i},1)">▼</button><button onclick="act('remove',${i},0)">X</button> ${f}</div>`);
      document.getElementById('playlist').innerHTML = p || 'EMPTY';
      let f = ''; d.files.forEach(n => f += `<div class="item"><button onclick="act('add','${n}')">ADD</button> ${n} <button onclick="act('del-disk','${n}')">DEL</button></div>`);
      document.getElementById('files').innerHTML = f;
    });
  }
  function act(a,i,d=0) { fetch(`/api/${a}?i=${i}&d=${d}&f=${i}`).then(load); }
  function addM() { let m=document.getElementById('mmsg').value; if(m) fetch('/api/add?f=M:'+m).then(load); }
  function addS() { let s=document.getElementById('sec').value; if(s) fetch('/api/add?f='+s).then(load); }
  function saveAndPlay() { fetch('/api/save-play').then(load); alert('Program Saved & Broadcast Started!'); }
  function up() {
    let fi=document.getElementById('fi'); if(fi.files.length==0) return;
    let fd=new FormData(); fd.append("data", fi.files[0], fi.files[0].name);
    let x=new XMLHttpRequest(); x.open("POST","/api/upload");
    x.upload.onprogress=(e)=>document.getElementById('prg').innerText=Math.round(e.loaded/e.total*100)+"%";
    x.onload=load; x.send(fd);
  }
  function clearPL() { if(confirm('Wipe list?')) fetch('/api/clear').then(load); }
  load();
</script></body></html>)rawliteral";

void setup() {
  Serial.begin(115200);
  setCpuFrequencyMhz(240);
  pinMode(PLAY_PIN, INPUT_PULLUP);
  pinMode(STOP_PIN, INPUT_PULLUP);
  if(!LittleFS.begin(true)) return;
  loadPlaylistFromDisk(); 
  
  WiFi.softAP(ssid, password);
  //sleep(false);
  pinMode(PTT_PIN, OUTPUT);
  digitalWrite(PTT_PIN, LOW);
  dacWrite(DAC_PIN, 128);  // Idle level = midpoint (silence)
  

  server.on("/", HTTP_GET, [](AsyncWebServerRequest *r){ r->send(200, "text/html", index_html); });
  server.on("/api/status", HTTP_GET, [](AsyncWebServerRequest *r){
    JsonDocument doc;
    doc["storage"] = getStorageInfo();
    JsonArray pl = doc["playlist"].to<JsonArray>();
    for(String s : playlist) pl.add(s);
    JsonArray fs = doc["files"].to<JsonArray>();
    File root = LittleFS.open("/");
    File file = root.openNextFile();
    while(file){ String n=file.name(); if(!n.endsWith(".json")) fs.add(n); file=root.openNextFile(); }
    String out; serializeJson(doc, out);
    r->send(200, "application/json", out);
  });

  server.on("/api/add", HTTP_GET, [](AsyncWebServerRequest *r){ if(r->hasParam("f")) playlist.push_back(r->getParam("f")->value()); r->send(200); });
  server.on("/api/move", HTTP_GET, [](AsyncWebServerRequest *r){
    int i = r->getParam("i")->value().toInt();
    int d = r->getParam("d")->value().toInt();
    if(i+d >= 0 && i+d < (int)playlist.size()) std::swap(playlist[i], playlist[i+d]);
    r->send(200);
  });
  server.on("/api/remove", HTTP_GET, [](AsyncWebServerRequest *r){ int i=r->getParam("i")->value().toInt(); if(i<(int)playlist.size()) playlist.erase(playlist.begin()+i); r->send(200); });
  server.on("/api/del-disk", HTTP_GET, [](AsyncWebServerRequest *r){ if(r->hasParam("f")) LittleFS.remove("/"+r->getParam("f")->value()); r->send(200); });
  server.on("/api/clear", HTTP_GET, [](AsyncWebServerRequest *r){ playlist.clear(); r->send(200); });
  server.on("/api/save-play", HTTP_GET, [](AsyncWebServerRequest *r){ savePlaylistToDisk(); isRunning = true; currentIndex = 0; r->send(200); });
  server.on("/api/upload", HTTP_POST, [](AsyncWebServerRequest *r){ r->send(200); }, handleUpload);
  
  server.begin();
  
  // Starting the task on Core 1
  // this allows the IDLE task (watchdog feeder) to sneak in
  xTaskCreatePinnedToCore(audioTask, "AudioTask", 10000, NULL, 1, &AudioTaskHandle, 1);
}

void loop() {
  if (digitalRead(PLAY_PIN) == LOW) isRunning = true;
  if (digitalRead(STOP_PIN) == LOW) { isRunning = false; dacWrite(DAC_PIN, 128); digitalWrite(PTT_PIN, LOW); }
  vTaskDelay(10 / portTICK_PERIOD_MS); 
}
