#include <Arduino.h>
#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <LittleFS.h>
#include <ArduinoJson.h>
#include <vector>
#include <esp_task_wdt.h>

// --- CONFIGURATION ---
const char* ssid = "FOX_HUNT_AM_TRANSMITTER";
const char* password = "foxhunt123";

#define DAC_PIN     25
#define PTT_PIN     27
#define PLAY_PIN    14
#define STOP_PIN    13
#define BUFFER_SIZE 512

uint8_t audioBuffer[BUFFER_SIZE];
int bufferPointer = 0;
int bytesInBuffer = 0;

// --- GLOBALS ---
AsyncWebServer server(80);
std::vector<String> playlist;
int currentIndex = 0;
bool isRunning = false;
float gain_booster = 1.15f;        // Slightly lower to reduce clipping
TaskHandle_t AudioTaskHandle = nullptr;

const char* morse_table[] = { /* unchanged */ };

// ... (keep your sine_table, step_table, index_table - I'll suggest a fix below)

int predictor = 0;
int step_index = 0;

// Fixed & complete IMA-ADPCM index table (standard)
const int8_t index_table[16] = {
  -1, -1, -1, -1, 2, 4, 6, 8,
  -1, -1, -1, -1, 2, 4, 6, 8
};

// === IMPROVED DECODE FUNCTION (faster + less jitter) ===
void decodeAndOutput(uint8_t nibble) {
  int step = step_table[step_index];
  int diff = step >> 3;

  if (nibble & 4) diff += step;
  if (nibble & 2) diff += step >> 1;
  if (nibble & 1) diff += step >> 2;
  if (nibble & 8) predictor -= diff;
  else            predictor += diff;

  predictor = constrain(predictor, -32768, 32767);
  step_index = constrain(step_index + index_table[nibble & 0x0F], 0, 88);

  int sample = (predictor + 32768) >> 8;
  int output = constrain((int)(sample * gain_booster), 0, 255);

  dacWrite(DAC_PIN, (uint8_t)output);
}

// === IMPROVED MORSE (non-blocking where possible) ===
void sendMorse(const String& text) {
  String upper = text;
  upper.toUpperCase();

  digitalWrite(PTT_PIN, HIGH);
  apply_ramp(128, 10);   // gentle ramp up

  for (char c : upper) {
    if (!isRunning) break;

    if (c == ' ') {
      dacWrite(DAC_PIN, 128);
      vTaskDelay(pdMS_TO_TICKS(700));
      continue;
    }

    int idx = -1;
    if (c >= '0' && c <= '9') idx = c - '0';
    else if (c >= 'A' && c <= 'Z') idx = c - 'A' + MORSE_LETTER_OFFSET;

    if (idx < 0) continue;

    const char* pattern = morse_table[idx];
    for (size_t j = 0; pattern[j] != '\0'; ++j) {
      if (!isRunning) break;

      int samples = (pattern[j] == '.') ? DOT_SAMPLES : DASH_SAMPLES;

      for (int s = 0; s < samples; ++s) {
        if (!isRunning) break;
        dacWrite(DAC_PIN, sine_table[s % 32]);
        delayMicroseconds(SINE_SAMPLE_US);
      }

      // Element space
      dacWrite(DAC_PIN, 128);
      vTaskDelay(pdMS_TO_TICKS(200));
    }

    // Character space
    vTaskDelay(pdMS_TO_TICKS(400));
  }

  apply_ramp(128, 15);
  digitalWrite(PTT_PIN, LOW);
}

// === PLAY NEXT ITEM (cleaner) ===
void playNextItem() {
  if (playlist.empty() || !isRunning) return;

  String item = playlist[currentIndex];

  if (item.startsWith("M:")) {
    sendMorse(item.substring(2));
  } 
  else if (item.toInt() > 0) {
    // Silence period
    dacWrite(DAC_PIN, 128);
    digitalWrite(PTT_PIN, LOW);
    delay(item.toInt() * 1000UL);
  } 
  else {
    // WAV file
    File f = LittleFS.open("/" + item, "r");
    if (!f) { currentIndex = (currentIndex + 1) % playlist.size(); return; }

    digitalWrite(PTT_PIN, HIGH);
    apply_ramp(128, 12);
    f.seek(44);                     // Skip WAV header
    predictor = 0;
    step_index = 0;

    while (f.available() && isRunning) {
      if (bufferPointer >= bytesInBuffer) {
        bytesInBuffer = f.read(audioBuffer, BUFFER_SIZE);
        bufferPointer = 0;
        if (bytesInBuffer == 0) break;
      }

      uint8_t b = audioBuffer[bufferPointer++];
      decodeAndOutput(b & 0x0F);
      decodeAndOutput(b >> 4);

      // Much lighter watchdog reset
      static uint32_t wdtLast = 0;
      if (millis() - wdtLast > 50) {
        esp_task_wdt_reset();
        wdtLast = millis();
      }
    }

    f.close();
    apply_ramp(128, 15);
    digitalWrite(PTT_PIN, LOW);
  }

  currentIndex = (currentIndex + 1) % playlist.size();
}

// === AUDIO TASK (higher priority) ===
void audioTask(void *pvParameters) {
  esp_task_wdt_add(nullptr);        // Add this task to watchdog

  while (true) {
    if (isRunning) {
      playNextItem();
    } else {
      vTaskDelay(pdMS_TO_TICKS(50));
    }
    taskYIELD();
  }
}

// Rest of your code (setup, web handlers, HTML, etc.) stays mostly the same.
// Just add this in setup():

void setup() {
  Serial.begin(115200);
  setCpuFrequencyMhz(240);

  pinMode(PLAY_PIN, INPUT_PULLUP);
  pinMode(STOP_PIN, INPUT_PULLUP);
  pinMode(PTT_PIN, OUTPUT);
  digitalWrite(PTT_PIN, LOW);
  dacWrite(DAC_PIN, 128);

  if (!LittleFS.begin(true)) {
    Serial.println("LittleFS Mount Failed");
    return;
  }

  loadPlaylistFromDisk();

  WiFi.softAP(ssid, password);

  // === Web server routes (unchanged except minor improvements) ===

  // ... your server.on() calls ...

  server.begin();

  // Configure watchdog and start audio task on Core 1 with higher priority
  esp_task_wdt_config_t wdt_config = {
    .timeout_ms = 5000,
    .idle_core_mask = 0,
    .trigger_panic = true
  };
  esp_task_wdt_init(&wdt_config);
  esp_task_wdt_add(nullptr);

  xTaskCreatePinnedToCore(audioTask, "Audio", 12288, NULL, 2, &AudioTaskHandle, 1);  // Priority 2
}
