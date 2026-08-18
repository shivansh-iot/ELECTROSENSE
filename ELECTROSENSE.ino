// ============================================
// ELECTROSENSE - Firmware v2.6 (Non-Blocking State Machine + Onboard LED)
// ============================================

#include <Wire.h>
#include <Adafruit_INA219.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <WiFi.h>
#include <WebSocketsServer.h>
#include <ELECTROSENCE_inferencing.h>

// ─────────────────────────────────
//  PIN CONFIG
// ─────────────────────────────────
#define DISCHARGE_PIN  25
#define BUZZER_PIN     26
#define LED_PIN         2   

// ─────────────────────────────────
//  OLED CONFIG
// ─────────────────────────────────
#define SCREEN_WIDTH  128
#define SCREEN_HEIGHT  64
#define OLED_RESET     -1
#define OLED_ADDRESS  0x3C
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// ─────────────────────────────────
//  OBJECTS
// ─────────────────────────────────
Adafruit_INA219  ina219;
WebSocketsServer webSocket = WebSocketsServer(81);

// ─────────────────────────────────
//  WIFI
// ─────────────────────────────────
const char* ssid     = "OPPO A78 5G";
const char* password = "@shivansh07";

// ─────────────────────────────────
//  THRESHOLDS & TIMINGS
// ─────────────────────────────────
#define DYNAMIC_RISE_DELTA   0.02f   // V  — Minimum positive delta to trigger capture
#define DYNAMIC_SETTLE_DELTA 0.015f  // V  — Noise floor for settled state
#define SETTLE_COUNT_REQ     3       // Consecutive quiet samples required to settle
#define DISCHARGE_TARGET_V   0.02f   // Target residual voltage for safe zeroing
#define DISCHARGE_MAX_MS     4000    // Max discharge window safety guard

#define BUZZ_ON_MS          300
#define BUZZ_OFF_MS         200
#define RESULT_DISPLAY_MS   3000

#define LED_HEARTBEAT_MS     500     // waiting state blink rate
#define LED_DISCHARGE_MS     150     // discharging state blink rate

// ─────────────────────────────────
//  CAPTURE BUFFER (for slow, readable serial print)
// ─────────────────────────────────
#define MAX_CAP_SAMPLES 30
unsigned long capT[MAX_CAP_SAMPLES];
float capV[MAX_CAP_SAMPLES], capI[MAX_CAP_SAMPLES], capP[MAX_CAP_SAMPLES], capD[MAX_CAP_SAMPLES];
int capCount = 0;

// ─────────────────────────────────
//  STATE MACHINE
// ─────────────────────────────────
enum SystemState {
  ST_WAITING,
  ST_CHARGING,
  ST_BUZZING,
  ST_RESULT_DISPLAY,
  ST_DISCHARGING
};

SystemState state = ST_WAITING;
unsigned long stateTimer = 0;
int  buzzCount = 0;
bool buzzOn    = false;

// LED state
bool ledOn = false;
unsigned long ledTimer = 0;

// ─────────────────────────────────
//  GLOBAL STATE
// ─────────────────────────────────
String currentLabel  = "waiting";
bool   chargeStarted = false;
bool   measureDone   = false;
float  lastVoltage   = 0.0f;
int    settleCount   = 0;

float  eiTopConf  = 0.0f;
String eiTopLabel = "";

// Sensor Readings
float gVoltage = 0.0f;
float gCurrent = 0.0f;
float gPower   = 0.0f;

float ei_buffer[EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE] = {0.0f};
int   ei_index = 0;

// ─────────────────────────────────
//  OLED DISPLAY
// ─────────────────────────────────
void updateOLED(float v, float i, float p, String label) {
  display.clearDisplay();
  display.setTextColor(WHITE);
  display.setTextSize(1);

  display.setCursor(0, 0);
  display.println(" ELECTROSENSE v2.6");
  display.drawLine(0, 9, 128, 9, WHITE);

  display.setCursor(0, 13);
  display.print("V: "); display.print(v, 2); display.println(" V");
  display.setCursor(0, 23);
  display.print("I: "); display.print(i, 2); display.println(" mA");
  display.setCursor(0, 33);
  display.print("P: "); display.print(p, 2); display.println(" mW");

  display.drawLine(0, 43, 128, 43, WHITE);

  display.setCursor(0, 47);
  if (label == "healthy" || label == "HEALTHY") {
    display.println("STATUS: HEALTHY OK");
  } else if (label == "faulty" || label == "FAULTY") {
    display.println("!! FAULTY DETECTED !!");
  } else if (label == "no_capacitor" || label == "NO CAPACITOR") {
    display.println("No Capacitor Found");
  } else {
    display.println("  Measuring...");
  }

  display.setCursor(0, 57);
  if (eiTopLabel != "") {
    String eiLine = "EI:" + String(eiTopConf * 100.0f, 1) + "% " + eiTopLabel;
    display.println(eiLine);
  }

  display.display();
}

// ─────────────────────────────────
//  EDGE IMPULSE INFERENCE
// ─────────────────────────────────
String runEdgeImpulse() {
  Serial.println("\n===== EI INFERENCE START =====");
  Serial.printf("Captured Samples Frame Index: %d / %d\n", ei_index, EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE);

  float finalV = (ei_index >= 4) ? ei_buffer[ei_index - 4] : gVoltage;

  // Align feature buffer to 4-element frame alignment
  while (ei_index % 4 != 0 && ei_index < EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE) {
    ei_buffer[ei_index++] = 0.0f;
  }

  // Feature-Aware Steady State Padding
  while (ei_index < EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE) {
    int slot = ei_index % 4;
    if      (slot == 0) ei_buffer[ei_index++] = finalV;  // Voltage plateau
    else if (slot == 1) ei_buffer[ei_index++] = 0.0f;     // Current decay
    else if (slot == 2) ei_buffer[ei_index++] = 0.0f;     // Power decay
    else                ei_buffer[ei_index++] = 0.0f;     // Delta V = 0
  }

  signal_t signal;
  signal.total_length = EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE;
  signal.get_data = [](size_t offset, size_t len, float* out) -> int {
    memcpy(out, ei_buffer + offset, len * sizeof(float));
    return 0;
  };

  ei_impulse_result_t result;
  EI_IMPULSE_ERROR err = run_classifier(&signal, &result, false);

  if (err != EI_IMPULSE_OK) {
    Serial.printf("# ERROR: EI classifier failed (%d)\n", err);
    return "ei_error";
  }

  float  maxConf  = 0.0f;
  String maxLabel = "unknown";
  for (int c = 0; c < EI_CLASSIFIER_LABEL_COUNT; c++) {
    if (result.classification[c].value > maxConf) {
      maxConf  = result.classification[c].value;
      maxLabel = String(result.classification[c].label);
    }
  }

  eiTopConf  = maxConf;
  eiTopLabel = maxLabel;

  Serial.println("========== Classification Results ==========");
  for (int c = 0; c < EI_CLASSIFIER_LABEL_COUNT; c++) {
    Serial.printf("  %-15s : %.2f%%\n", result.classification[c].label, result.classification[c].value * 100.0f);
  }
  Serial.printf("# Selected: %s (%.1f%%)\n", maxLabel.c_str(), maxConf * 100.0f);

  return maxLabel;
}

// ─────────────────────────────────
//  WEBSOCKET BROADCAST
// ─────────────────────────────────
void broadcastData(float v, float i, float p, String label) {
  String json = "{";
  json += "\"voltage\":"  + String(v, 2) + ",";
  json += "\"current\":"  + String(i, 2) + ",";
  json += "\"power\":"    + String(p, 2) + ",";
  json += "\"label\":\""  + label + "\"";
  json += "}";
  webSocket.broadcastTXT(json);
}

// ─────────────────────────────────
//  ONBOARD LED HANDLER (non-blocking)
// ─────────────────────────────────
void handleLED(unsigned long now) {
  switch (state) {

    case ST_WAITING:
      // slow heartbeat blink
      if (now - ledTimer >= LED_HEARTBEAT_MS) {
        ledTimer = now;
        ledOn = !ledOn;
        digitalWrite(LED_PIN, ledOn ? HIGH : LOW);
      }
      break;

    case ST_CHARGING:
    case ST_RESULT_DISPLAY:
      // solid ON — actively capturing / showing result
      digitalWrite(LED_PIN, HIGH);
      break;

    case ST_BUZZING:
      // mirror buzzer state (buzzOn is updated in handleStateMachine)
      digitalWrite(LED_PIN, buzzOn ? HIGH : LOW);
      break;

    case ST_DISCHARGING:
      // fast blink while discharging
      if (now - ledTimer >= LED_DISCHARGE_MS) {
        ledTimer = now;
        ledOn = !ledOn;
        digitalWrite(LED_PIN, ledOn ? HIGH : LOW);
      }
      break;
  }
}

// ─────────────────────────────────
//  STATE MACHINE HANDLER
//  (called once per sample, ~30Hz — no blocking anywhere)
// ─────────────────────────────────
void handleStateMachine(float deltaV, unsigned long now) {
  switch (state) {

    case ST_WAITING:
      if (gVoltage < 0.05f) {
        ei_index      = 0;
        measureDone   = false;
        chargeStarted = false;
      }
      if (deltaV >= DYNAMIC_RISE_DELTA || gCurrent > 0.5f) {
        chargeStarted = true;
        ei_index      = 0;
        settleCount   = 0;
        capCount      = 0;     // reset readable-capture buffer
        state = ST_CHARGING;
        Serial.println("# Charging event detected. Capturing frame...");
      }
      break;

    case ST_CHARGING:
      if (ei_index <= EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE - 4) {
        ei_buffer[ei_index++] = gVoltage;
        ei_buffer[ei_index++] = gCurrent;
        ei_buffer[ei_index++] = gPower;
        ei_buffer[ei_index++] = deltaV;

        // Mirror into readable-capture buffer (for slow print later)
        if (capCount < MAX_CAP_SAMPLES) {
          capT[capCount] = now;
          capV[capCount] = gVoltage;
          capI[capCount] = gCurrent;
          capP[capCount] = gPower;
          capD[capCount] = deltaV;
          capCount++;
        }
      }

      if (fabsf(deltaV) < DYNAMIC_SETTLE_DELTA) settleCount++;
      else settleCount = 0;

      if (settleCount >= SETTLE_COUNT_REQ || ei_index >= EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE) {
        settleCount   = 0;
        measureDone   = true;
        chargeStarted = false;

        // ── Readable slow-print of captured cycle ──
        Serial.println("\n----- Captured Cycle (readable) -----");
        Serial.println("timestamp,voltage,current,power,deltaV");
        for (int k = 0; k < capCount; k++) {
          Serial.print(capT[k]);      Serial.print(",");
          Serial.print(capV[k], 4);   Serial.print(",");
          Serial.print(capI[k], 4);   Serial.print(",");
          Serial.print(capP[k], 4);   Serial.print(",");
          Serial.println(capD[k], 4);

          // Keep dashboard + LED alive during this readable pause
          webSocket.loop();
          broadcastData(capV[k], capI[k], capP[k], "capturing");
          delay(100);   // slows serial print for readability only
        }
        Serial.println("--------------------------------------\n");

        currentLabel = runEdgeImpulse();
        updateOLED(gVoltage, gCurrent, gPower, currentLabel);

        Serial.println();
        Serial.println("############################################");
        Serial.printf("###   RESULT: %-30s ###\n", currentLabel.c_str());
        Serial.println("############################################");
        Serial.println();

        if (currentLabel == "faulty" || currentLabel == "FAULTY") {
          buzzCount = 3;
        } else if (currentLabel == "no_capacitor" || currentLabel == "NO CAPACITOR") {
          buzzCount = 1;
        } else {
          buzzCount = 0;
        }

        buzzOn = false;
        stateTimer = now;
        state = (buzzCount > 0) ? ST_BUZZING : ST_RESULT_DISPLAY;
      }
      break;

    case ST_BUZZING:
      if (buzzCount > 0) {
        if (!buzzOn && (now - stateTimer >= BUZZ_OFF_MS)) {
          digitalWrite(BUZZER_PIN, HIGH);
          buzzOn = true;
          stateTimer = now;
        } else if (buzzOn && (now - stateTimer >= BUZZ_ON_MS)) {
          digitalWrite(BUZZER_PIN, LOW);
          buzzOn = false;
          stateTimer = now;
          buzzCount--;
        }
      } else {
        stateTimer = now;
        state = ST_RESULT_DISPLAY;
      }
      break;

    case ST_RESULT_DISPLAY:
      if (now - stateTimer >= RESULT_DISPLAY_MS) {
        Serial.println("# Active Smart Discharge Initiated...");
        updateOLED(0, 0, 0, "waiting");
        digitalWrite(DISCHARGE_PIN, HIGH);
        stateTimer = now;
        state = ST_DISCHARGING;
      }
      break;

    case ST_DISCHARGING:
      if (gVoltage <= DISCHARGE_TARGET_V || (now - stateTimer >= DISCHARGE_MAX_MS)) {
        digitalWrite(DISCHARGE_PIN, LOW);
        lastVoltage = gVoltage;
        Serial.printf("# Discharge complete. Residual V: %.4fV\n", lastVoltage);

        // System State Reset
        ei_index      = 0;
        measureDone   = false;
        chargeStarted = false;
        settleCount   = 0;
        currentLabel  = "waiting";
        state = ST_WAITING;
      }
      break;
  }
}

// ─────────────────────────────────
//  SETUP
// ─────────────────────────────────
void setup() {
  Serial.begin(115200);
  delay(500);

  pinMode(DISCHARGE_PIN, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(DISCHARGE_PIN, LOW);
  digitalWrite(BUZZER_PIN, LOW);
  digitalWrite(LED_PIN, LOW);

  if (!ina219.begin()) {
    Serial.println("ERROR: INA219 not found!");
    while (1);
  }

  if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDRESS)) {
    Serial.println("ERROR: OLED not found!");
    while (1);
  }
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(20, 20);
  display.println("ELECTROSENSE");
  display.setCursor(30, 35);
  display.println("v2.6 Ready");
  display.display();

  Serial.println("Connecting WiFi...");
  WiFi.begin(ssid, password);
  int tries = 0;
  while (WiFi.status() != WL_CONNECTED && tries < 20) {
    delay(500); Serial.print(".");
    digitalWrite(LED_PIN, !digitalRead(LED_PIN)); // blink while connecting
    tries++;
  }
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\nWiFi Connected! IP: " + WiFi.localIP().toString());
  } else {
    Serial.println("\nWiFi Offline Mode");
  }

  webSocket.begin();

  // Initial short buzz + LED flash (blocking here is safe — boot only)
  for (int b = 0; b < 2; b++) {
    digitalWrite(BUZZER_PIN, HIGH);
    digitalWrite(LED_PIN, HIGH);
    delay(150);
    digitalWrite(BUZZER_PIN, LOW);
    digitalWrite(LED_PIN, LOW);
    delay(100);
  }

  // Initial discharge — done once at boot, blocking acceptable
  // (dashboard has nothing to show yet)
  digitalWrite(DISCHARGE_PIN, HIGH);
  unsigned long startMs = millis();
  while (millis() - startMs < DISCHARGE_MAX_MS) {
    float vCheck = ina219.getBusVoltage_V();
    digitalWrite(LED_PIN, (millis() / 150) % 2); // fast blink during boot discharge
    if (vCheck <= DISCHARGE_TARGET_V) break;
    delay(10);
  }
  digitalWrite(DISCHARGE_PIN, LOW);
  digitalWrite(LED_PIN, LOW);
  delay(50);
  lastVoltage = ina219.getBusVoltage_V();
  Serial.printf("# Initial discharge complete. Residual V: %.4fV\n", lastVoltage);

  ledTimer = millis();
  state = ST_WAITING;
}

// ─────────────────────────────────
//  MAIN LOOP  (fully non-blocking)
// ─────────────────────────────────
void loop() {
  webSocket.loop();

  static unsigned long lastSample    = 0;
  static unsigned long lastBroadcast = 0;
  unsigned long now = millis();

  // Dashboard Broadcast — runs continuously, every 150ms,
  // regardless of what state the system is in.
  if (now - lastBroadcast >= 150) {
    lastBroadcast = now;
    broadcastData(gVoltage, gCurrent, gPower, currentLabel);
  }

  // Onboard LED — updated every loop pass (non-blocking, cheap)
  handleLED(now);

  // Sensor Sampling Loop (~30 Hz) — accuracy-critical, untouched
  if (now - lastSample >= 33) {
    lastSample = now;

    gVoltage      = ina219.getBusVoltage_V();
    gCurrent      = ina219.getCurrent_mA();
    gPower        = gVoltage * (gCurrent / 1000.0f) * 1000.0f;
    float deltaV  = gVoltage - lastVoltage;
    lastVoltage   = gVoltage;

    handleStateMachine(deltaV, now);
  }
}