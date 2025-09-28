/* ESP32 - POT + adaptive interaction (patched)
   Fixes applied:
   - rate-limit CPU freq changes, avoid boost on WS handshake
   - limit per-iteration reads from TCP clients
   - safer non-blocking writes (check availableForWrite)
   - keep sampling+led on core 1 (unchanged)
*/

#include <Arduino.h>
#include <WiFi.h>
#include <driver/ledc.h>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include <stdarg.h>
#include <string.h>

#include "sample_data.h"
#include "led_renderer.h"
#include "led_config.h"
#include "sampling_task.h"
#include "wifi_captive.h"
#include "ws_manager.h"
#include "power_manager.h"

#include <Preferences.h>

#if defined(ARDUINO_ARCH_ESP32)
#include "esp32-hal-cpu.h"
#endif

// ---------- CONFIG ----------
// No built-in default WiFi credentials present in source.
// The firmware will only attempt STA connect when valid credentials are stored
// in Preferences (prefs.getString("ssid","")) or when the captive portal
// provides credentials via the /save handler (pendingConnect -> WiFi.begin).

#ifndef SERIAL_ENABLED
#define SERIAL_ENABLED 1
#endif

#if SERIAL_ENABLED
void SLog(const char *fmt, ...) {
  char _slog_buf[256];
  va_list args;
  va_start(args, fmt);
  vsnprintf(_slog_buf, sizeof(_slog_buf), fmt, args);
  va_end(args);
  Serial.print(_slog_buf);
}
#else
inline void SLog(const char *fmt, ...) { (void)fmt; }
#endif

// ---------- CPU frequency scaling (idle cooling) ----------
#ifndef MIN_IDLE_CPU_MHZ
#define MIN_IDLE_CPU_MHZ 80   // closest supported frequency to requested 40MHz
#endif
#ifndef NORMAL_CPU_MHZ
#define NORMAL_CPU_MHZ 160    // original preferred speed
#endif

static int currentCpuMhz = 0;
volatile unsigned long lastCpuBoostTime = 0;
const unsigned long IDLE_CPU_TIMEOUT_MS = 3000UL; // after this of no boost-worthy activity, lower CPU

// --- FIX --- rate-limit CPU freq changes to avoid rapid toggles that destabilize Wi-Fi driver
const unsigned long MIN_CPU_CHANGE_MS = 150; // don't change CPU freq more often than this
static unsigned long lastCpuChangeMs = 0;

static inline void setCpuIfNeeded(int mhz) {
#if defined(ARDUINO_ARCH_ESP32)
  unsigned long now = millis();
  if (now - lastCpuChangeMs < MIN_CPU_CHANGE_MS) return; // too soon to change again

  int use;
  if (mhz >= 240) use = 240;
  else if (mhz >= 160) use = 160;
  else use = 80;
  if (currentCpuMhz == use) return;
  setCpuFrequencyMhz(use);
  currentCpuMhz = use;
  lastCpuChangeMs = now;
  SLog("CPU freq set to %d MHz\n", use);
#else
  (void)mhz;
#endif
}

void requestCpuNormal() {
  setCpuIfNeeded(NORMAL_CPU_MHZ);
}

void requestCpuIdle() {
  setCpuIfNeeded(MIN_IDLE_CPU_MHZ);
}

// LED pins
const int greenPin = 18; // GPIO18
const int bluePin  = 17; // GPIO17
const int redPin   = 16; // GPIO16

// LEDC config
const ledc_timer_t LEDC_TIMER = LEDC_TIMER_0;
const ledc_mode_t  LEDC_MODE  = LEDC_HIGH_SPEED_MODE;
const uint32_t     PWM_FREQ   = 5000;
const ledc_timer_bit_t PWM_RES = LEDC_TIMER_8_BIT; // 0..255
const ledc_channel_t GREEN_CH = LEDC_CHANNEL_0;
const ledc_channel_t BLUE_CH  = LEDC_CHANNEL_1;
const ledc_channel_t RED_CH   = LEDC_CHANNEL_2;

// POT
extern const int potPin = 36; // VP (ADC1)

// Interaction / Idle intervals
extern const unsigned long INTERACT_SAMPLE_MS   = 30;
const unsigned long INTERACT_BROAD_MS    = 120;
extern const unsigned long IDLE_SAMPLE_MS       = 300;
const unsigned long IDLE_BROAD_MS        = 1000;

// windows and thresholds
const float ACTIVITY_MIN_FOR_INTERACT = 0.05f;
const unsigned long INTERACT_SUSTAIN_MS = 1200UL;
const unsigned long INTERACT_MAX_MS = 30000UL;

// broadcast backoff protection
const int BASE_BROADCAST_THRESHOLD_RAW = 6;
const int BROADCAST_WINDOW_MS = 2000;
const int BROADCAST_MAX_PER_WINDOW = 18;
const float BACKOFF_FACTOR = 2.0f;
const unsigned long BACKOFF_RECOVERY_MS = 5000;

// wifi/fades
const unsigned long WIFI_CONNECT_TIMEOUT_MS = 10000;
const unsigned long GREEN_SHOW_MS = 500;

// BUTTON (D19)
#define BUTTON_PIN 19
const unsigned long BUTTON_DEBOUNCE_MS = 50;
const unsigned long BUTTON_LONG_PRESS_MS = 3000;

static int buttonReading = HIGH;
static int buttonStableState = HIGH;
static unsigned long lastButtonChangeMs = 0;
static unsigned long buttonPressStartMs = 0;
static bool buttonLongActionTriggered = false;

// servers
WiFiServer wsServer(81);
Preferences prefs;

// state (simple volatile flags)
volatile bool showPot = false;
volatile unsigned long connectedAt = 0;
volatile bool interacting = false;

// button state handled via buttonReading/buttonStableState

// ---------- sample single-slot queue ----------
QueueHandle_t sampleQueue = NULL;

// runtime (used by main loop)
unsigned long lastBroadcastTime = 0;
int lastBroadcastRaw = 0;

// adaptive runtime
unsigned long broadcastInterval = IDLE_BROAD_MS;
unsigned long lastInteractionMillis = 0;
unsigned long interactingSince = 0;


// broadcast counting for backoff
int broadcastCountWindow = 0;
unsigned long broadcastWindowStart = 0;
bool inBackoff = false;
unsigned long backoffStart = 0;

// ---------- setup ----------
void setup() {
  #if SERIAL_ENABLED
  Serial.begin(115200);
  delay(50);
  #endif
  SLog("ESP32 optimized startup\n");

  prefs.begin("airvol", false);

  // ensure CPU starts at normal speed for initial network attempts
  requestCpuNormal();
  lastCpuBoostTime = millis();

  ledc_timer_config_t ledc_timer;
  memset(&ledc_timer, 0, sizeof(ledc_timer));
  ledc_timer.speed_mode = LEDC_MODE;
  ledc_timer.duty_resolution = PWM_RES;
  ledc_timer.timer_num = LEDC_TIMER;
  ledc_timer.freq_hz = PWM_FREQ;
  ledc_timer.clk_cfg = LEDC_AUTO_CLK;
  ledc_timer_config(&ledc_timer);

  ledc_channel_config_t chcfg;
  memset(&chcfg, 0, sizeof(chcfg));
  chcfg.gpio_num = greenPin; chcfg.speed_mode = LEDC_MODE; chcfg.channel = GREEN_CH;
  chcfg.intr_type = LEDC_INTR_DISABLE; chcfg.timer_sel = LEDC_TIMER; chcfg.duty = 0; chcfg.hpoint = 0;
  ledc_channel_config(&chcfg);
  chcfg.gpio_num = bluePin; chcfg.channel = BLUE_CH; chcfg.duty = 0; ledc_channel_config(&chcfg);
  chcfg.gpio_num = redPin; chcfg.channel = RED_CH; chcfg.duty = 0; ledc_channel_config(&chcfg);

  ledc_set_duty(LEDC_MODE, GREEN_CH, 0);  ledc_update_duty(LEDC_MODE, GREEN_CH);
  ledc_set_duty(LEDC_MODE, BLUE_CH, 0);   ledc_update_duty(LEDC_MODE, BLUE_CH);
  ledc_set_duty(LEDC_MODE, RED_CH, 0);    ledc_update_duty(LEDC_MODE, RED_CH);

  startSamplingTask();

  unsigned long initNow = millis();
  powerManagerSetup(initNow, IDLE_BROAD_MS);

  wsInit();

  WiFi.setSleep(false);

  // button init (D19) - INPUT_PULLUP, button closes to GND when pressed
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  int initialButton = digitalRead(BUTTON_PIN);
  buttonReading = initialButton;
  buttonStableState = initialButton;
  lastButtonChangeMs = millis();
  buttonPressStartMs = 0;
  buttonLongActionTriggered = false;

  // ledTask su core 1 per evitare contention con Wi-Fi (core 0)
  xTaskCreatePinnedToCore(ledTask, "led", 3072, NULL, 3, NULL, 1);        // med prio -> core 1

  lastBroadcastTime = millis();
  broadcastWindowStart = millis();
}

// limit bytes read per client per loop to avoid hogging core 0
const int MAX_HS_READ_PER_ITER = 128;

// ---------- main loop ----------
void loop() {
  unsigned long now = millis();

  int reading = digitalRead(BUTTON_PIN);
  if (reading != buttonReading) {
    lastButtonChangeMs = now;
    buttonReading = reading;
  }

  if ((now - lastButtonChangeMs) >= BUTTON_DEBOUNCE_MS) {
    if (buttonReading != buttonStableState) {
      buttonStableState = buttonReading;
      if (buttonStableState == LOW) {
        buttonPressStartMs = now;
        buttonLongActionTriggered = false;
      } else {
        if (!buttonLongActionTriggered) {
          if (powerIsOn()) {
            powerTurnOff(now);
          } else {
            powerTurnOn(now);
          }
        }
        buttonPressStartMs = 0;
        SLog("Button D19 released -> ready for next press\n");
      }
    }

    if (buttonStableState == LOW && !buttonLongActionTriggered &&
        powerIsOn() && (now - buttonPressStartMs) >= BUTTON_LONG_PRESS_MS) {
      buttonLongActionTriggered = true;
      powerHandleLongPress(now);
    }
  }

  bool powered = powerIsOn();
  static bool offLogged = false;

  if (powered) {
    offLogged = false;

    wifiManageState(now);

    wsTick(now);

    Sample_t latest;
    bool haveSample = false;
    if (sampleQueue && xQueuePeek(sampleQueue, &latest, 0) == pdTRUE) haveSample = true;

    if (haveSample) {
      if (!interacting && latest.activity >= ACTIVITY_MIN_FOR_INTERACT) {
        interacting = true;
        interactingSince = now;
        broadcastInterval = INTERACT_BROAD_MS;
        SLog("Switched to INTERACT mode\n");
        requestCpuNormal();
        lastCpuBoostTime = now;
      } else if (interacting) {
        if ((now - lastInteractionMillis) > INTERACT_SUSTAIN_MS) {
          if ((now - interactingSince) > INTERACT_MAX_MS) {
            interacting = false;
            broadcastInterval = IDLE_BROAD_MS;
            SLog("Interact max timeout -> switch to IDLE\n");
            requestCpuIdle();
          } else {
            interacting = false;
            broadcastInterval = IDLE_BROAD_MS;
            SLog("Sustain expired -> switch to IDLE\n");
            requestCpuIdle();
          }
        }
      }
    }

    if (haveSample) {
      if (now - lastBroadcastTime >= broadcastInterval) {
        int potRaw = latest.raw;
        float activity = latest.activity;
        int deltaSinceLastBroadcast = abs(potRaw - lastBroadcastRaw);
        int adaptiveThreshold = BASE_BROADCAST_THRESHOLD_RAW + (int)((1.0f - activity) * 20.0f);
        if (adaptiveThreshold < 1) adaptiveThreshold = 1;
        bool significant = (deltaSinceLastBroadcast >= adaptiveThreshold);
        bool heartbeat = (now - lastBroadcastTime) >= (IDLE_BROAD_MS * 2);

        if (significant || heartbeat) {
          lastBroadcastTime = now;
          lastBroadcastRaw = potRaw;

          if (now - broadcastWindowStart >= (unsigned long)BROADCAST_WINDOW_MS) {
            broadcastWindowStart = now;
            broadcastCountWindow = 0;
            if (inBackoff && (now - backoffStart >= BACKOFF_RECOVERY_MS)) {
              inBackoff = false;
              broadcastInterval = interacting ? INTERACT_BROAD_MS : IDLE_BROAD_MS;
              SLog("Backoff recovered\n");
            }
          }

          broadcastCountWindow++;
          if (!inBackoff && broadcastCountWindow > BROADCAST_MAX_PER_WINDOW) {
            inBackoff = true;
            backoffStart = now;
            broadcastInterval = (unsigned long)((float)broadcastInterval * BACKOFF_FACTOR);
            if (broadcastInterval > (IDLE_BROAD_MS * 4)) broadcastInterval = IDLE_BROAD_MS * 4;
            SLog("Entering backoff\n");
          }

          char payload[160];
          int plen = snprintf(payload, sizeof(payload),
                              "{\"raw\":%d,\"voltage\":%.4f,\"percent\":%.3f,\"activity\":%.4f}",
                              latest.raw, latest.voltage, latest.percent, latest.activity);

          wsBroadcastPayload(now, payload, (size_t)plen);
        }
      }
    }

    if (currentCpuMhz > MIN_IDLE_CPU_MHZ) {
      bool safeToLower = !interacting && !wifiIsCaptiveActive() && !wifiHasPendingConnect();
      if (safeToLower && (now - lastCpuBoostTime >= IDLE_CPU_TIMEOUT_MS)) {
        requestCpuIdle();
      }
    }

    static unsigned long lastSerial = 0;
    if (now - lastSerial >= 800) {
      lastSerial = now;
      if (haveSample) {
        SLog("mode=%s act=%.4f bcast=%lums bwcnt=%d backoff=%d raw=%d v=%.3f pct=%.1f showPot=%d apActive=%d cpu=%d\n",
             interacting ? "INTERACT" : "IDLE",
             latest.activity, broadcastInterval, broadcastCountWindow, inBackoff ? 1 : 0,
             latest.raw, latest.voltage, latest.percent, showPot ? 1 : 0, wifiIsCaptiveActive() ? 1 : 0, currentCpuMhz);
      } else {
        SLog("no sample yet apActive=%d cpu=%d\n", wifiIsCaptiveActive() ? 1 : 0, currentCpuMhz);
      }
    }
  } else {
    if (!offLogged) {
      SLog("Device is OFF - short press to power ON\n");
      offLogged = true;
    }
  }

  vTaskDelay(pdMS_TO_TICKS(2));
}
