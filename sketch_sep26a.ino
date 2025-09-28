/* ESP32 - POT + adaptive interaction (patched)
   Fixes applied:
   - rate-limit CPU freq changes, avoid boost on WS handshake
   - limit per-iteration reads from TCP clients
   - safer non-blocking writes (check availableForWrite)
   - keep sampling+led on core 1 (unchanged)
*/

#include <Arduino.h>
#include <WiFi.h>
#include <WiFiClient.h>
#include <mbedtls/sha1.h>
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

// servers
WiFiServer wsServer(81);
Preferences prefs;

// state (simple volatile flags)
volatile bool showPot = false;
volatile unsigned long connectedAt = 0;
volatile bool interacting = false;

// button state (debounce)
static int lastButtonState = HIGH;            // INPUT_PULLUP => HIGH when not pressed
static unsigned long lastButtonChangeMs = 0;
static bool buttonHandled = false;

// ---------- sample single-slot queue ----------
QueueHandle_t sampleQueue = NULL;

// runtime (used by main loop)
unsigned long lastBroadcastTime = 0;
int lastBroadcastRaw = 0;

// adaptive runtime
unsigned long broadcastInterval = IDLE_BROAD_MS;
unsigned long lastInteractionMillis = 0;
unsigned long interactingSince = 0;

// WS clients
const int MAX_WS = 4;
WiFiClient wsClients[MAX_WS];
bool wsHandshakeDone[MAX_WS];

// per-client handshake buffer (non-blocking)
const size_t HS_BUF_SZ = 600;
char hsBuf[MAX_WS][HS_BUF_SZ];
size_t hsLen[MAX_WS];

// pending send buffers (one per client)
static uint8_t wsPendingBuf[MAX_WS][256];
static size_t  wsPendingLen[MAX_WS];
static size_t  wsPendingOff[MAX_WS];
static bool    wsHasPending[MAX_WS];
static unsigned long wsPendingSince[MAX_WS];
static unsigned long wsLastProgressMs[MAX_WS];
static unsigned long wsClientConnectedAt[MAX_WS];
static unsigned long wsLastClientActivity = 0;

const unsigned long WS_PENDING_STALL_MS = 6000UL;      // drop clients whose tx buffer cannot flush within 6s
const unsigned long WS_HANDSHAKE_TIMEOUT_MS = 4000UL;  // drop handshakes that never complete within 4s

static void wsResetClientState(int idx);
void wsDropClient(int idx, const char *reason);
void wsDropAllClients();
unsigned long wsGetLastClientActivity();

static inline void wsMarkActivity() {
  wsLastClientActivity = millis();
}

unsigned long wsGetLastClientActivity() {
  return wsLastClientActivity;
}

bool wsHasActiveClients() {
  for (int i = 0; i < MAX_WS; ++i) {
    if (wsClients[i] && wsClients[i].connected()) {
      return true;
    }
  }
  return false;
}

static void wsResetClientState(int idx) {
  if (idx < 0 || idx >= MAX_WS) return;
  wsHandshakeDone[idx] = false;
  hsLen[idx] = 0;
  hsBuf[idx][0] = '\0';
  wsHasPending[idx] = false;
  wsPendingLen[idx] = 0;
  wsPendingOff[idx] = 0;
  wsPendingSince[idx] = 0;
  wsLastProgressMs[idx] = 0;
  wsClientConnectedAt[idx] = 0;
}

void wsDropClient(int idx, const char *reason) {
  if (idx < 0 || idx >= MAX_WS) return;
  bool hadConnection = wsClients[idx] && wsClients[idx].connected();
  if (wsClients[idx]) {
    wsClients[idx].stop();
  }
  wsClients[idx] = WiFiClient();
  wsResetClientState(idx);
  if (reason && hadConnection) {
    SLog("Dropped client %d: %s\n", idx, reason);
  }
}

void wsDropAllClients() {
  for (int i = 0; i < MAX_WS; ++i) {
    bool hadConnection = wsClients[i] && wsClients[i].connected();
    wsDropClient(i, hadConnection ? "forced reset" : NULL);
  }
}

// broadcast counting for backoff
int broadcastCountWindow = 0;
unsigned long broadcastWindowStart = 0;
bool inBackoff = false;
unsigned long backoffStart = 0;

// ---------- helpers ----------
static inline void base64_encode_static(const uint8_t *data, size_t len, char *out, size_t outlen) {
  static const char b64[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
  size_t pos = 0;
  for (size_t j = 0; j < len && pos + 4 < outlen; j += 3) {
    uint8_t in0 = data[j];
    uint8_t in1 = (j + 1 < len) ? data[j + 1] : 0;
    uint8_t in2 = (j + 2 < len) ? data[j + 2] : 0;
    uint32_t triple = (in0 << 16) | (in1 << 8) | in2;
    out[pos++] = b64[(triple >> 18) & 0x3F];
    out[pos++] = b64[(triple >> 12) & 0x3F];
    out[pos++] = (j + 1 < len) ? b64[(triple >> 6) & 0x3F] : '=';
    out[pos++] = (j + 2 < len) ? b64[triple & 0x3F] : '=';
  }
  out[pos] = '\0';
}

static inline void computeWebSocketAccept_static(const char *key, char *out, size_t outlen) {
  const char *GUID = "258EAFA5-E914-47DA-95CA-C5AB0DC85B11";
  char tmp[128];
  snprintf(tmp, sizeof(tmp), "%s%s", key, GUID);
  unsigned char sha1sum[20];
  mbedtls_sha1((const unsigned char*)tmp, strlen(tmp), sha1sum);
  base64_encode_static(sha1sum, 20, out, outlen);
}

// find header value in buf (case-sensitive headerName like "Sec-WebSocket-Key")
static inline bool findHeaderValueInBuf(const char *buf, const char *headerName, char *out, size_t outlen) {
  const char *p = strstr(buf, headerName);
  if (!p) return false;
  p = strchr(p, ':');
  if (!p) return false;
  p++; // skip ':'
  while (*p == ' ' || *p == '\t') p++;
  const char *end = strstr(p, "\r\n");
  if (!end) return false;
  size_t len = end - p;
  if (len >= outlen) return false;
  memcpy(out, p, len);
  out[len] = '\0';
  return true;
}

// --- FIX --- safer ws text send: check availableForWrite for header+payload
static inline void wsSendTextBufFast(WiFiClient &c, const char *msg, size_t len) {
  if (!c || !c.connected()) return;
  int headerLen = 0;
  uint8_t header[4];
  if (len <= 125) {
    header[0] = 0x81; header[1] = (uint8_t)len;
    headerLen = 2;
  } else if (len <= 65535) {
    header[0] = 0x81; header[1] = 126;
    header[2] = (len >> 8) & 0xFF; header[3] = len & 0xFF;
    headerLen = 4;
  } else return;

  #if defined(WIFI_CLIENT_AVAILABLE_FOR_WRITE)
    int need = headerLen + (int)len;
    if (c.availableForWrite() < need) return;
  #endif

  c.write(header, headerLen);
  c.write((const uint8_t*)msg, len);
}

// ---------- pending buffer helpers ----------
static inline void trySendPending(int i) {
  if (!wsHasPending[i]) return;
  WiFiClient &c = wsClients[i];
  if (!c || !c.connected()) {
    wsDropClient(i, "pending flush lost socket");
    return;
  }

  int canWrite = 1024;
  #if defined(WIFI_CLIENT_AVAILABLE_FOR_WRITE)
    canWrite = c.availableForWrite();
  #endif
  if (canWrite <= 0) return;

  size_t rem = wsPendingLen[i] - wsPendingOff[i];
  size_t toWrite = rem;
  if ((size_t)canWrite < toWrite) toWrite = canWrite;

  int written = c.write(wsPendingBuf[i] + wsPendingOff[i], toWrite);
  if (written > 0) {
    wsPendingOff[i] += (size_t)written;
    wsMarkActivity();
    wsLastProgressMs[i] = millis();
    if (wsPendingOff[i] >= wsPendingLen[i]) {
      wsHasPending[i] = false;
      wsPendingOff[i] = wsPendingLen[i] = 0;
      wsPendingSince[i] = 0;
      wsLastProgressMs[i] = 0;
    }
  }
}

static inline void enqueuePayloadForClient(int i, const char *payload, size_t plen) {
  if (!(wsClients[i] && wsClients[i].connected() && wsHandshakeDone[i])) return;
  uint8_t header[4];
  int headerLen = 0;
  if (plen <= 125) {
    header[0] = 0x81;
    header[1] = (uint8_t)plen;
    headerLen = 2;
  } else if (plen <= 65535) {
    header[0] = 0x81;
    header[1] = 126;
    header[2] = (plen >> 8) & 0xFF;
    header[3] = plen & 0xFF;
    headerLen = 4;
  } else {
    size_t maxPayload = sizeof(wsPendingBuf[i]) - 4;
    if (maxPayload <= 0) return;
    plen = maxPayload;
    header[0] = 0x81;
    header[1] = 126;
    header[2] = (plen >> 8) & 0x3F;
    header[3] = plen & 0xFF;
    headerLen = 4;
  }

  size_t total = headerLen + plen;
  if (total > sizeof(wsPendingBuf[i])) {
    plen = sizeof(wsPendingBuf[i]) - headerLen;
    total = headerLen + plen;
  }

  memcpy(wsPendingBuf[i], header, headerLen);
  memcpy(wsPendingBuf[i] + headerLen, payload, plen);
  wsPendingLen[i] = total;
  wsPendingOff[i] = 0;
  if (!wsHasPending[i]) {
    unsigned long now = millis();
    wsPendingSince[i] = now;
    wsLastProgressMs[i] = now;
  }
  wsHasPending[i] = true;

  // try immediate partial send
  trySendPending(i);
}

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

  String storedSsid = prefs.getString("ssid", "");
  String storedPass = prefs.getString("pass", "");
  WiFi.mode(WIFI_STA);
  if (storedSsid.length() > 0) {
    SLog("Found saved creds - attempting connect to '%s'\n", storedSsid.c_str());
    WiFi.begin(storedSsid.c_str(), storedPass.c_str());
    wifiRegisterStaAttempt(millis());
  } else {
    SLog("No saved creds - default auto-connect disabled (no WIFI_SSID). Waiting for user action or captive AP trigger.\n");
    // Do not call WiFi.begin() with a built-in default. Connection will only be attempted
    // when the user presses the D19 button (which starts captive AP and then Save -> connect)
    // or when credentials are stored in Preferences.
  }

  showPot = false;
  connectedAt = 0;

  wsServer.begin();
  SLog("WS (81) started\n");
  for (int i = 0; i < MAX_WS; ++i) {
    wsResetClientState(i);
  }

  WiFi.setSleep(false);

  // button init (D19) - INPUT_PULLUP, button closes to GND when pressed
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  lastButtonState = digitalRead(BUTTON_PIN);
  lastButtonChangeMs = millis();
  buttonHandled = false;

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

  // --- button handling (D19) ---
  {
    int btn = digitalRead(BUTTON_PIN); // HIGH = released, LOW = pressed (pullup)
    if (btn != lastButtonState) {
      lastButtonChangeMs = now;
      lastButtonState = btn;
    } else {
      // stable state for debounce interval
      if (btn == LOW && !buttonHandled && (now - lastButtonChangeMs) >= BUTTON_DEBOUNCE_MS) {
        // Button pressed: forget wifi and start captive AP
        buttonHandled = true;
        SLog("Button D19 pressed -> forgetting WiFi and starting captive AP\n");

        // remove saved creds
        prefs.remove("ssid");
        prefs.remove("pass");

        // try disconnect and clear any ongoing STA connection
        WiFi.disconnect(true);
        wifiClearPendingConnect();

        // ensure captive AP is started (explicit user action -> immediate AP)
        startCaptiveAP();

        // lower CPU for idle captive mode
        requestCpuIdle();

        // show captive/pot UI as appropriate
        showPot = false;

        // reset some runtime state to be safe
        connectedAt = 0;

        // reset spontaneous disconnect timer because user forced AP
        wifiResetStaGraceTimer();
      }

      // reset handled flag when released (so next press triggers again)
      if (btn == HIGH && buttonHandled) {
        buttonHandled = false;
        SLog("Button D19 released -> ready for next press\n");
      }
    }
  }
  // --- end button handling ---

  wifiManageState(now);

  // accept new TCP clients (non-blocking) - DO NOT boost CPU here (avoid probes)
  WiFiClient newClient = wsServer.available();
  if (newClient) {
    int slot = -1;
    for (int i = 0; i < MAX_WS; ++i) if (!wsClients[i] || !wsClients[i].connected()) { slot = i; break; }
    if (slot >= 0) {
      wsClients[slot] = newClient;
      wsClients[slot].setNoDelay(true);
      wsResetClientState(slot);
      wsClientConnectedAt[slot] = now;
      wsMarkActivity();
      SLog("New TCP client -> slot %d\n", slot);
      // do not boost CPU here; only after meaningful activity (sampling etc)
    } else newClient.stop();
  }

  // process handshake non-blocking: read available bytes into hsBuf
  for (int i = 0; i < MAX_WS; ++i) {
    WiFiClient &c = wsClients[i];
    if (c && c.connected() && !wsHandshakeDone[i]) {
      if (wsClientConnectedAt[i] != 0 && (now - wsClientConnectedAt[i]) >= WS_HANDSHAKE_TIMEOUT_MS) {
        wsDropClient(i, "handshake timeout");
        continue;
      }
      int avail = c.available();
      if (avail > 0) {
        int toRead = avail;
        if (toRead > MAX_HS_READ_PER_ITER) toRead = MAX_HS_READ_PER_ITER; // --- FIX ---
        while (toRead--) {
          if (hsLen[i] + 1 < HS_BUF_SZ) {
            int ch = c.read();
            if (ch < 0) break;
            hsBuf[i][hsLen[i]++] = (char)ch;
            hsBuf[i][hsLen[i]] = '\0';
          } else {
            wsDropClient(i, "handshake buffer overflow");
            break;
          }
        }
      }
      if (hsLen[i] > 4 && strstr(hsBuf[i], "\r\n\r\n")) {
        char key[128];
        if (findHeaderValueInBuf(hsBuf[i], "Sec-WebSocket-Key", key, sizeof(key))) {
          char accept[128]; computeWebSocketAccept_static(key, accept, sizeof(accept));
          char resp[256];
          int n = snprintf(resp, sizeof(resp),
            "HTTP/1.1 101 Switching Protocols\r\n"
            "Upgrade: websocket\r\n"
            "Connection: Upgrade\r\n"
            "Sec-WebSocket-Accept: %s\r\n\r\n",
            accept);

          // --- FIX --- check space before handshake write
          bool writeOk = true;
          #if defined(WIFI_CLIENT_AVAILABLE_FOR_WRITE)
            if (c.availableForWrite() < n) writeOk = false;
          #endif
          if (!writeOk) {
            SLog("Handshake: not enough tx buffer, closing client %d\n", i);
            wsDropClient(i, "handshake tx backpressure");
          } else {
            c.write((const uint8_t*)resp, n);
            wsHandshakeDone[i] = true;
            wsMarkActivity();
            SLog("Handshake done for client %d\n", i);
            // DO NOT boost CPU here (avoid freq toggles on handshake) -- boost comes from sampling/activity
            lastCpuBoostTime = millis();
          }
        } else { wsDropClient(i, "handshake missing key"); }
      }
    } else if (c && !c.connected()) {
      wsDropClient(i, "socket closed");
    }
  }

  // Fetch latest sample (peek single-slot queue)
  Sample_t latest;
  bool haveSample = false;
  if (sampleQueue && xQueuePeek(sampleQueue, &latest, 0) == pdTRUE) haveSample = true;

  // update interacting state based on activity + lastInteractionMillis
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

  // Broadcast logic (use latest sample)
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

        // iterate clients and enqueue payload
        for (int i = 0; i < MAX_WS; ++i) {
          WiFiClient &c = wsClients[i];
          if (c && c.connected() && wsHandshakeDone[i]) {
            enqueuePayloadForClient(i, payload, (size_t)plen);
          } else if (c && !c.connected()) {
            wsDropClient(i, "socket closed");
          }
        }
      }
    }
  }

  // try to flush any pending partially-sent frames (non-blocking)
  for (int i = 0; i < MAX_WS; ++i) if (wsHasPending[i]) trySendPending(i);

  // drop clients whose TX buffers have been stuck for too long (likely Wi-Fi loss)
  for (int i = 0; i < MAX_WS; ++i) {
    if (!wsHasPending[i]) continue;
    unsigned long start = wsLastProgressMs[i] ? wsLastProgressMs[i] : wsPendingSince[i];
    if (start != 0 && (now - start) >= WS_PENDING_STALL_MS) {
      wsDropClient(i, "tx stalled");
    }
  }

  // automatic CPU downscale if we were boosted but nothing relevant happened for a while
  if (currentCpuMhz > MIN_IDLE_CPU_MHZ) {
    bool safeToLower = !interacting && !wifiIsCaptiveActive() && !wifiHasPendingConnect();
    if (safeToLower && (now - lastCpuBoostTime >= IDLE_CPU_TIMEOUT_MS)) {
      requestCpuIdle();
    }
  }

  // minimal serial status (infrequent)
  static unsigned long lastSerial = 0;
  if (now - lastSerial >= 800) {
    lastSerial = now;
    if (haveSample) {
      SLog("mode=%s act=%.4f bcast=%lums bwcnt=%d backoff=%d raw=%d v=%.3f pct=%.1f showPot=%d apActive=%d cpu=%d\n",
           interacting ? "INTERACT" : "IDLE",
           latest.activity, broadcastInterval, broadcastCountWindow, inBackoff?1:0,
           latest.raw, latest.voltage, latest.percent, showPot?1:0, wifiIsCaptiveActive()?1:0, currentCpuMhz);
    } else {
      SLog("no sample yet apActive=%d cpu=%d\n", wifiIsCaptiveActive()?1:0, currentCpuMhz);
    }
  }

  vTaskDelay(pdMS_TO_TICKS(2));
}
