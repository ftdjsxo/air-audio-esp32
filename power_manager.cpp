#include "power_manager.h"

#include <WiFi.h>
#include <Preferences.h>

#include "wifi_captive.h"
#include "ws_manager.h"

void SLog(const char *fmt, ...);

extern Preferences prefs;
extern volatile bool showPot;
extern volatile unsigned long connectedAt;
extern volatile bool interacting;
extern volatile unsigned long lastInteractionMillis;
extern unsigned long interactingSince;
extern unsigned long broadcastInterval;
extern unsigned long lastBroadcastTime;
extern int lastBroadcastRaw;
extern unsigned long broadcastWindowStart;
extern int broadcastCountWindow;
extern bool inBackoff;
extern unsigned long backoffStart;
extern volatile unsigned long lastCpuBoostTime;

void requestCpuNormal();
void requestCpuIdle();
void wifiResetStaGraceTimer();
void wifiClearPendingConnect();
void wifiRegisterStaAttempt(unsigned long now);
void startCaptiveAP();
void stopCaptiveAP();
void wsDropAllClients();

namespace {
volatile bool gDevicePowered = true;
unsigned long gIdleBroadcastMs = 0;
constexpr unsigned long POWER_AUTO_OFF_TIMEOUT_MS = 90UL * 60UL * 1000UL;
unsigned long gLastActivityMs = 0;

void resetRuntimeState(unsigned long now) {
  showPot = false;
  connectedAt = 0;
  interacting = false;
  lastInteractionMillis = now;
  interactingSince = 0;
  if (gIdleBroadcastMs > 0) {
    broadcastInterval = gIdleBroadcastMs;
  }
  lastBroadcastTime = now;
  lastBroadcastRaw = 0;
  broadcastWindowStart = now;
  broadcastCountWindow = 0;
  inBackoff = false;
  backoffStart = 0;
  gLastActivityMs = now;
}

void connectFromStoredCredentials(unsigned long now) {
  String storedSsid = prefs.getString("ssid", "");
  String storedPass = prefs.getString("pass", "");
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  if (storedSsid.length() > 0) {
    SLog("Connecting to saved network '%s'\n", storedSsid.c_str());
    WiFi.begin(storedSsid.c_str(), storedPass.c_str());
    wifiRegisterStaAttempt(now);
  } else {
    SLog("No saved credentials - waiting for captive portal\n");
  }
}
}  // namespace

void powerManagerSetup(unsigned long now, unsigned long idleBroadcastMs) {
  gDevicePowered = true;
  gIdleBroadcastMs = idleBroadcastMs;
  resetRuntimeState(now);
  connectFromStoredCredentials(now);
}

bool powerIsOn() {
  return gDevicePowered;
}

void powerTurnOff(unsigned long now) {
  if (!gDevicePowered) return;
  SLog("Device powering OFF\n");
  gDevicePowered = false;
  resetRuntimeState(now);
  wsDropAllClients();
  wifiClearPendingConnect();
  wifiResetStaGraceTimer();
  stopCaptiveAP();
  WiFi.disconnect(true);
  WiFi.mode(WIFI_OFF);
  requestCpuIdle();
}

void powerTurnOn(unsigned long now) {
  if (gDevicePowered) return;
  SLog("Device powering ON\n");
  gDevicePowered = true;
  resetRuntimeState(now);
  wifiClearPendingConnect();
  wifiResetStaGraceTimer();
  requestCpuNormal();
  lastCpuBoostTime = now;
  connectFromStoredCredentials(now);
}

void powerHandleLongPress(unsigned long now) {
  if (!gDevicePowered) return;
  SLog("Button D19 long press -> starting captive portal\n");
  prefs.remove("ssid");
  prefs.remove("pass");
  WiFi.disconnect(true);
  wifiClearPendingConnect();
  stopCaptiveAP();
  startCaptiveAP();
  requestCpuIdle();
  resetRuntimeState(now);
  wifiResetStaGraceTimer();
}

void powerManagerTick(unsigned long now) {
  if (!gDevicePowered) return;
  if (lastInteractionMillis != 0 && lastInteractionMillis > gLastActivityMs) {
    gLastActivityMs = lastInteractionMillis;
  }
  if (gLastActivityMs != 0 && (now - gLastActivityMs) >= POWER_AUTO_OFF_TIMEOUT_MS) {
    SLog("Auto power-off after inactivity\n");
    powerTurnOff(now);
  }
}
