#include "wifi_captive.h"

#include <WiFi.h>
#include <WebServer.h>
#include <DNSServer.h>
#include <Preferences.h>

#include "captive_pages.h"

void SLog(const char *fmt, ...);
void requestCpuNormal();
void requestCpuIdle();
bool wsHasActiveClients();

extern volatile bool showPot;
extern volatile unsigned long connectedAt;
extern volatile unsigned long lastCpuBoostTime;
extern Preferences prefs;

namespace {
const char *const CAPTIVE_AP_SSID = "Air Volume";
const byte DNS_PORT = 53;
const unsigned long CONNECT_TRY_TIMEOUT_MS = 10000;
const unsigned long STA_TO_CAPTIVE_DELAY_MS = CONNECT_TRY_TIMEOUT_MS;
const unsigned long GREEN_SHOW_MS = 500;

WebServer captiveServer(80);
DNSServer dnsServer;

unsigned long staDisconnectAt = 0;
bool apActive = false;
bool pendingConnect = false;
char pendingSSID[64] = "";
char pendingPASS[64] = "";
unsigned long connectAttemptStart = 0;
bool delayingCaptiveForClients = false;

void resetPendingBuffers() {
  pendingSSID[0] = '\0';
  pendingPASS[0] = '\0';
}
}  // namespace

void startCaptiveAP() {
  if (apActive) return;
  SLog("Starting captive AP '%s'\n", CAPTIVE_AP_SSID);
  WiFi.mode(WIFI_AP_STA);
  WiFi.softAP(CAPTIVE_AP_SSID);
  delay(50);
  IPAddress apIP = WiFi.softAPIP();
  dnsServer.start(DNS_PORT, "*", apIP);
  captiveServer.on("/", []() {
    captiveServer.send(200, "text/html", CAPTIVE_PAGE_FORM);
  });
  captiveServer.on("/save", []() {
    String ssid = captiveServer.arg("ssid");
    String pass = captiveServer.arg("pass");
    if (ssid.length() < 1) {
      captiveServer.send(400, "text/plain", "Missing SSID");
      return;
    }
    prefs.putString("ssid", ssid);
    prefs.putString("pass", pass);
    SLog("Saved credentials to Preferences (ssid='%s')\n", ssid.c_str());
    ssid.toCharArray(pendingSSID, sizeof(pendingSSID));
    pass.toCharArray(pendingPASS, sizeof(pendingPASS));
    pendingConnect = true;
    captiveServer.send(200, "text/html", CAPTIVE_PAGE_CONNECTING);
  });
  captiveServer.on("/forget", []() {
    prefs.remove("ssid");
    prefs.remove("pass");
    SLog("Cleared saved credentials from Preferences\n");
    captiveServer.send(200, "text/html", CAPTIVE_PAGE_FORGOTTEN);
  });
  captiveServer.onNotFound([]() {
    captiveServer.sendHeader("Location", String("http://") + WiFi.softAPIP().toString() + "/", true);
    captiveServer.send(302, "text/plain", "");
  });
  captiveServer.begin();
  apActive = true;
}

void stopCaptiveAP() {
  if (!apActive) return;
  SLog("Stopping captive AP\n");
  dnsServer.stop();
  captiveServer.stop();
  WiFi.softAPdisconnect(true);
  delay(50);
  apActive = false;
}

void wifiManageState(unsigned long now) {
  wl_status_t wifiSt = WiFi.status();

  if (wifiSt == WL_CONNECTED) {
    staDisconnectAt = 0;
    delayingCaptiveForClients = false;
    if (apActive) stopCaptiveAP();
    if (!showPot && (now - connectedAt >= GREEN_SHOW_MS)) {
      showPot = true;
      SLog("Switching to POT display mode\n");
    }
    connectedAt = (connectedAt == 0) ? now : connectedAt;
  } else {
    if (connectAttemptStart != 0) {
      staDisconnectAt = 0;
      delayingCaptiveForClients = false;
    } else {
      if (!apActive && !pendingConnect) {
        if (wsHasActiveClients()) {
          if (!delayingCaptiveForClients) {
            SLog("STA lost but WS clients active -> delaying captive AP\n");
            delayingCaptiveForClients = true;
          }
          staDisconnectAt = 0;
        } else {
          if (delayingCaptiveForClients) {
            delayingCaptiveForClients = false;
          }
          if (staDisconnectAt == 0) {
            staDisconnectAt = now;
            SLog("STA lost: will wait %lums before enabling captive AP\n", STA_TO_CAPTIVE_DELAY_MS);
          } else if (now - staDisconnectAt >= STA_TO_CAPTIVE_DELAY_MS) {
            SLog("STA loss grace expired -> starting captive AP\n");
            startCaptiveAP();
            requestCpuIdle();
            staDisconnectAt = 0;
          }
        }
      } else {
        staDisconnectAt = 0;
        delayingCaptiveForClients = false;
      }
    }
  }

  if (apActive) {
    dnsServer.processNextRequest();
    captiveServer.handleClient();
  }

  wifiSt = WiFi.status();
  if (wifiSt == WL_CONNECTED) {
    delayingCaptiveForClients = false;
    if (!showPot && (now - connectedAt >= GREEN_SHOW_MS)) {
      showPot = true;
      SLog("Switching to POT display mode\n");
    }
    connectedAt = (connectedAt == 0) ? now : connectedAt;
  }

  if (pendingConnect) {
    if (apActive) {
      stopCaptiveAP();
      delay(50);
    }
    SLog("Attempting STA connect to SSID='%s'\n", pendingSSID);
    requestCpuNormal();
    lastCpuBoostTime = now;
    WiFi.mode(WIFI_STA);
    WiFi.begin(pendingSSID, pendingPASS);
    connectAttemptStart = now;
    pendingConnect = false;
    resetPendingBuffers();
    staDisconnectAt = 0;
    delayingCaptiveForClients = false;
  }

  if (connectAttemptStart != 0) {
    wl_status_t st = WiFi.status();
    if (st == WL_CONNECTED) {
      SLog("STA connect success\n");
      connectAttemptStart = 0;
      apActive = false;
      delayingCaptiveForClients = false;
    } else if (now - connectAttemptStart >= CONNECT_TRY_TIMEOUT_MS) {
      SLog("STA connect timeout, re-enabling captive AP\n");
      connectAttemptStart = 0;
      startCaptiveAP();
      requestCpuIdle();
      delayingCaptiveForClients = false;
    }
  }
}

void wifiClearPendingConnect() {
  pendingConnect = false;
}

void wifiResetStaGraceTimer() {
  staDisconnectAt = 0;
}

bool wifiIsCaptiveActive() {
  return apActive;
}

bool wifiHasPendingConnect() {
  return pendingConnect;
}
