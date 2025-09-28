#include "ws_manager.h"

#include <WiFi.h>
#include <WiFiClient.h>
#include <mbedtls/sha1.h>

#include <string.h>
#include <stdio.h>

void SLog(const char *fmt, ...);
extern volatile unsigned long lastCpuBoostTime;

namespace {
constexpr int WS_MAX_CLIENTS = 4;
constexpr size_t HS_BUF_SZ = 600;
constexpr unsigned long WS_PENDING_STALL_MS = 6000UL;
constexpr unsigned long WS_HANDSHAKE_TIMEOUT_MS = 4000UL;

WiFiServer wsServer(WS_SERVER_PORT);
WiFiClient wsClients[WS_MAX_CLIENTS];
bool wsHandshakeDone[WS_MAX_CLIENTS];
char hsBuf[WS_MAX_CLIENTS][HS_BUF_SZ];
size_t hsLen[WS_MAX_CLIENTS];
uint8_t wsPendingBuf[WS_MAX_CLIENTS][256];
size_t wsPendingLen[WS_MAX_CLIENTS];
size_t wsPendingOff[WS_MAX_CLIENTS];
bool wsHasPending[WS_MAX_CLIENTS];
unsigned long wsPendingSince[WS_MAX_CLIENTS];
unsigned long wsLastProgressMs[WS_MAX_CLIENTS];
unsigned long wsClientConnectedAt[WS_MAX_CLIENTS];
unsigned long wsLastClientActivity = 0;

inline void wsMarkActivity(unsigned long now) {
  wsLastClientActivity = now;
}

void wsResetClientState(int idx) {
  if (idx < 0 || idx >= WS_MAX_CLIENTS) return;
  wsHandshakeDone[idx] = false;
  hsLen[idx] = 0;
  hsBuf[idx][0] = '\0';
  wsHasPending[idx] = false;
  wsPendingLen[idx] = wsPendingOff[idx] = 0;
  wsPendingSince[idx] = 0;
  wsLastProgressMs[idx] = 0;
  wsClientConnectedAt[idx] = 0;
}

bool findHeaderValueInBuf(const char *buf, const char *headerName, char *out, size_t outlen) {
  const char *p = strstr(buf, headerName);
  if (!p) return false;
  p = strchr(p, ':');
  if (!p) return false;
  p++;
  while (*p == ' ' || *p == '	') p++;
  const char *end = strstr(p, "\r\n");
  if (!end) return false;
  size_t len = static_cast<size_t>(end - p);
  if (len >= outlen) return false;
  memcpy(out, p, len);
  out[len] = '\0';
  return true;
}

void base64_encode_static(const uint8_t *data, size_t len, char *out, size_t outlen) {
  static const char b64[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
  size_t pos = 0;
  for (size_t j = 0; j < len && pos + 4 < outlen; j += 3) {
    uint8_t in0 = data[j];
    uint8_t in1 = (j + 1 < len) ? data[j + 1] : 0;
    uint8_t in2 = (j + 2 < len) ? data[j + 2] : 0;
    uint32_t triple = (static_cast<uint32_t>(in0) << 16) | (static_cast<uint32_t>(in1) << 8) | in2;
    out[pos++] = b64[(triple >> 18) & 0x3F];
    out[pos++] = b64[(triple >> 12) & 0x3F];
    out[pos++] = (j + 1 < len) ? b64[(triple >> 6) & 0x3F] : '=';
    out[pos++] = (j + 2 < len) ? b64[triple & 0x3F] : '=';
  }
  out[pos] = '\0';
}

void computeWebSocketAccept_static(const char *key, char *out, size_t outlen) {
  const char *GUID = "258EAFA5-E914-47DA-95CA-C5AB0DC85B11";
  char tmp[128];
  snprintf(tmp, sizeof(tmp), "%s%s", key, GUID);
  unsigned char sha1sum[20];
  mbedtls_sha1(reinterpret_cast<const unsigned char *>(tmp), strlen(tmp), sha1sum);
  base64_encode_static(sha1sum, 20, out, outlen);
}

void wsDropClientInternal(int idx, const char *reason, unsigned long now) {
  if (idx < 0 || idx >= WS_MAX_CLIENTS) return;
  bool hadConnection = wsClients[idx] && wsClients[idx].connected();
  if (wsClients[idx]) {
    wsClients[idx].stop();
  }
  wsClients[idx] = WiFiClient();
  wsResetClientState(idx);
  if (hadConnection) {
    wsMarkActivity(now);
  }
  if (reason && hadConnection) {
    SLog("Dropped client %d: %s\n", idx, reason);
  }
}

void trySendPending(int idx, unsigned long now) {
  if (idx < 0 || idx >= WS_MAX_CLIENTS) return;
  if (!wsHasPending[idx]) return;
  WiFiClient &c = wsClients[idx];
  if (!c || !c.connected()) {
    wsDropClientInternal(idx, "pending flush lost socket", now);
    return;
  }

  int canWrite = 1024;
  #if defined(WIFI_CLIENT_AVAILABLE_FOR_WRITE)
    canWrite = c.availableForWrite();
  #endif
  if (canWrite <= 0) return;

  size_t remaining = wsPendingLen[idx] - wsPendingOff[idx];
  size_t toWrite = remaining;
  if (static_cast<size_t>(canWrite) < toWrite) toWrite = static_cast<size_t>(canWrite);

  int written = c.write(wsPendingBuf[idx] + wsPendingOff[idx], toWrite);
  if (written > 0) {
    wsPendingOff[idx] += static_cast<size_t>(written);
    wsLastProgressMs[idx] = now;
    wsMarkActivity(now);
    if (wsPendingOff[idx] >= wsPendingLen[idx]) {
      wsHasPending[idx] = false;
      wsPendingOff[idx] = wsPendingLen[idx] = 0;
      wsPendingSince[idx] = 0;
      wsLastProgressMs[idx] = 0;
    }
  }
}

void enqueuePayloadForClient(int idx, const char *payload, size_t plen, unsigned long now) {
  WiFiClient &c = wsClients[idx];
  if (!(c && c.connected() && wsHandshakeDone[idx])) return;
  uint8_t header[4];
  int headerLen = 0;

  if (plen <= 125) {
    header[0] = 0x81;
    header[1] = static_cast<uint8_t>(plen);
    headerLen = 2;
  } else if (plen <= 65535) {
    header[0] = 0x81;
    header[1] = 126;
    header[2] = static_cast<uint8_t>((plen >> 8) & 0xFF);
    header[3] = static_cast<uint8_t>(plen & 0xFF);
    headerLen = 4;
  } else {
    size_t maxPayload = sizeof(wsPendingBuf[idx]) - 4;
    if (maxPayload == 0) return;
    plen = maxPayload;
    header[0] = 0x81;
    header[1] = 126;
    header[2] = static_cast<uint8_t>((plen >> 8) & 0x3F);
    header[3] = static_cast<uint8_t>(plen & 0xFF);
    headerLen = 4;
  }

  size_t total = headerLen + plen;
  if (total > sizeof(wsPendingBuf[idx])) {
    plen = sizeof(wsPendingBuf[idx]) - headerLen;
    total = headerLen + plen;
  }

  memcpy(wsPendingBuf[idx], header, headerLen);
  memcpy(wsPendingBuf[idx] + headerLen, payload, plen);
  wsPendingLen[idx] = total;
  wsPendingOff[idx] = 0;
  if (!wsHasPending[idx]) {
    wsPendingSince[idx] = now;
    wsLastProgressMs[idx] = now;
  }
  wsHasPending[idx] = true;
  trySendPending(idx, now);
}

} // namespace

void wsInit() {
  wsServer.begin();
  unsigned long now = millis();
  for (int i = 0; i < WS_MAX_CLIENTS; ++i) {
    wsResetClientState(i);
  }
  wsMarkActivity(now);
  SLog("WS (81) started\n");
}

void wsTick(unsigned long now) {
  WiFiClient newClient = wsServer.available();
  if (newClient) {
    int slot = -1;
    for (int i = 0; i < WS_MAX_CLIENTS; ++i) {
      if (!wsClients[i] || !wsClients[i].connected()) {
        slot = i;
        break;
      }
    }
    if (slot >= 0) {
      wsClients[slot] = newClient;
      wsClients[slot].setNoDelay(true);
      wsResetClientState(slot);
      wsClientConnectedAt[slot] = now;
      wsMarkActivity(now);
      SLog("New TCP client -> slot %d\n", slot);
    } else {
      newClient.stop();
    }
  }

  for (int i = 0; i < WS_MAX_CLIENTS; ++i) {
    WiFiClient &c = wsClients[i];
    if (c && c.connected() && !wsHandshakeDone[i]) {
      if (wsClientConnectedAt[i] != 0 && (now - wsClientConnectedAt[i]) >= WS_HANDSHAKE_TIMEOUT_MS) {
        wsDropClientInternal(i, "handshake timeout", now);
        continue;
      }
      int avail = c.available();
      if (avail > 0) {
        int toRead = avail;
        const int MAX_HS_READ_PER_ITER = 128;
        if (toRead > MAX_HS_READ_PER_ITER) toRead = MAX_HS_READ_PER_ITER;
        while (toRead--) {
          if (hsLen[i] + 1 < HS_BUF_SZ) {
            int ch = c.read();
            if (ch < 0) break;
            hsBuf[i][hsLen[i]++] = static_cast<char>(ch);
            hsBuf[i][hsLen[i]] = '\0';
          } else {
            wsDropClientInternal(i, "handshake buffer overflow", now);
            break;
          }
        }
      }
      if (hsLen[i] > 4 && strstr(hsBuf[i], "\r\n\r\n")) {
        char key[128];
        if (findHeaderValueInBuf(hsBuf[i], "Sec-WebSocket-Key", key, sizeof(key))) {
          char accept[128];
          computeWebSocketAccept_static(key, accept, sizeof(accept));
          char resp[256];
          int n = snprintf(resp, sizeof(resp),
            "HTTP/1.1 101 Switching Protocols\r\n"
            "Upgrade: websocket\r\n"
            "Connection: Upgrade\r\n"
            "Sec-WebSocket-Accept: %s\r\n\r\n",
            accept);
          bool writeOk = true;
          #if defined(WIFI_CLIENT_AVAILABLE_FOR_WRITE)
            if (c.availableForWrite() < n) writeOk = false;
          #endif
          if (!writeOk) {
            wsDropClientInternal(i, "handshake tx backpressure", now);
          } else {
            c.write(reinterpret_cast<const uint8_t*>(resp), n);
            wsHandshakeDone[i] = true;
            wsMarkActivity(now);
            SLog("Handshake done for client %d\n", i);
            lastCpuBoostTime = now;
          }
        } else {
          wsDropClientInternal(i, "handshake missing key", now);
        }
      }
    } else if (c && !c.connected()) {
      wsDropClientInternal(i, "socket closed", now);
    }
  }

  for (int i = 0; i < WS_MAX_CLIENTS; ++i) {
    if (wsHasPending[i]) trySendPending(i, now);
  }

  for (int i = 0; i < WS_MAX_CLIENTS; ++i) {
    if (!wsHasPending[i]) continue;
    unsigned long start = wsLastProgressMs[i] ? wsLastProgressMs[i] : wsPendingSince[i];
    if (start != 0 && (now - start) >= WS_PENDING_STALL_MS) {
      wsDropClientInternal(i, "tx stalled", now);
    }
  }
}

void wsBroadcastPayload(unsigned long now, const char *payload, size_t len) {
  if (!payload || len == 0) return;
  for (int i = 0; i < WS_MAX_CLIENTS; ++i) {
    WiFiClient &c = wsClients[i];
    if (c && c.connected() && wsHandshakeDone[i]) {
      enqueuePayloadForClient(i, payload, len, now);
    } else if (c && !c.connected()) {
      wsDropClientInternal(i, "socket closed", now);
    }
  }
}

bool wsHasActiveClients() {
  for (int i = 0; i < WS_MAX_CLIENTS; ++i) {
    if (wsClients[i] && wsClients[i].connected()) return true;
  }
  return false;
}

void wsDropAllClients() {
  unsigned long now = millis();
  for (int i = 0; i < WS_MAX_CLIENTS; ++i) {
    wsDropClientInternal(i, "forced reset", now);
  }
}

unsigned long wsGetLastClientActivity() {
  return wsLastClientActivity;
}
