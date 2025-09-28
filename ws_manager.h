#pragma once

#include <Arduino.h>

constexpr uint16_t WS_SERVER_PORT = 81;

void wsInit();
void wsTick(unsigned long now);
void wsBroadcastPayload(unsigned long now, const char *payload, size_t len);
bool wsHasActiveClients();
void wsDropAllClients();
unsigned long wsGetLastClientActivity();
