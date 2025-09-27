#pragma once

#include <Arduino.h>

void startCaptiveAP();
void stopCaptiveAP();
void wifiManageState(unsigned long now);
void wifiClearPendingConnect();
void wifiResetStaGraceTimer();
bool wifiIsCaptiveActive();
bool wifiHasPendingConnect();
