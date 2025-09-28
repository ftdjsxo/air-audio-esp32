#pragma once

#include <Arduino.h>

void powerManagerSetup(unsigned long now, unsigned long idleBroadcastMs);
void powerTurnOn(unsigned long now);
void powerTurnOff(unsigned long now);
void powerHandleLongPress(unsigned long now);
bool powerIsOn();
