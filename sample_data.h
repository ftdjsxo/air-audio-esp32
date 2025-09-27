#pragma once

#include <Arduino.h>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

typedef struct {
  int raw;
  float voltage;
  float percent;
  float activity;
  unsigned long ts;
} Sample_t;

extern QueueHandle_t sampleQueue;
