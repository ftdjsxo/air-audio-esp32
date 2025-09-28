#include "sampling_task.h"

#include <Arduino.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "sample_data.h"

extern const int potPin;
extern QueueHandle_t sampleQueue;
extern volatile bool interacting;
extern unsigned long lastInteractionMillis;
extern volatile unsigned long lastCpuBoostTime;

void requestCpuNormal();

extern const unsigned long INTERACT_SAMPLE_MS;
extern const unsigned long IDLE_SAMPLE_MS;

namespace {
constexpr int HISTORY_SIZE = 64;
struct DeltaEntry {
  unsigned long ts;
  int delta;
};

DeltaEntry history[HISTORY_SIZE];
int historyHead = 0;
int historyCount = 0;

constexpr unsigned long HISTORY_WINDOW_MS = 500;
constexpr int IMMEDIATE_TRIGGER_RAW = 12;
constexpr int MIN_MOVEMENT_RAW = 10;

void samplingTask(void *pv) {
  (void)pv;
  int local_potRaw = analogRead(potPin);
  unsigned long local_lastPotSample = xTaskGetTickCount() * portTICK_PERIOD_MS;
  const float inv4095 = 1.0f / 4095.0f;
  int immediateConsecutive = 0;

  for (;;) {
    unsigned long now = xTaskGetTickCount() * portTICK_PERIOD_MS;
    unsigned long local_sampleInterval = interacting ? INTERACT_SAMPLE_MS : IDLE_SAMPLE_MS;

    if (now - local_lastPotSample >= local_sampleInterval) {
      local_lastPotSample = now;
      int raw = analogRead(potPin);
      int deltaRaw = abs(raw - local_potRaw);

      history[historyHead].ts = now;
      history[historyHead].delta = deltaRaw;
      historyHead = (historyHead + 1) % HISTORY_SIZE;
      if (historyCount < HISTORY_SIZE) historyCount++;

      unsigned long cutoff = (now > HISTORY_WINDOW_MS) ? (now - HISTORY_WINDOW_MS) : 0;
      long sum = 0;
      int cnt = 0;
      for (int i = 0; i < historyCount; ++i) {
        int idx = (historyHead - 1 - i + HISTORY_SIZE) % HISTORY_SIZE;
        if (history[idx].ts >= cutoff) {
          sum += history[idx].delta;
          cnt++;
        } else {
          break;
        }
      }
      float activity = 0.0f;
      if (cnt > 0) activity = (static_cast<float>(sum) / static_cast<float>(cnt)) * inv4095;

      local_potRaw = raw;
      Sample_t s;
      s.raw = raw;
      s.voltage = static_cast<float>(raw) * 3.3f * inv4095;
      s.percent = static_cast<float>(raw) * 100.0f * inv4095;
      s.activity = activity;
      s.ts = now;
      if (sampleQueue) xQueueOverwrite(sampleQueue, &s);

      if (deltaRaw >= MIN_MOVEMENT_RAW) {
        lastInteractionMillis = now;
      }

      if (deltaRaw >= IMMEDIATE_TRIGGER_RAW) {
        immediateConsecutive++;
        if (immediateConsecutive >= 2) {
          lastInteractionMillis = now;
          requestCpuNormal();
          lastCpuBoostTime = now;
          immediateConsecutive = 0;
        }
      } else {
        immediateConsecutive = 0;
      }
    }
    vTaskDelay(pdMS_TO_TICKS(4));
  }
}
}  // namespace

void startSamplingTask() {
  analogSetPinAttenuation(potPin, ADC_11db);
  analogReadResolution(12);
  delay(5);

  if (!sampleQueue) {
    sampleQueue = xQueueCreate(1, sizeof(Sample_t));
  }

  int initRaw = analogRead(potPin);
  if (sampleQueue) {
    Sample_t ini;
    ini.raw = initRaw;
    ini.voltage = static_cast<float>(initRaw) * 3.3f / 4095.0f;
    ini.percent = static_cast<float>(initRaw) * 100.0f / 4095.0f;
    ini.activity = 0.0f;
    ini.ts = millis();
    xQueueOverwrite(sampleQueue, &ini);
  }

  xTaskCreatePinnedToCore(samplingTask, "sampling", 4096, NULL, 4, NULL, 1);
}
