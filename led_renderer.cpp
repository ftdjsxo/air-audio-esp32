#include "led_renderer.h"

#include <Arduino.h>
#include <WiFi.h>
#include <math.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "sample_data.h"
#include "led_config.h"
#include "power_manager.h"

extern volatile bool showPot;

namespace {
constexpr unsigned long RED_FADE_UPDATE_MS = 40;
constexpr int RED_FADE_STEP = 4;
constexpr unsigned long BLUE_FADE_UPDATE_MS = 10;
constexpr int BLUE_FADE_STEP = 8;

int redFadeVal = 0;
int redFadeDir = 1;
int blueFadeVal = 0;
int blueFadeDir = 1;

inline int m255_float(float v, float a, float b) {
  if (b - a <= 0.00001f) return 0;
  if (v <= a) return 0;
  if (v >= b) return 255;
  float f = (v - a) / (b - a);
  int r = (int)roundf(f * 255.0f);
  if (r < 0) r = 0;
  if (r > 255) r = 255;
  return r;
}
}  // namespace

void ledTask(void *pv) {
  (void)pv;
  unsigned long lastRed = 0;
  unsigned long lastBlue = 0;
  bool ledsCleared = false;

  for (;;) {
    unsigned long now = xTaskGetTickCount() * portTICK_PERIOD_MS;
    bool powered = powerIsOn();

    if (!powered) {
      if (!ledsCleared) {
        ledc_set_duty(LEDC_MODE, GREEN_CH, 0);
        ledc_update_duty(LEDC_MODE, GREEN_CH);
        ledc_set_duty(LEDC_MODE, BLUE_CH, 0);
        ledc_update_duty(LEDC_MODE, BLUE_CH);
        ledc_set_duty(LEDC_MODE, RED_CH, 0);
        ledc_update_duty(LEDC_MODE, RED_CH);
        ledsCleared = true;
      }
      vTaskDelay(pdMS_TO_TICKS(40));
      continue;
    }

    ledsCleared = false;
    bool sp = showPot;

    if (!sp) {
      wl_status_t st = WiFi.status();
      if (st != WL_CONNECTED) {
        if (st == WL_NO_SHIELD || st == WL_DISCONNECTED) {
          if (now - lastRed >= RED_FADE_UPDATE_MS) {
            lastRed = now;
            redFadeVal += redFadeDir * RED_FADE_STEP;
            if (redFadeVal >= 255) {
              redFadeVal = 255;
              redFadeDir = -1;
            }
            if (redFadeVal <= 0) {
              redFadeVal = 0;
              redFadeDir = 1;
            }
            ledc_set_duty(LEDC_MODE, RED_CH, redFadeVal);
            ledc_update_duty(LEDC_MODE, RED_CH);
            ledc_set_duty(LEDC_MODE, GREEN_CH, 0);
            ledc_update_duty(LEDC_MODE, GREEN_CH);
            ledc_set_duty(LEDC_MODE, BLUE_CH, 0);
            ledc_update_duty(LEDC_MODE, BLUE_CH);
          }
        } else {
          if (now - lastBlue >= BLUE_FADE_UPDATE_MS) {
            lastBlue = now;
            blueFadeVal += blueFadeDir * BLUE_FADE_STEP;
            if (blueFadeVal >= 255) {
              blueFadeVal = 255;
              blueFadeDir = -1;
            }
            if (blueFadeVal <= 0) {
              blueFadeVal = 0;
              blueFadeDir = 1;
            }
            ledc_set_duty(LEDC_MODE, BLUE_CH, blueFadeVal);
            ledc_update_duty(LEDC_MODE, BLUE_CH);
            ledc_set_duty(LEDC_MODE, GREEN_CH, 0);
            ledc_update_duty(LEDC_MODE, GREEN_CH);
            ledc_set_duty(LEDC_MODE, RED_CH, 0);
            ledc_update_duty(LEDC_MODE, RED_CH);
          }
        }
      } else {
        ledc_set_duty(LEDC_MODE, GREEN_CH, 255);
        ledc_update_duty(LEDC_MODE, GREEN_CH);
        ledc_set_duty(LEDC_MODE, RED_CH, 0);
        ledc_update_duty(LEDC_MODE, RED_CH);
        ledc_set_duty(LEDC_MODE, BLUE_CH, 0);
        ledc_update_duty(LEDC_MODE, BLUE_CH);
      }
    } else {
      Sample_t latest;
      if (sampleQueue && xQueuePeek(sampleQueue, &latest, 0) == pdTRUE) {
        const float t1 = 33.3333333f;
        const float t2 = 66.6666667f;
        float pct = latest.percent;

        int greenVal = m255_float(pct, 0.0f, t1);
        if (pct > t1) greenVal = 255;

        int blueVal = 0;
        if (pct <= t1) {
          blueVal = 0;
        } else if (pct <= t2) {
          blueVal = m255_float(pct, t1, t2);
        } else {
          blueVal = 255;
        }

        int redVal = 0;
        if (pct <= t2) {
          redVal = 0;
        } else {
          redVal = m255_float(pct, t2, 100.0f);
        }

        ledc_set_duty(LEDC_MODE, GREEN_CH, greenVal);
        ledc_update_duty(LEDC_MODE, GREEN_CH);
        ledc_set_duty(LEDC_MODE, BLUE_CH, blueVal);
        ledc_update_duty(LEDC_MODE, BLUE_CH);
        ledc_set_duty(LEDC_MODE, RED_CH, redVal);
        ledc_update_duty(LEDC_MODE, RED_CH);
      }

      vTaskDelay(pdMS_TO_TICKS(16));
    }

    vTaskDelay(pdMS_TO_TICKS(8));
  }
}
