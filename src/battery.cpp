#include "battery.h"
#include <Arduino.h>
#include <M5Unified.h>
#include <math.h>

static const int BATTERY_PIN = 8;
static const float DIVIDER_RATIO = 2.0;
static const uint32_t SAMPLE_INTERVAL_MS = 1000;
static const int ADC_READS_PER_SAMPLE = 8;
static const float FILTER_ALPHA = 0.2;

// A floating pin is told apart from a real battery by a plausible and stable voltage
static const float MIN_PLAUSIBLE_V = 2.8;
static const float MAX_PLAUSIBLE_V = 4.4;
static const float MAX_STDDEV_V = 0.05;
static const int WINDOW_SIZE = 5;

// Charge level breakpoints of the Atomic Battery Base documentation
static const float CURVE_V[] = {3.00, 3.48, 3.62, 3.82, 4.20};
static const int CURVE_PERCENT[] = {0, 25, 50, 75, 100};
static const int CURVE_POINTS = sizeof(CURVE_V) / sizeof(CURVE_V[0]);

static uint32_t lastSample = 0;
static float window[WINDOW_SIZE];
static int windowCount = 0;
static int windowPos = 0;
static float filteredV = 0;
static bool present = false;

void batteryBegin() {
  // The default ADC attenuation (11 dB) covers the divided battery voltage
  analogReadMilliVolts(BATTERY_PIN);
}

static float readVoltage() {
  uint32_t sum = 0;
  for (int i = 0; i < ADC_READS_PER_SAMPLE; i++) {
    sum += analogReadMilliVolts(BATTERY_PIN);
  }
  return sum / (float)ADC_READS_PER_SAMPLE / 1000.0 * DIVIDER_RATIO;
}

static bool windowIsBattery() {
  if (windowCount < WINDOW_SIZE) return false;
  float mean = 0;
  for (int i = 0; i < WINDOW_SIZE; i++) {
    if (window[i] < MIN_PLAUSIBLE_V || window[i] > MAX_PLAUSIBLE_V) return false;
    mean += window[i];
  }
  mean /= WINDOW_SIZE;
  float var = 0;
  for (int i = 0; i < WINDOW_SIZE; i++) var += (window[i] - mean) * (window[i] - mean);
  return sqrt(var / WINDOW_SIZE) <= MAX_STDDEV_V;
}

void batteryUpdate() {
  if (lastSample != 0 && millis() - lastSample < SAMPLE_INTERVAL_MS) return;
  lastSample = millis();

  float v = readVoltage();
  window[windowPos] = v;
  windowPos = (windowPos + 1) % WINDOW_SIZE;
  if (windowCount < WINDOW_SIZE) windowCount++;

  filteredV = (filteredV == 0) ? v : filteredV + FILTER_ALPHA * (v - filteredV);
  bool wasPresent = present;
  present = windowIsBattery();
  if (present != wasPresent) {
    M5.Log.printf("Battery %s, %.2f V\n", present ? "detected" : "not detected", filteredV);
  }
}

bool batteryPresent() {
  return present;
}

float batteryVoltage() {
  return filteredV;
}

int batteryPercent() {
  if (filteredV <= CURVE_V[0]) return 0;
  for (int i = 1; i < CURVE_POINTS; i++) {
    if (filteredV <= CURVE_V[i]) {
      float t = (filteredV - CURVE_V[i - 1]) / (CURVE_V[i] - CURVE_V[i - 1]);
      return (int)round(CURVE_PERCENT[i - 1] + t * (CURVE_PERCENT[i] - CURVE_PERCENT[i - 1]));
    }
  }
  return 100;
}
