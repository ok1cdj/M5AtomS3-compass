#pragma once
#include <Arduino.h>

// Supported sensor sets, detected at startup: GY-511 (LSM303D or LSM303DLHC) on the Grove
// port (G1/G2, either wire order), QMC5883L on the internal bus header pins (SDA 38 / SCL 39)
enum class SensorSet {
  None,
  QMC5883L_InternalImu, // external QMC5883L magnetometer + AtomS3 internal accelerometer
  LSM303DLHC,           // external GY-511: accelerometer and magnetometer in one chip
  LSM303D               // external GY-511 variant: newer chip, one I2C address for both
};

bool sensorsBegin();
SensorSet sensorSet();
const char* sensorName();
// Short prefix for NVS keys, so every sensor set keeps its own calibration
const char* sensorKeyPrefix();

// Magnetometer in device axes (arrow = +Y, display up = +Z), raw sensor counts.
// Returns false when no new sample is available or the read failed.
bool readMagRaw(int &x, int &y, int &z);
// Blocking variant for calibration: waits up to timeoutMs for a new sample
bool readMagRawWait(int &x, int &y, int &z, uint32_t timeoutMs = 100);

// Accelerometer in device axes in g, reads +1 g upwards at rest
bool readAccRaw(float &x, float &y, float &z);
