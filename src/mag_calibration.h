#pragma once
#include <stdint.h>

struct MagCalibration {
  bool ok;
  bool horizontalOnly; // only X/Y could be fitted; Z offset derived from the inclination
  float offset[3];     // hard-iron offsets in raw sensor counts
  float scale[3];      // per-axis gain correction
};

// Fits hard-iron offsets and per-axis gains (axis-aligned ellipsoid) to magnetometer samples
// collected while the sensor is turned. When the Z axis barely changes, i.e. the sensor was
// only turned around the vertical axis (antenna boom), only X/Y are fitted: the Z gain is
// taken from X/Y and the Z offset follows from the known magnetic inclination, assuming the
// sensor was level with Z up. Pure math, no hardware access.
MagCalibration fitMagCalibration(const int16_t (*samples)[3], int count, float inclinationDeg);
