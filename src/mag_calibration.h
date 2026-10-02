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
// sensor was level with Z up. Only the first levelCount samples (taken level) are used for that
// fit, since tilted samples shift the X/Y circle. Pure math, no hardware access.
MagCalibration fitMagCalibration(const int16_t (*samples)[3], int count, int levelCount, float inclinationDeg);

// Magnetic inclination in degrees (positive = field points down) from a calibrated field
// vector and the accelerometer reading (+1 g upwards at rest), in the same axes
float fieldInclination(const float mag[3], const float acc[3]);
