#include "mag_calibration.h"
#include <math.h>

static const int MIN_SAMPLES = 20;
static const float MIN_RANGE = 100;          // raw counts; less means the sensor was not turned
// Z range vs X/Y range needed for a full fit: a level turn gives ~0, turning in hand or the
// guided boom sequence with 90 deg rolls ~1-1.6. A turn at 30 deg elevation alone (~0.5) does
// not determine Z well, so it falls back to the inclination
static const float FULL_3D_RANGE_RATIO = 0.8;
static const int MAX_PARAMS = 6;

// Solves the n x n system m * x = v by Gaussian elimination with partial pivoting
static bool solve(double m[MAX_PARAMS][MAX_PARAMS], double v[MAX_PARAMS], int n, double x[MAX_PARAMS]) {
  for (int col = 0; col < n; col++) {
    int pivot = col;
    for (int row = col + 1; row < n; row++) {
      if (fabs(m[row][col]) > fabs(m[pivot][col])) pivot = row;
    }
    if (fabs(m[pivot][col]) < 1e-12) return false;
    if (pivot != col) {
      for (int k = 0; k < n; k++) {
        double t = m[col][k]; m[col][k] = m[pivot][k]; m[pivot][k] = t;
      }
      double t = v[col]; v[col] = v[pivot]; v[pivot] = t;
    }
    for (int row = col + 1; row < n; row++) {
      double f = m[row][col] / m[col][col];
      for (int k = col; k < n; k++) m[row][k] -= f * m[col][k];
      v[row] -= f * v[col];
    }
  }
  for (int row = n - 1; row >= 0; row--) {
    double sum = v[row];
    for (int k = row + 1; k < n; k++) sum -= m[row][k] * x[k];
    x[row] = sum / m[row][row];
  }
  return true;
}

// Least-squares fit of a*u^2 + b*v^2 (+ c*w^2) + d*u + e*v (+ f*w) = 1 over the given axes.
// Coordinates are centred and scaled to keep the normal equations well conditioned.
// Returns centres and semi-axes in raw counts.
static bool fitAxisAligned(const int16_t (*samples)[3], int count, int axes,
                           const double mean[3], double centre[3], double radius[3]) {
  const double unit = 1000.0;
  int n = 2 * axes;
  double m[MAX_PARAMS][MAX_PARAMS] = {};
  double v[MAX_PARAMS] = {};
  for (int i = 0; i < count; i++) {
    double row[MAX_PARAMS];
    for (int a = 0; a < axes; a++) {
      double c = (samples[i][a] - mean[a]) / unit;
      row[a] = c * c;
      row[axes + a] = c;
    }
    for (int r = 0; r < n; r++) {
      for (int k = 0; k < n; k++) m[r][k] += row[r] * row[k];
      v[r] += row[r];
    }
  }
  double p[MAX_PARAMS];
  if (!solve(m, v, n, p)) return false;

  double g = 1.0;
  for (int a = 0; a < axes; a++) {
    if (p[a] <= 0) return false; // not an ellipsoid
    g += p[axes + a] * p[axes + a] / (4 * p[a]);
  }
  if (g <= 0) return false;
  for (int a = 0; a < axes; a++) {
    centre[a] = mean[a] - p[axes + a] / (2 * p[a]) * unit;
    radius[a] = sqrt(g / p[a]) * unit;
  }
  return true;
}

MagCalibration fitMagCalibration(const int16_t (*samples)[3], int count, float inclinationDeg) {
  MagCalibration cal = {};
  if (count < MIN_SAMPLES) return cal;

  double mean[3] = {0, 0, 0};
  double minV[3], maxV[3];
  for (int a = 0; a < 3; a++) {
    minV[a] = samples[0][a];
    maxV[a] = samples[0][a];
  }
  for (int i = 0; i < count; i++) {
    for (int a = 0; a < 3; a++) {
      mean[a] += samples[i][a];
      if (samples[i][a] < minV[a]) minV[a] = samples[i][a];
      if (samples[i][a] > maxV[a]) maxV[a] = samples[i][a];
    }
  }
  for (int a = 0; a < 3; a++) mean[a] /= count;

  double rangeXY = fmin(maxV[0] - minV[0], maxV[1] - minV[1]);
  if (rangeXY < MIN_RANGE) return cal;
  cal.horizontalOnly = (maxV[2] - minV[2]) < FULL_3D_RANGE_RATIO * rangeXY;

  double centre[3], radius[3];
  if (cal.horizontalOnly) {
    if (!fitAxisAligned(samples, count, 2, mean, centre, radius)) return cal;
    double r = (radius[0] + radius[1]) / 2;
    cal.scale[0] = r / radius[0];
    cal.scale[1] = r / radius[1];
    // The axes have the same nominal gain, so Z gets the X/Y average
    cal.scale[2] = (cal.scale[0] + cal.scale[1]) / 2;
    cal.offset[0] = centre[0];
    cal.offset[1] = centre[1];
    // Level with Z up, the field points down: calibrated Z = -horizontal * tan(inclination)
    double verticalField = r * tan(inclinationDeg * M_PI / 180.0);
    cal.offset[2] = mean[2] + verticalField / cal.scale[2];
  } else {
    if (!fitAxisAligned(samples, count, 3, mean, centre, radius)) return cal;
    double r = (radius[0] + radius[1] + radius[2]) / 3;
    for (int a = 0; a < 3; a++) {
      cal.scale[a] = r / radius[a];
      cal.offset[a] = centre[a];
    }
  }
  cal.ok = true;
  return cal;
}
