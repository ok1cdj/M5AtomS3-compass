// Host test of the calibration fit, no hardware needed:
//   g++ -O2 -Wall -o /tmp/test_cal test/host/test_mag_calibration.cpp src/mag_calibration.cpp && /tmp/test_cal
#include "../../src/mag_calibration.h"
#include <cstdio>
#include <cmath>
#include <random>
int16_t buf[3000][3];
int main() {
  std::mt19937 rng(1); std::normal_distribution<double> noise(0, 15); std::uniform_real_distribution<double> u(-1, 1);
  const double off[3] = {300, 1100, 700}, gain[3] = {1.05, 0.95, 1.0}, H = 2400, I = 66 * M_PI / 180;
  const double field[3] = {H, 0, -H * tan(I)}; // north, level, pointing down
  int fails = 0;
  // 1) boom: level, turned around Z only, 2 full turns
  int n = 0;
  for (int i = 0; i < 1500; i++) {
    double h = 4 * M_PI * i / 1500;
    double b[3] = {field[0] * cos(h), -field[0] * sin(h), field[2]};
    for (int a = 0; a < 3; a++) buf[n][a] = (int16_t)lround(b[a] * gain[a] + off[a] + noise(rng));
    n++;
  }
  MagCalibration c = fitMagCalibration(buf, n, 66);
  printf("boom: ok=%d 2D=%d off %.0f %.0f %.0f  scl %.3f %.3f %.3f\n", c.ok, c.horizontalOnly, c.offset[0], c.offset[1], c.offset[2], c.scale[0], c.scale[1], c.scale[2]);
  if (!c.ok || !c.horizontalOnly || fabs(c.offset[0]-off[0]) > 20 || fabs(c.offset[1]-off[1]) > 20 || fabs(c.offset[2]-off[2]) > 300) fails++;
  // gain ratio X/Y must be corrected
  if (fabs(c.scale[0]*gain[0] - c.scale[1]*gain[1]) > 0.01) fails++;
  // 2) in hand: random orientations
  n = 0;
  for (int i = 0; i < 1500; i++) {
    double v[3]; double len;
    do { for (int a = 0; a < 3; a++) v[a] = u(rng); len = sqrt(v[0]*v[0]+v[1]*v[1]+v[2]*v[2]); } while (len > 1 || len < 0.1);
    double F = sqrt(field[0]*field[0] + field[2]*field[2]);
    for (int a = 0; a < 3; a++) buf[n][a] = (int16_t)lround(v[a] / len * F * gain[a] + off[a] + noise(rng));
    n++;
  }
  c = fitMagCalibration(buf, n, 66);
  printf("hand: ok=%d 2D=%d off %.0f %.0f %.0f  scl %.3f %.3f %.3f\n", c.ok, c.horizontalOnly, c.offset[0], c.offset[1], c.offset[2], c.scale[0], c.scale[1], c.scale[2]);
  for (int a = 0; a < 3; a++) if (fabs(c.offset[a]-off[a]) > 20) fails++;
  if (fabs(c.scale[0]*gain[0] - c.scale[2]*gain[2]) > 0.01 || fabs(c.scale[1]*gain[1] - c.scale[2]*gain[2]) > 0.01) fails++;
  // 3) not turned at all
  for (int i = 0; i < 100; i++) for (int a = 0; a < 3; a++) buf[i][a] = (int16_t)lround(off[a] + noise(rng));
  c = fitMagCalibration(buf, 100, 66);
  printf("still: ok=%d (expected 0)\n", c.ok);
  if (c.ok) fails++;
  printf(fails ? "FAILED %d\n" : "ALL PASSED\n", fails);
  return fails;
}
