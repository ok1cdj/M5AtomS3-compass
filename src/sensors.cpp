#include "sensors.h"
#include <M5Unified.h>
#include <Wire.h>

// QMC5883L shares the AtomS3 internal I2C bus (header pins), GY-511 is on the Grove port
static TwoWire &QMC_BUS = Wire1;
static const int QMC_SDA = 38;
static const int QMC_SCL = 39;
static TwoWire &LSM_BUS = Wire;
// Grove port pins: SDA G2 / SCL G1 by M5Stack pinout; the opposite order is tried as well,
// so swapped wires still work
static const int GROVE_PIN_A = 2;
static const int GROVE_PIN_B = 1;

static const uint8_t QMC5883L_ADDR = 0x0D;
static const uint8_t LSM303_ACC_ADDR = 0x19;
static const uint8_t LSM303_MAG_ADDR = 0x1E;
// LSM303D: accelerometer and magnetometer share one address (0x1D, or 0x1E with SA0 low)
static const uint8_t LSM303D_ADDRS[] = {0x1D, 0x1E};
static const uint8_t LSM303D_WHO_AM_I = 0x49;
static uint8_t lsm303dAddr = 0x1D;

// LSM303DLHC magnetometer output rate is 30 Hz; reading faster only repeats samples
static const uint32_t LSM303_MAG_PERIOD_MS = 34;

static SensorSet currentSet = SensorSet::None;
static uint32_t lastLsmMagRead = 0;

// Maps sensor chip axes to device axes: out[i] = sign[i] * in[src[i]]
struct AxisMap {
  uint8_t src[3];
  int8_t sign[3];
};

// QMC5883L axes match the AtomS3 IMU axes (verified on recorded data)
static const AxisMap QMC_MAG_MAP = {{0, 1, 2}, {1, 1, 1}};
// GY-511 mounting not verified yet: identity until measured
static const AxisMap LSM_MAG_MAP = {{0, 1, 2}, {1, 1, 1}};
static const AxisMap LSM_ACC_MAP = {{0, 1, 2}, {1, 1, 1}};
// LSM303D has one set of axes for both sensors. In the boom holder (3D/gy511_boom_mount.scad)
// the module Y arrow and cable point to the mast, so the module is turned 180 deg about Z
// against the antenna direction (device +Y)
static const AxisMap LSM303D_MAP = {{0, 1, 2}, {-1, -1, 1}};

template <typename T>
static void applyMap(const AxisMap &map, const T in[3], T &x, T &y, T &z) {
  x = map.sign[0] * in[map.src[0]];
  y = map.sign[1] * in[map.src[1]];
  z = map.sign[2] * in[map.src[2]];
}

static bool probe(TwoWire &bus, uint8_t addr) {
  bus.beginTransmission(addr);
  return bus.endTransmission() == 0;
}

static bool writeReg(TwoWire &bus, uint8_t addr, uint8_t reg, uint8_t value) {
  bus.beginTransmission(addr);
  bus.write(reg);
  bus.write(value);
  return bus.endTransmission() == 0;
}

static bool readRegs(TwoWire &bus, uint8_t addr, uint8_t reg, uint8_t *buf, size_t len) {
  bus.beginTransmission(addr);
  bus.write(reg);
  if (bus.endTransmission(false) != 0) return false;
  if (bus.requestFrom(addr, (uint8_t)len) != len) return false;
  for (size_t i = 0; i < len; i++) buf[i] = bus.read();
  return true;
}

static bool detectLsm303() {
  if (!probe(LSM_BUS, LSM303_ACC_ADDR) || !probe(LSM_BUS, LSM303_MAG_ADDR)) return false;
  uint8_t id[3];
  // Identification registers IRA/IRB/IRC_REG_M read "H43"
  return readRegs(LSM_BUS, LSM303_MAG_ADDR, 0x0A, id, 3) && id[0] == 'H' && id[1] == '4' && id[2] == '3';
}

static bool detectLsm303d() {
  for (uint8_t addr : LSM303D_ADDRS) {
    uint8_t id;
    if (probe(LSM_BUS, addr) && readRegs(LSM_BUS, addr, 0x0F, &id, 1) && id == LSM303D_WHO_AM_I) {
      lsm303dAddr = addr;
      return true;
    }
  }
  return false;
}

static bool detectQmc5883l() {
  uint8_t id;
  // Chip ID register reads 0xFF
  return probe(QMC_BUS, QMC5883L_ADDR) && readRegs(QMC_BUS, QMC5883L_ADDR, 0x0D, &id, 1) && id == 0xFF;
}

static bool initLsm303() {
  bool ok = true;
  ok &= writeReg(LSM_BUS, LSM303_MAG_ADDR, 0x00, 0x14); // CRA_REG_M: 30 Hz, temperature sensor off
  ok &= writeReg(LSM_BUS, LSM303_MAG_ADDR, 0x01, 0x20); // CRB_REG_M: +-1.3 gauss
  ok &= writeReg(LSM_BUS, LSM303_MAG_ADDR, 0x02, 0x00); // MR_REG_M: continuous conversion
  ok &= writeReg(LSM_BUS, LSM303_ACC_ADDR, 0x20, 0x57); // CTRL_REG1_A: 100 Hz, X/Y/Z enabled
  ok &= writeReg(LSM_BUS, LSM303_ACC_ADDR, 0x23, 0x08); // CTRL_REG4_A: +-2 g, high resolution
  return ok;
}

static bool initLsm303d() {
  bool ok = true;
  ok &= writeReg(LSM_BUS, lsm303dAddr, 0x20, 0x6F); // CTRL1: 100 Hz, block data update, X/Y/Z enabled
  ok &= writeReg(LSM_BUS, lsm303dAddr, 0x21, 0xC0); // CTRL2: anti-alias filter 50 Hz, +-2 g
  ok &= writeReg(LSM_BUS, lsm303dAddr, 0x24, 0x70); // CTRL5: magnetometer high resolution, 50 Hz
  ok &= writeReg(LSM_BUS, lsm303dAddr, 0x25, 0x00); // CTRL6: +-2 gauss
  ok &= writeReg(LSM_BUS, lsm303dAddr, 0x26, 0x00); // CTRL7: continuous conversion
  return ok;
}

static bool initQmc5883l() {
  bool ok = true;
  ok &= writeReg(QMC_BUS, QMC5883L_ADDR, 0x0A, 0x80); // CONTROL_2: soft reset
  delay(10);
  ok &= writeReg(QMC_BUS, QMC5883L_ADDR, 0x0B, 0x01); // SET/RESET period, recommended by datasheet
  // CONTROL_1: OSR=512, RNG=2G, ODR=50Hz, continuous. 2G gives 4x the resolution of 8G;
  // the Earth's field (~0.5 G) fits easily
  ok &= writeReg(QMC_BUS, QMC5883L_ADDR, 0x09, 0x05);
  return ok;
}

// Logs all responding I2C addresses with their WHO_AM_I register (0x0F), to diagnose
// wiring and chip type when no sensor is detected
static void logI2cScan(TwoWire &bus, const char *name) {
  String found;
  for (uint8_t addr = 0x08; addr < 0x78; addr++) {
    if (!probe(bus, addr)) continue;
    uint8_t id;
    char entry[24];
    if (readRegs(bus, addr, 0x0F, &id, 1)) {
      snprintf(entry, sizeof(entry), " 0x%02X(id 0x%02X)", addr, id);
    } else {
      snprintf(entry, sizeof(entry), " 0x%02X", addr);
    }
    found += entry;
  }
  M5.Log.printf("I2C %s:%s\n", name, found.isEmpty() ? " no devices" : found.c_str());
}

// Tries a GY-511 on the Grove port with the given pin order
static bool detectGrove(int sda, int scl) {
  LSM_BUS.begin(sda, scl);
  char name[32];
  snprintf(name, sizeof(name), "Grove (SDA G%d, SCL G%d)", sda, scl);
  logI2cScan(LSM_BUS, name);
  if (detectLsm303d() && initLsm303d()) {
    currentSet = SensorSet::LSM303D;
    return true;
  }
  if (detectLsm303() && initLsm303()) {
    currentSet = SensorSet::LSM303DLHC;
    return true;
  }
  LSM_BUS.end();
  return false;
}

bool sensorsBegin() {
  QMC_BUS.begin(QMC_SDA, QMC_SCL);
  logI2cScan(QMC_BUS, "internal (SDA 38, SCL 39)");

  currentSet = SensorSet::None;
  if (!detectGrove(GROVE_PIN_A, GROVE_PIN_B) && !detectGrove(GROVE_PIN_B, GROVE_PIN_A)) {
    if (detectQmc5883l() && initQmc5883l()) {
      currentSet = SensorSet::QMC5883L_InternalImu;
    }
  }
  M5.Log.printf("Sensor set: %s\n", sensorName());
  return currentSet != SensorSet::None;
}

SensorSet sensorSet() {
  return currentSet;
}

const char* sensorName() {
  switch (currentSet) {
    case SensorSet::QMC5883L_InternalImu: return "QMC5883L";
    case SensorSet::LSM303DLHC: return "LSM303DLHC";
    case SensorSet::LSM303D: return "LSM303D";
    default: return "none";
  }
}

const char* sensorKeyPrefix() {
  switch (currentSet) {
    case SensorSet::QMC5883L_InternalImu: return "q_";
    case SensorSet::LSM303DLHC: return "l_";
    // Changed with the 180 deg axis map: calibration stored in the old axes is not valid
    case SensorSet::LSM303D: return "dr_";
    default: return "n_";
  }
}

static bool readQmcMag(int &x, int &y, int &z) {
  uint8_t status;
  // STATUS: bit0 = data ready, bit1 = overflow (sample invalid)
  if (!readRegs(QMC_BUS, QMC5883L_ADDR, 0x06, &status, 1) || !(status & 0x01)) return false;
  uint8_t buf[6];
  if (!readRegs(QMC_BUS, QMC5883L_ADDR, 0x00, buf, 6)) return false;
  if (status & 0x02) return false;
  int raw[3] = {
    (int16_t)(buf[0] | (buf[1] << 8)),
    (int16_t)(buf[2] | (buf[3] << 8)),
    (int16_t)(buf[4] | (buf[5] << 8))
  };
  applyMap(QMC_MAG_MAP, raw, x, y, z);
  return true;
}

static bool readLsmMag(int &x, int &y, int &z) {
  if (millis() - lastLsmMagRead < LSM303_MAG_PERIOD_MS) return false;
  uint8_t buf[6];
  if (!readRegs(LSM_BUS, LSM303_MAG_ADDR, 0x03, buf, 6)) return false;
  lastLsmMagRead = millis();
  // Output order is X, Z, Y, big-endian
  int raw[3] = {
    (int16_t)((buf[0] << 8) | buf[1]),
    (int16_t)((buf[4] << 8) | buf[5]),
    (int16_t)((buf[2] << 8) | buf[3])
  };
  // -4096 marks an overflowed axis
  if (raw[0] == -4096 || raw[1] == -4096 || raw[2] == -4096) return false;
  applyMap(LSM_MAG_MAP, raw, x, y, z);
  return true;
}

static bool readLsm303dMag(int &x, int &y, int &z) {
  uint8_t status;
  // STATUS_M: bit3 = new X/Y/Z data
  if (!readRegs(LSM_BUS, lsm303dAddr, 0x07, &status, 1) || !(status & 0x08)) return false;
  uint8_t buf[6];
  // MSB of the register address enables auto-increment; X, Y, Z little-endian
  if (!readRegs(LSM_BUS, lsm303dAddr, 0x08 | 0x80, buf, 6)) return false;
  int raw[3] = {
    (int16_t)(buf[0] | (buf[1] << 8)),
    (int16_t)(buf[2] | (buf[3] << 8)),
    (int16_t)(buf[4] | (buf[5] << 8))
  };
  applyMap(LSM303D_MAP, raw, x, y, z);
  return true;
}

bool readMagRaw(int &x, int &y, int &z) {
  switch (currentSet) {
    case SensorSet::QMC5883L_InternalImu: return readQmcMag(x, y, z);
    case SensorSet::LSM303DLHC: return readLsmMag(x, y, z);
    case SensorSet::LSM303D: return readLsm303dMag(x, y, z);
    default: return false;
  }
}

bool readMagRawWait(int &x, int &y, int &z, uint32_t timeoutMs) {
  uint32_t start = millis();
  while (millis() - start < timeoutMs) {
    if (readMagRaw(x, y, z)) return true;
    delay(2);
  }
  return false;
}

bool readAccRaw(float &x, float &y, float &z) {
  if (currentSet == SensorSet::QMC5883L_InternalImu) {
    M5.Imu.update();
    return M5.Imu.getAccelData(&x, &y, &z);
  }
  if (currentSet == SensorSet::LSM303DLHC) {
    uint8_t buf[6];
    // MSB of the register address enables auto-increment
    if (!readRegs(LSM_BUS, LSM303_ACC_ADDR, 0x28 | 0x80, buf, 6)) return false;
    // 12-bit left-justified, 1 mg/LSB at +-2 g high resolution
    float raw[3] = {
      ((int16_t)(buf[0] | (buf[1] << 8)) >> 4) / 1000.0f,
      ((int16_t)(buf[2] | (buf[3] << 8)) >> 4) / 1000.0f,
      ((int16_t)(buf[4] | (buf[5] << 8)) >> 4) / 1000.0f
    };
    applyMap(LSM_ACC_MAP, raw, x, y, z);
    return true;
  }
  if (currentSet == SensorSet::LSM303D) {
    uint8_t buf[6];
    if (!readRegs(LSM_BUS, lsm303dAddr, 0x28 | 0x80, buf, 6)) return false;
    // 16-bit, 0.061 mg/LSB at +-2 g
    float raw[3] = {
      (int16_t)(buf[0] | (buf[1] << 8)) * 0.000061f,
      (int16_t)(buf[2] | (buf[3] << 8)) * 0.000061f,
      (int16_t)(buf[4] | (buf[5] << 8)) * 0.000061f
    };
    applyMap(LSM303D_MAP, raw, x, y, z);
    return true;
  }
  return false;
}
