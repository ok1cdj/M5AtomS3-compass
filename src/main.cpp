#include <Arduino.h>
#include <M5Unified.h>
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include "esp32-hal-cpu.h"
#include <Preferences.h>
#include "sensors.h"
#include "network.h"
#include "web.h"
#include "battery.h"
#include "mag_calibration.h"

AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

static M5Canvas canvas(&M5.Display);
Preferences preferences;

String azimuth;
String elevation;

unsigned long previousMillis = 0UL;
unsigned long interval = 250UL;

// Sensors are sampled faster than the display refresh and low-pass filtered
const unsigned long SENSOR_INTERVAL_MS = 20;
const float SENSOR_FILTER_ALPHA = 0.1;
unsigned long lastSensorRead = 0;
bool filterInitialized = false;
float magF[3];     // filtered calibrated magnetic field
float accF[3];     // filtered calibrated acceleration
int lastRaw[3];    // last raw magnetometer sample, for diagnostics

// Calibration of the detected sensor set, stored in NVS under sensorKeyPrefix()
bool calibrated = false;
int offsetX = 0;
int offsetY = 0;
int offsetZ = 0;
const unsigned long LEVEL_CALIBRATION_TIME_MS = 3000;
// Magnetic inclination of the location (web setting), used for the Z offset when the sensor
// can only be turned around the vertical (antenna boom) and to check a calibration.
// 66 deg in Central Europe; see the NOAA magnetic field calculator.
float magneticInclination = 66.0;
// Calibration quality: measured at the end of a calibration while level and still
float calInclination = NAN;  // measured inclination, deg
float calFieldNorm = 0;      // calibrated field magnitude
bool calHorizontalOnly = false;
const float CAL_INCLINATION_TOLERANCE_DEG = 3;  // DONE screen: measured vs set inclination
// CHECK CAL: field magnitude or inclination far from the calibration values for a while
const float CHECK_CAL_NORM_RATIO = 0.2;
const float CHECK_CAL_INCLINATION_DEG = 8;
const float CHECK_CAL_FILTER_ALPHA = 0.05;     // per display update (4 Hz): ~5 s
float checkNormRatio = 1.0;
float checkInclinationDiff = 0;
bool calSuspect = false;
// Guided calibration: the screen tells what to do; samples are collected only during the
// phases, not while the antenna is being moved to the next position
// Live angle shown during a phase, from the accelerometer
enum class CalibrationAngle { Elevation, Roll };
struct CalibrationPhase {
  const char *title;
  const char *action;
  unsigned long durationMs;
  CalibrationAngle angle;
  float targetDeg; // roll: positive = right side down, looking along the antenna
};
const CalibrationPhase CALIBRATION_PHASES[] = {
  {"LEVEL", "Turn 360", 30000, CalibrationAngle::Elevation, 0},
  {"EL 30", "Turn 360", 30000, CalibrationAngle::Elevation, 30},
  {"ROLL L 90", "Hold", 15000, CalibrationAngle::Roll, -90},
  {"ROLL R 90", "Hold", 15000, CalibrationAngle::Roll, 90},
};
const float CALIBRATION_ANGLE_TOLERANCE_DEG = 5; // angle shown green within this of the target
const unsigned long CALIBRATION_REDRAW_MS = 200;
const unsigned long CALIBRATION_MOVE_TIME_MS = 8000; // to move to the next position
// Samples kept for the calibration fit, enough for all phases at 50 Hz
const int CALIBRATION_MAX_SAMPLES = 4800;
static int16_t calibrationSamples[CALIBRATION_MAX_SAMPLES][3];
// Per-axis gain correction (soft-iron / sensor gain mismatch)
float scaleX = 1.0;
float scaleY = 1.0;
float scaleZ = 1.0;
// Accelerometer zero when lying flat
float accOffX = 0.0;
float accOffY = 0.0;

int magneticDeclination = 5;
// Requests from the WebSocket handler (async_tcp task), applied in loop()
const int NO_PENDING_DECLINATION = -1000;
volatile int pendingDeclination = NO_PENDING_DECLINATION;
volatile float pendingInclination = NAN;
volatile bool calResetRequested = false;
bool declinationMode = false;
unsigned long lastDeclinationSetTime = 0;

uint32_t pressStartTime = 0;

const int BATTERY_WARN_PERCENT = 25;
const int BATTERY_LOW_PERCENT = 10;

String settingsJson() {
  char json[220];
  snprintf(json, sizeof(json),
           "{\"settings\":{\"decl\":%d,\"incl\":%.1f,\"sensor\":\"%s\",\"calibrated\":%s,"
           "\"calMode\":\"%s\",\"calIncl\":%s,\"check\":%s}}",
           magneticDeclination, magneticInclination, sensorName(), calibrated ? "true" : "false",
           calHorizontalOnly ? "2D" : "3D", isnan(calInclination) ? "null" : String(calInclination, 1).c_str(),
           calSuspect ? "true" : "false");
  return String(json);
}

// Runs in the async_tcp task: only records requests, loop() applies and saves them
void onWsEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type, void *arg, uint8_t *data, size_t len) {
  if (type == WS_EVT_CONNECT) {
    client->text(settingsJson());
  } else if (type == WS_EVT_DATA) {
    AwsFrameInfo *info = (AwsFrameInfo*)arg;
    if (info->final && info->index == 0 && info->len == len && info->opcode == WS_TEXT) {
      String message = String((const char *)data, len);
      if (message.startsWith("decl:")) {
        int decl = message.substring(5).toInt();
        if (decl >= -90 && decl <= 90) pendingDeclination = decl;
      } else if (message.startsWith("incl:")) {
        float incl = message.substring(5).toFloat();
        if (incl >= 0 && incl <= 90) pendingInclination = incl;
      } else if (message == "calreset") {
        calResetRequested = true;
      }
    }
  }
}

void runCalibration(); // Forward declaration

// Set by holding the button during startup; WiFi stays off until the next reset
bool wifiDisabled = false;
const unsigned long WIFI_OFF_MESSAGE_MS = 1500;

void drawWiFiStatusBar() {
  canvas.setTextSize(1);
  canvas.setTextDatum(BC_DATUM); // Bottom-Center datum
  if (wifiDisabled) {
      canvas.setTextColor(DARKGREY);
      canvas.drawString("WiFi OFF", canvas.width() / 2, canvas.height() - 2);
  } else if (WiFi.status() == WL_CONNECTED) {
      canvas.setTextColor(CYAN);
      canvas.drawString(WiFi.localIP().toString(), canvas.width() / 2, canvas.height() - 2);
  } else if (networkApActive()) {
      canvas.setTextColor(YELLOW);
      canvas.drawString("AP: Compass_Setup", canvas.width() / 2, canvas.height() - 2);
  } else {
      canvas.setTextColor(ORANGE);
      canvas.drawString("WiFi Disconnected", canvas.width() / 2, canvas.height() - 2);
  }
  canvas.setTextColor(WHITE); // Reset text color
}

String calKey(const char* name) {
  return String(sensorKeyPrefix()) + name;
}

// Preferences::getFloat logs an error for a missing key, so check it first
float getFloatOr(const String &key, float defaultValue) {
  return preferences.isKey(key.c_str()) ? preferences.getFloat(key.c_str(), defaultValue) : defaultValue;
}

void loadCalibration() {
  preferences.begin("compass", true);
  calibrated = preferences.getBool(calKey("cal").c_str(), false);
  offsetX = preferences.getInt(calKey("offX").c_str(), 0);
  offsetY = preferences.getInt(calKey("offY").c_str(), 0);
  offsetZ = preferences.getInt(calKey("offZ").c_str(), 0);
  scaleX = getFloatOr(calKey("sclX"), 1.0);
  scaleY = getFloatOr(calKey("sclY"), 1.0);
  scaleZ = getFloatOr(calKey("sclZ"), 1.0);
  accOffX = getFloatOr(calKey("accX"), 0.0);
  accOffY = getFloatOr(calKey("accY"), 0.0);
  calInclination = getFloatOr(calKey("cIncl"), NAN);
  calFieldNorm = getFloatOr(calKey("cNorm"), 0.0);
  calHorizontalOnly = preferences.getBool(calKey("c2D").c_str(), false);
  preferences.end();
  M5.Log.printf("Calibration (%s): %s, offsets %d, %d, %d, scales %.3f, %.3f, %.3f, acc %.3f, %.3f\n",
                sensorName(), calibrated ? "yes" : "no", offsetX, offsetY, offsetZ,
                scaleX, scaleY, scaleZ, accOffX, accOffY);
}

void saveCalibration() {
  preferences.begin("compass", false);
  preferences.putBool(calKey("cal").c_str(), calibrated);
  preferences.putInt(calKey("offX").c_str(), offsetX);
  preferences.putInt(calKey("offY").c_str(), offsetY);
  preferences.putInt(calKey("offZ").c_str(), offsetZ);
  preferences.putFloat(calKey("sclX").c_str(), scaleX);
  preferences.putFloat(calKey("sclY").c_str(), scaleY);
  preferences.putFloat(calKey("sclZ").c_str(), scaleZ);
  preferences.putFloat(calKey("accX").c_str(), accOffX);
  preferences.putFloat(calKey("accY").c_str(), accOffY);
  preferences.putFloat(calKey("cIncl").c_str(), calInclination);
  preferences.putFloat(calKey("cNorm").c_str(), calFieldNorm);
  preferences.putBool(calKey("c2D").c_str(), calHorizontalOnly);
  preferences.end();
}

void resetCalibration() {
  calibrated = false;
  offsetX = offsetY = offsetZ = 0;
  scaleX = scaleY = scaleZ = 1.0;
  accOffX = accOffY = 0.0;
  calInclination = NAN;
  calFieldNorm = 0;
  calHorizontalOnly = false;
  calSuspect = false;
  saveCalibration();
  filterInitialized = false;
  M5.Log.printf("Calibration of %s reset\n", sensorName());
}

// Applies settings requested over the WebSocket and tells all clients the new state
void applyPendingSettings() {
  bool changed = false;
  if (pendingDeclination != NO_PENDING_DECLINATION) {
    magneticDeclination = pendingDeclination;
    pendingDeclination = NO_PENDING_DECLINATION;
    preferences.begin("compass", false);
    preferences.putInt("decl", magneticDeclination);
    preferences.end();
    M5.Log.printf("Declination set to %d\n", magneticDeclination);
    changed = true;
  }
  if (!isnan(pendingInclination)) {
    magneticInclination = pendingInclination;
    pendingInclination = NAN;
    preferences.begin("compass", false);
    preferences.putFloat("incl", magneticInclination);
    preferences.end();
    M5.Log.printf("Inclination set to %.1f\n", magneticInclination);
    changed = true;
  }
  if (calResetRequested) {
    calResetRequested = false;
    resetCalibration();
    changed = true;
  }
  if (changed) ws.textAll(settingsJson());
}

// Reads the sensors and updates the filtered, calibrated vectors
void updateSensors() {
  int rx, ry, rz;
  float ax, ay, az;
  if (!readAccRaw(ax, ay, az)) return;
  if (!readMagRaw(rx, ry, rz)) return;

  lastRaw[0] = rx;
  lastRaw[1] = ry;
  lastRaw[2] = rz;
  float m[3] = {(rx - offsetX) * scaleX, (ry - offsetY) * scaleY, (rz - offsetZ) * scaleZ};
  float acc[3] = {ax - accOffX, ay - accOffY, az};

  // Filter the vectors, not the angle, so there is no jump at 359 -> 0
  for (int i = 0; i < 3; i++) {
    if (filterInitialized) {
      magF[i] += SENSOR_FILTER_ALPHA * (m[i] - magF[i]);
      accF[i] += SENSOR_FILTER_ALPHA * (acc[i] - accF[i]);
    } else {
      magF[i] = m[i];
      accF[i] = acc[i];
    }
  }
  filterInitialized = true;
}

void setup() {
  btStop();
  setCpuFrequencyMhz(80); //Set CPU clock to 80MHz fo example
  M5.begin();
  M5.Log.println("Starting setup...");
  canvas.createSprite(M5.Lcd.width(), M5.Lcd.height());

  // Holding the button during startup disables WiFi (saves battery in the field)
  M5.update();
  if (M5.BtnA.isPressed()) {
    wifiDisabled = true;
    WiFi.mode(WIFI_OFF);
    M5.Log.println("WiFi disabled by button at startup");
    canvas.fillSprite(BLACK);
    canvas.setTextDatum(MC_DATUM);
    canvas.setTextSize(2);
    canvas.setTextColor(ORANGE);
    canvas.drawString("WiFi OFF", canvas.width() / 2, canvas.height() / 2);
    canvas.setTextColor(WHITE);
    canvas.pushSprite(0, 0);
    delay(WIFI_OFF_MESSAGE_MS);
  }
  
  preferences.begin("compass", false);
  magneticDeclination = preferences.getInt("decl", 5);
  magneticInclination = getFloatOr("incl", 66.0);
  preferences.end();

  sensorsBegin();
  loadCalibration();
  batteryBegin();
  M5.Lcd.setRotation(0);

  ws.onEvent(onWsEvent);
  server.addHandler(&ws);

  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    sendEmbeddedPage(request, "index.html");
  });

  server.on("/settings", HTTP_GET, [](AsyncWebServerRequest *request){
    sendEmbeddedPage(request, "settings.html");
  });

  if (!wifiDisabled) {
    networkBegin(server);
    server.begin();
  }
}

void loop() {
  int a;
  M5.update();
  batteryUpdate();
  applyPendingSettings();

  if (declinationMode) {
    if (M5.BtnA.wasReleased()) {
        magneticDeclination++;
        if (magneticDeclination > 10) {
            magneticDeclination = -10;
        }
        lastDeclinationSetTime = millis();
    }

    canvas.fillSprite(BLACK);
    canvas.setTextDatum(MC_DATUM);
    canvas.setTextSize(2);
    canvas.drawString("Set Declination", canvas.width() / 2, 40);
    canvas.setTextSize(5);
    String decl_str = String(magneticDeclination);
    canvas.drawString(decl_str, canvas.width() / 2, 85);
    int numWidth = canvas.textWidth(decl_str);
    // Draw a circle for the degree symbol
    canvas.drawCircle(canvas.width() / 2 + numWidth / 2 + 6, 85 - (8*5)/2 + 4, 4, WHITE);

    drawWiFiStatusBar();

    canvas.pushSprite(0, 0);

    if (millis() - lastDeclinationSetTime > 2000) {
        preferences.begin("compass", false);
        preferences.putInt("decl", magneticDeclination);
        preferences.end();
        declinationMode = false;
        canvas.fillSprite(BLACK);
        canvas.pushSprite(0, 0);
    }
  } else {
    unsigned long currentMillis = millis();
    if (currentMillis - lastSensorRead >= SENSOR_INTERVAL_MS) {
      lastSensorRead = currentMillis;
      updateSensors();
    }

    if (sensorSet() == SensorSet::None && currentMillis - previousMillis > interval) {
      canvas.fillSprite(BLACK);
      canvas.setTextDatum(MC_DATUM);
      canvas.setTextSize(2);
      canvas.setTextColor(RED);
      canvas.drawString("NO SENSOR", canvas.width() / 2, canvas.height() / 2);
      canvas.setTextColor(WHITE);
      if (batteryPresent()) {
        canvas.setTextSize(1);
        canvas.drawString("BAT " + String(batteryPercent()) + "% " + String(batteryVoltage(), 2) + "V",
                          canvas.width() / 2, canvas.height() / 2 + 20);
      }
      drawWiFiStatusBar();
      canvas.pushSprite(0, 0);
      previousMillis = currentMillis;
    }

    if (filterInitialized && currentMillis - previousMillis > interval)
    {
      float accX = accF[0], accY = accF[1], accZ = accF[2];
      float mx = magF[0], my = magF[1], mz = magF[2];

      // Elevation (tilt of Y axis, the arrow direction) for display
      float elevation = atan2(accY, sqrt(accX * accX + accZ * accZ)) * 180.0 / M_PI;

      // Tilt compensation: build East and North in the device frame from gravity and field.
      // Accelerometer reads +1 g upwards at rest, so down = -acc.
      float acc_norm = sqrt(accX * accX + accY * accY + accZ * accZ);
      float dx = -accX / acc_norm, dy = -accY / acc_norm, dz = -accZ / acc_norm;
      // East = down x field
      float ex = dy * mz - dz * my;
      float ey = dz * mx - dx * mz;
      float ez = dx * my - dy * mx;
      // North = East x down
      float ny = ez * dx - ex * dz;
      // Heading of the arrow (device +Y axis): atan2(forward . East, forward . North)
      float heading = atan2(ey, ny);

      // Diagnostics: field magnitude and angle between field and gravity must stay
      // constant in any orientation; if they change with tilt, calibration is wrong
      float mag_norm = sqrt(mx * mx + my * my + mz * mz);
      float dip = acos((mx * accX + my * accY + mz * accZ) / (mag_norm * acc_norm)) * 180.0 / M_PI;
      M5.Log.printf("Diag: raw=%d,%d,%d acc=%.2f,%.2f,%.2f mag=%.0f,%.0f,%.0f |m|=%.0f dip=%.1f\n",
                    lastRaw[0], lastRaw[1], lastRaw[2], accX, accY, accZ, mx, my, mz, mag_norm, dip);

      // CHECK CAL: the field should keep the magnitude and inclination it had at calibration
      if (calibrated && calFieldNorm > 0 && !isnan(calInclination)) {
        const float mag[3] = {mx, my, mz};
        const float acc[3] = {accX, accY, accZ};
        float inclination = fieldInclination(mag, acc);
        checkNormRatio += CHECK_CAL_FILTER_ALPHA * (mag_norm / calFieldNorm - checkNormRatio);
        checkInclinationDiff += CHECK_CAL_FILTER_ALPHA * (inclination - calInclination - checkInclinationDiff);
        bool suspect = fabs(checkNormRatio - 1) > CHECK_CAL_NORM_RATIO ||
                       fabs(checkInclinationDiff) > CHECK_CAL_INCLINATION_DEG;
        if (suspect != calSuspect) {
          calSuspect = suspect;
          M5.Log.printf("Calibration check: %s (field %.2f x, inclination %+.1f deg)\n",
                        suspect ? "suspect" : "ok", checkNormRatio, checkInclinationDiff);
          ws.textAll(settingsJson());
        }
      }

      heading += magneticDeclination * M_PI / 180.0;

      a = (int)round(heading * 180 / M_PI) % 360;
      if (a < 0) a += 360;

      M5.Log.printf("Calculated Azimuth: %d, Elevation: %.1f\n", a, elevation);

      canvas.fillSprite(BLACK);

      // Status bar for calibration
      if (calibrated && calSuspect) {
          canvas.setTextSize(1);
          canvas.setTextColor(ORANGE);
          canvas.setTextDatum(TC_DATUM); // Top-Center datum
          canvas.drawString("CHECK CAL", canvas.width() / 2, 2);
      } else if (calibrated) {
          canvas.setTextSize(1);
          canvas.setTextColor(GREEN);
          canvas.setTextDatum(TC_DATUM); // Top-Center datum
          String calStatusText = "CALIBRATED " + String(magneticDeclination);
          int textWidth = canvas.textWidth(calStatusText);
          // Draw text shifted left to make space for the circle, keeping the group centered
          canvas.drawString(calStatusText, canvas.width() / 2 - 3, 2);
          // Draw a circle for the degree symbol
          canvas.drawCircle(canvas.width() / 2 + textWidth / 2, 4, 2, GREEN);
      } else {
          canvas.setTextSize(1);
          canvas.setTextColor(RED);
          canvas.setTextDatum(TC_DATUM); // Top-Center datum
          canvas.drawString("UNCALIBRATED", canvas.width() / 2, 2);
      }
      canvas.setTextColor(WHITE); // Reset text color

      // Battery level (only with the Atomic Battery Base)
      if (batteryPresent()) {
          int percent = batteryPercent();
          canvas.setTextSize(1);
          canvas.setTextDatum(TR_DATUM); // Top-Right datum
          canvas.setTextColor(percent <= BATTERY_LOW_PERCENT ? RED : (percent <= BATTERY_WARN_PERCENT ? ORANGE : GREEN));
          canvas.drawString(String(percent) + "%", canvas.width() - 1, 2);
          canvas.setTextColor(WHITE);
      }

      // Red triangle arrow
      int centerX = canvas.width() / 2;
      canvas.fillTriangle(centerX, 25, centerX - 8, 40, centerX + 8, 40, RED);

      // Centered degrees
      canvas.setTextSize(5);
      canvas.setTextDatum(MC_DATUM); // Middle-Center datum
      canvas.drawString(String(a), centerX, 75);

      // Elevation value
      canvas.setTextSize(2);
      String elev_str = "Elev: " + String((int)elevation);
      canvas.drawString(elev_str, centerX, 110);

      drawWiFiStatusBar();

      canvas.pushSprite(0, 0);
      // Raw sensor data is included for remote diagnostics of axis mapping and calibration
      char bat_json[40];
      if (batteryPresent()) {
        snprintf(bat_json, sizeof(bat_json), "{\"v\":%.2f,\"p\":%d}", batteryVoltage(), batteryPercent());
      } else {
        strcpy(bat_json, "null");
      }
      char json_data[360];
      snprintf(json_data, sizeof(json_data),
               "{\"azimuth\":%d, \"elev\":%d, \"sensor\":\"%s\", \"acc\":[%.3f,%.3f,%.3f], \"raw\":[%d,%d,%d], \"off\":[%d,%d,%d], \"scl\":[%.3f,%.3f,%.3f], \"dip\":%.1f, \"bat\":%s, \"chk\":%s}",
               a, (int)elevation, sensorName(), accX, accY, accZ, lastRaw[0], lastRaw[1], lastRaw[2],
               offsetX, offsetY, offsetZ, scaleX, scaleY, scaleZ, dip, bat_json, calSuspect ? "true" : "false");
      ws.textAll(json_data);
      previousMillis = currentMillis;
    }

    ws.cleanupClients();

    if (M5.BtnA.wasPressed()) {
      pressStartTime = millis();
    }

    if (M5.BtnA.wasReleased()) {
      if (pressStartTime > 0) {
        uint32_t pressed_ms = millis() - pressStartTime;
        if (pressed_ms >= 5000) {
          declinationMode = true;
          lastDeclinationSetTime = millis();
        } else if (pressed_ms >= 2000 && sensorSet() != SensorSet::None) {
          runCalibration();
        }
        pressStartTime = 0;
      }
    }
  }
}

void showCalibrationResult(uint16_t color, const char *line1, const char *line2) {
  canvas.fillSprite(color);
  canvas.setTextDatum(MC_DATUM);
  canvas.setTextSize(2);
  canvas.drawString(line1, canvas.width() / 2, 50);
  canvas.drawString(line2, canvas.width() / 2, 80);
  canvas.pushSprite(0, 0);
  delay(2000);
}

// Elevation (+ = antenna up) or roll (+ = right side down) in degrees, NAN if not available
float calibrationAngle(CalibrationAngle angle) {
  float ax, ay, az;
  if (!readAccRaw(ax, ay, az)) return NAN;
  ax -= accOffX;
  ay -= accOffY;
  if (angle == CalibrationAngle::Elevation) {
    return atan2(ay, sqrt(ax * ax + az * az)) * 180.0 / M_PI;
  }
  // Device +X points to the right of the antenna, so right side down gives negative X
  return atan2(-ax, az) * 180.0 / M_PI;
}

// Text size 2 fits 10 characters on the 128 px display
void drawCalibrationScreen(uint16_t color, const String &step, const CalibrationPhase &phase,
                           const char *action, int secondsLeft) {
  canvas.fillSprite(color);
  canvas.setTextDatum(MC_DATUM);
  canvas.setTextSize(1);
  canvas.drawString(step, canvas.width() / 2, 8);
  canvas.setTextSize(2);
  canvas.drawString(phase.title, canvas.width() / 2, 28);
  canvas.drawString(action, canvas.width() / 2, 50);

  // Live angle, green when on target
  float angle = calibrationAngle(phase.angle);
  if (!isnan(angle)) {
    String text;
    if (phase.angle == CalibrationAngle::Elevation) {
      text = "EL " + String((int)round(angle));
    } else {
      text = String(angle >= 0 ? "R " : "L ") + String((int)round(fabs(angle)));
    }
    canvas.fillRect(14, 62, canvas.width() - 28, 24, BLACK);
    canvas.setTextColor(fabs(angle - phase.targetDeg) <= CALIBRATION_ANGLE_TOLERANCE_DEG ? GREEN : YELLOW);
    canvas.drawString(text, canvas.width() / 2, 74);
    canvas.setTextColor(WHITE);
  }

  canvas.setTextSize(3);
  canvas.drawString(String(secondsLeft), canvas.width() / 2, 108);
  canvas.pushSprite(0, 0);
}

// Waits while the antenna is moved to the next position; no samples are taken.
// The live angle helps to set the position.
void calibrationMovePause(const CalibrationPhase &next) {
  unsigned long start = millis();
  while (millis() - start < CALIBRATION_MOVE_TIME_MS) {
    int secondsLeft = (CALIBRATION_MOVE_TIME_MS - (millis() - start) + 999) / 1000;
    drawCalibrationScreen(ORANGE, "Move to next position", next, "", secondsLeft);
    M5.update();
    delay(CALIBRATION_REDRAW_MS);
  }
}

void runCalibration() {
  // Sensor in hand: turn it in all directions during all phases. On an antenna boom follow
  // the phases: a level turn, a turn at 30 deg elevation, then 90 deg rolls to both sides
  int count = 0;
  int levelCount = 0; // samples of the first (level) phase
  const int phaseCount = sizeof(CALIBRATION_PHASES) / sizeof(CALIBRATION_PHASES[0]);
  for (int p = 0; p < phaseCount; p++) {
    const CalibrationPhase &phase = CALIBRATION_PHASES[p];
    if (p > 0) calibrationMovePause(phase);

    unsigned long start = millis();
    unsigned long lastRedraw = 0;
    while (millis() - start < phase.durationMs) {
      if (lastRedraw == 0 || millis() - lastRedraw >= CALIBRATION_REDRAW_MS) {
        lastRedraw = millis();
        int secondsLeft = (phase.durationMs - (millis() - start) + 999) / 1000;
        drawCalibrationScreen(RED, "CAL " + String(p + 1) + "/" + String(phaseCount), phase,
                              phase.action, secondsLeft);
      }
      int rawX, rawY, rawZ;
      if (readMagRawWait(rawX, rawY, rawZ) && count < CALIBRATION_MAX_SAMPLES) {
        calibrationSamples[count][0] = rawX;
        calibrationSamples[count][1] = rawY;
        calibrationSamples[count][2] = rawZ;
        count++;
      }
      M5.update(); // Keep M5 services running
    }
    if (p == 0) levelCount = count;
  }

  MagCalibration fit = fitMagCalibration(calibrationSamples, count, levelCount, magneticInclination);
  if (!fit.ok) {
    M5.Log.printf("Calibration failed (%d samples), previous calibration kept\n", count);
    showCalibrationResult(RED, "CAL FAILED", "turn more");
    canvas.fillSprite(BLACK);
    canvas.pushSprite(0, 0);
    return;
  }
  offsetX = lround(fit.offset[0]);
  offsetY = lround(fit.offset[1]);
  offsetZ = lround(fit.offset[2]);
  scaleX = fit.scale[0];
  scaleY = fit.scale[1];
  scaleZ = fit.scale[2];
  calHorizontalOnly = fit.horizontalOnly;
  M5.Log.printf("Calibration (%s, %d samples): offsets %d, %d, %d, scales %.3f, %.3f, %.3f\n",
                fit.horizontalOnly ? "horizontal" : "3D", count,
                offsetX, offsetY, offsetZ, scaleX, scaleY, scaleZ);
  showCalibrationResult(DARKGREEN, fit.horizontalOnly ? "CAL 2D OK" : "CAL 3D OK",
                        fit.horizontalOnly ? "Z by incl." : "all axes");

  // --- Accelerometer level step (replaces gyro calibration: the gyro is not used) ---
  // Time to bring the antenna back level; live elevation and roll show when it is there
  unsigned long levelMoveStart = millis();
  while (millis() - levelMoveStart < CALIBRATION_MOVE_TIME_MS) {
    int secondsLeft = (CALIBRATION_MOVE_TIME_MS - (millis() - levelMoveStart) + 999) / 1000;
    canvas.fillSprite(BLUE);
    canvas.setTextDatum(MC_DATUM);
    canvas.setTextSize(2);
    canvas.drawString("Level it", canvas.width() / 2, 16);
    const CalibrationAngle angles[] = {CalibrationAngle::Elevation, CalibrationAngle::Roll};
    for (int i = 0; i < 2; i++) {
      float angle = calibrationAngle(angles[i]);
      if (isnan(angle)) continue;
      String text = i == 0 ? "EL " + String((int)round(angle))
                           : String(angle >= 0 ? "R " : "L ") + String((int)round(fabs(angle)));
      canvas.fillRect(14, 30 + i * 26, canvas.width() - 28, 24, BLACK);
      canvas.setTextColor(fabs(angle) <= CALIBRATION_ANGLE_TOLERANCE_DEG ? GREEN : YELLOW);
      canvas.drawString(text, canvas.width() / 2, 42 + i * 26);
    }
    canvas.setTextColor(WHITE);
    canvas.setTextSize(3);
    canvas.drawString(String(secondsLeft), canvas.width() / 2, 108);
    canvas.pushSprite(0, 0);
    M5.update();
    delay(CALIBRATION_REDRAW_MS);
  }

  canvas.fillSprite(BLUE);
  canvas.setTextDatum(MC_DATUM);
  canvas.setTextSize(2);
  canvas.drawString("Leveling", canvas.width() / 2, 50);
  canvas.drawString("keep still", canvas.width() / 2, 75);
  canvas.pushSprite(0, 0);

  // The magnetometer is averaged too: level and still, it gives the calibration quality
  float sumX = 0, sumY = 0, sumZ = 0;
  int samples = 0;
  double magSum[3] = {0, 0, 0};
  int magSamples = 0;
  unsigned long levelStartTime = millis();
  while (millis() - levelStartTime < LEVEL_CALIBRATION_TIME_MS) {
    float ax, ay, az;
    if (readAccRaw(ax, ay, az)) {
      sumX += ax;
      sumY += ay;
      sumZ += az;
      samples++;
    }
    int rx, ry, rz;
    if (readMagRaw(rx, ry, rz)) {
      magSum[0] += rx;
      magSum[1] += ry;
      magSum[2] += rz;
      magSamples++;
    }
    delay(10);
  }
  if (samples > 0) {
    accOffX = sumX / samples;
    accOffY = sumY / samples;
  }
  M5.Log.printf("Level calibration: acc offsets %.3f, %.3f from %d samples\n", accOffX, accOffY, samples);

  calInclination = NAN;
  calFieldNorm = 0;
  if (samples > 0 && magSamples > 0) {
    const float mag[3] = {(float)((magSum[0] / magSamples - offsetX) * scaleX),
                          (float)((magSum[1] / magSamples - offsetY) * scaleY),
                          (float)((magSum[2] / magSamples - offsetZ) * scaleZ)};
    const float acc[3] = {sumX / samples - accOffX, sumY / samples - accOffY, sumZ / samples};
    calInclination = fieldInclination(mag, acc);
    calFieldNorm = sqrt(mag[0] * mag[0] + mag[1] * mag[1] + mag[2] * mag[2]);
    M5.Log.printf("Calibration quality: inclination %.1f (set %.1f), field %.0f\n",
                  calInclination, magneticInclination, calFieldNorm);
  }
  checkNormRatio = 1.0;
  checkInclinationDiff = 0;
  calSuspect = false;

  calibrated = true;
  saveCalibration();
  filterInitialized = false; // restart the filter with the new calibration

  // DONE with the measured vs set inclination; in 2D the Z offset was derived from the set
  // value, so only a 3D calibration gives an independent check
  canvas.fillSprite(DARKGREEN);
  canvas.setTextDatum(MC_DATUM);
  canvas.setTextSize(3);
  canvas.drawString("DONE", canvas.width() / 2, 28);
  canvas.setTextSize(2);
  if (!isnan(calInclination)) {
    bool good = calHorizontalOnly || fabs(calInclination - magneticInclination) <= CAL_INCLINATION_TOLERANCE_DEG;
    canvas.fillRect(8, 56, canvas.width() - 16, 24, BLACK);
    canvas.setTextColor(good ? GREEN : ORANGE);
    canvas.drawString("I " + String((int)round(calInclination)) + "/" + String((int)round(magneticInclination)),
                      canvas.width() / 2, 68);
    canvas.setTextColor(WHITE);
  }
  canvas.setTextSize(1);
  canvas.drawString(calHorizontalOnly ? "2D: Z from set incl." : "3D: measured/set incl.",
                    canvas.width() / 2, 100);
  canvas.pushSprite(0, 0);
  delay(4000);

  canvas.fillSprite(BLACK);
  canvas.pushSprite(0, 0);
}

