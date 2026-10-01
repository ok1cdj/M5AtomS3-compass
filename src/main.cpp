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
const unsigned long CALIBRATION_TIME_MS = 30000;
const unsigned long LEVEL_CALIBRATION_TIME_MS = 3000;
// Per-axis gain correction (soft-iron / sensor gain mismatch)
float scaleX = 1.0;
float scaleY = 1.0;
float scaleZ = 1.0;
// Accelerometer zero when lying flat
float accOffX = 0.0;
float accOffY = 0.0;

int magneticDeclination = 5;
bool declinationMode = false;
unsigned long lastDeclinationSetTime = 0;

uint32_t pressStartTime = 0;

const int BATTERY_WARN_PERCENT = 25;
const int BATTERY_LOW_PERCENT = 10;

void onWsEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type, void *arg, uint8_t *data, size_t len) {
  if (type == WS_EVT_DATA) {
    AwsFrameInfo *info = (AwsFrameInfo*)arg;
    if (info->final && info->index == 0 && info->len == len && info->opcode == WS_TEXT) {
      data[len] = 0; // Null-terminate
      String message = (char*)data;
      if (message.startsWith("decl:")) {
        int decl = message.substring(5).toInt();
        // Basic validation for declination
        if (decl >= -90 && decl <= 90) { 
          magneticDeclination = decl;
          preferences.begin("compass", false);
          preferences.putInt("decl", magneticDeclination);
          preferences.end();
          M5.Log.printf("Declination set to %d via WebSocket\n", magneticDeclination);
        }
      }
    }
  }
}

void runCalibration(); // Forward declaration

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
  preferences.end();
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
  
  preferences.begin("compass", false);
  magneticDeclination = preferences.getInt("decl", 5);
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

  networkBegin(server);
  server.begin();
}

void loop() {
  int a;
  M5.update();
  batteryUpdate();

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

    // WiFi status bar
    canvas.setTextSize(1);
    canvas.setTextDatum(BC_DATUM); // Bottom-Center datum
    if (WiFi.status() == WL_CONNECTED) {
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

      heading += magneticDeclination * M_PI / 180.0;

      a = (int)round(heading * 180 / M_PI) % 360;
      if (a < 0) a += 360;

      M5.Log.printf("Calculated Azimuth: %d, Elevation: %.1f\n", a, elevation);

      canvas.fillSprite(BLACK);

      // Status bar for calibration
      if (calibrated) {
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

      // WiFi status bar
      canvas.setTextSize(1);
      canvas.setTextDatum(BC_DATUM); // Bottom-Center datum
      if (WiFi.status() == WL_CONNECTED) {
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
               "{\"azimuth\":%d, \"elev\":%d, \"sensor\":\"%s\", \"acc\":[%.3f,%.3f,%.3f], \"raw\":[%d,%d,%d], \"off\":[%d,%d,%d], \"scl\":[%.3f,%.3f,%.3f], \"dip\":%.1f, \"bat\":%s}",
               a, (int)elevation, sensorName(), accX, accY, accZ, lastRaw[0], lastRaw[1], lastRaw[2],
               offsetX, offsetY, offsetZ, scaleX, scaleY, scaleZ, dip, bat_json);
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

void runCalibration() {
  int minX = 32767, maxX = -32767;
  int minY = 32767, maxY = -32767;
  int minZ = 32767, maxZ = -32767;

  // Every axis must point both along and against the field, so the device
  // has to be turned in all directions (incl. upside down), not only flat
  unsigned long startTime = millis();
  int lastSecondsLeft = -1;
  while (millis() - startTime < CALIBRATION_TIME_MS) {
    int secondsLeft = (CALIBRATION_TIME_MS - (millis() - startTime) + 999) / 1000;
    if (secondsLeft != lastSecondsLeft) {
      lastSecondsLeft = secondsLeft;
      canvas.fillSprite(RED);
      canvas.setTextDatum(MC_DATUM);
      canvas.setTextSize(3);
      canvas.drawString("CAL " + String(secondsLeft), canvas.width() / 2, 35);
      canvas.setTextSize(2);
      canvas.drawString("Rotate in", canvas.width() / 2, 75);
      canvas.drawString("all dirs", canvas.width() / 2, 95);
      canvas.pushSprite(0, 0);
    }

    int rawX, rawY, rawZ;
    if (!readMagRawWait(rawX, rawY, rawZ)) continue;

    if (rawX < minX) minX = rawX;
    if (rawX > maxX) maxX = rawX;
    if (rawY < minY) minY = rawY;
    if (rawY > maxY) maxY = rawY;
    if (rawZ < minZ) minZ = rawZ;
    if (rawZ > maxZ) maxZ = rawZ;

    M5.update(); // Keep M5 services running
  }
  
  offsetX = (maxX + minX) / 2;
  offsetY = (maxY + minY) / 2;
  offsetZ = (maxZ + minZ) / 2;

  // Equalize axis gains: scale each axis range to the average range
  float rangeX = maxX - minX, rangeY = maxY - minY, rangeZ = maxZ - minZ;
  float avgRange = (rangeX + rangeY + rangeZ) / 3.0;
  if (rangeX > 100 && rangeY > 100 && rangeZ > 100) {
    scaleX = avgRange / rangeX;
    scaleY = avgRange / rangeY;
    scaleZ = avgRange / rangeZ;
  } else {
    scaleX = scaleY = scaleZ = 1.0; // not enough rotation for gain correction
  }
  M5.Log.printf("Calibration: offsets %d, %d, %d, ranges %.0f, %.0f, %.0f, scales %.3f, %.3f, %.3f\n",
                offsetX, offsetY, offsetZ, rangeX, rangeY, rangeZ, scaleX, scaleY, scaleZ);

  // --- Accelerometer level step (replaces gyro calibration: the gyro is not used) ---
  canvas.fillSprite(BLUE);
  canvas.setTextDatum(MC_DATUM);
  canvas.setTextSize(2);
  canvas.drawString("Place device", canvas.width() / 2, 45);
  canvas.drawString("flat & still", canvas.width() / 2, 75);
  canvas.pushSprite(0, 0);
  delay(3000);

  canvas.fillSprite(BLUE);
  canvas.drawString("Leveling...", canvas.width() / 2, 60);
  canvas.pushSprite(0, 0);

  float sumX = 0, sumY = 0;
  int samples = 0;
  unsigned long levelStartTime = millis();
  while (millis() - levelStartTime < LEVEL_CALIBRATION_TIME_MS) {
    float ax, ay, az;
    if (readAccRaw(ax, ay, az)) {
      sumX += ax;
      sumY += ay;
      samples++;
    }
    delay(10);
  }
  if (samples > 0) {
    accOffX = sumX / samples;
    accOffY = sumY / samples;
  }
  M5.Log.printf("Level calibration: acc offsets %.3f, %.3f from %d samples\n", accOffX, accOffY, samples);

  calibrated = true;
  saveCalibration();
  filterInitialized = false; // restart the filter with the new calibration

  canvas.fillSprite(GREEN);
  canvas.setTextSize(2);
  canvas.setTextDatum(MC_DATUM);
  canvas.drawString("DONE", canvas.width() / 2, canvas.height() / 2);
  canvas.pushSprite(0, 0);
  delay(2000);

  canvas.fillSprite(BLACK);
  canvas.pushSprite(0, 0);
}

