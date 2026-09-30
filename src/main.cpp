#include <Arduino.h>
#include <M5Unified.h>
#include <WiFi.h>
#include <WiFiManager.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include "esp32-hal-cpu.h"
#include <Preferences.h>
#include <LittleFS.h>
#include <ESPmDNS.h>
#include <WiFiMulti.h>
#include <ArduinoJson.h>
#include "sensors.h"

AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

static M5Canvas canvas(&M5.Display);
Preferences preferences;
WiFiMulti wifiMulti;

const size_t MAX_WIFI_NETWORKS = 5;
const uint32_t WIFI_PORTAL_TIMEOUT_S = 180;
const uint32_t WIFI_RECONNECT_INTERVAL_MS = 30000;
volatile int wifiNetworkCount = 0;
volatile bool portalActive = false;
bool mdnsStarted = false;
bool serverStarted = false;
WiFiManager wm;

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

void saveWiFiCredentials(const char* ssid, const char* password) {
  // Own Preferences instance: called from the WiFi task while loop() uses the global one
  Preferences prefs;
  prefs.begin("wifi-config", false);
  String current_creds = prefs.getString("creds", "[]");
  prefs.end();

  JsonDocument doc;
  deserializeJson(doc, current_creds);
  JsonArray array = doc.as<JsonArray>();

  if (array.isNull()) {
    array = doc.to<JsonArray>();
  }

  // Keep the list ordered from oldest to newest: move an existing SSID to the end
  for (size_t i = 0; i < array.size(); i++) {
    if (String(ssid) == array[i]["ssid"].as<String>()) {
      array.remove(i);
      break;
    }
  }

  JsonObject new_cred = array.add<JsonObject>();
  new_cred["ssid"] = ssid;
  new_cred["password"] = password;

  // Drop the oldest networks over the limit
  while (array.size() > MAX_WIFI_NETWORKS) {
    array.remove(0);
  }

  String new_creds;
  serializeJson(doc, new_creds);

  prefs.begin("wifi-config", false);
  prefs.putString("creds", new_creds);
  prefs.end();
  M5.Log.printf("Saved new WiFi credentials for %s\n", ssid);
}

int loadWiFiCredentials() {
  preferences.begin("wifi-config", true); // read-only
  String current_creds = preferences.getString("creds", "[]");
  preferences.end();

  JsonDocument doc;
  DeserializationError error = deserializeJson(doc, current_creds);
  if (error) {
    M5.Log.printf("Failed to parse wifi creds: %s\n", error.c_str());
    return 0;
  }

  JsonArray array = doc.as<JsonArray>();
  int count = 0;
  // Load only the newest MAX_WIFI_NETWORKS entries
  size_t skip = array.size() > MAX_WIFI_NETWORKS ? array.size() - MAX_WIFI_NETWORKS : 0;
  for (JsonObject obj : array) {
    if (skip > 0) {
      skip--;
      continue;
    }
    const char* ssid = obj["ssid"];
    const char* password = obj["password"];
    if (ssid && password) {
      wifiMulti.addAP(ssid, password);
      M5.Log.printf("Loaded WiFi network: %s\n", ssid);
      count++;
    }
  }
  return count;
}

void startMDNS() {
  if (mdnsStarted) return;
  if (!MDNS.begin("compass")) {
    M5.Log.println("Error setting up MDNS responder!");
  } else {
    M5.Log.println("mDNS responder started");
    MDNS.addService("http", "tcp", 80);
    mdnsStarted = true;
  }
}

void onWiFiConnected() {
  startMDNS();
  if (!serverStarted) {
    // Started only after the config portal is closed: both use port 80
    server.begin();
    serverStarted = true;
    M5.Log.println("Web server started");
  }
}

// All WiFi work runs here so the compass starts immediately: tries saved networks,
// once falls back to the config portal in the background, reconnects on drops
void wifiTask(void *param) {
  bool portalTried = false;
  bool firstAttempt = true;
  uint32_t lastAttempt = 0;

  for (;;) {
    if (portalActive) {
      if (wm.process()) {
        M5.Log.printf("WiFi connected via portal to %s\n", WiFi.SSID().c_str());
        saveWiFiCredentials(WiFi.SSID().c_str(), WiFi.psk().c_str());
        wifiMulti.addAP(WiFi.SSID().c_str(), WiFi.psk().c_str());
        wifiNetworkCount = wifiNetworkCount + 1;
      }
      if (!wm.getConfigPortalActive()) {
        portalActive = false;
        if (WiFi.status() != WL_CONNECTED) {
          M5.Log.println("Config portal closed, running offline.");
          WiFi.mode(WIFI_STA);
        }
      }
      vTaskDelay(pdMS_TO_TICKS(10));
      continue;
    }

    if (WiFi.status() == WL_CONNECTED) {
      onWiFiConnected();
    } else if (firstAttempt || millis() - lastAttempt >= WIFI_RECONNECT_INTERVAL_MS) {
      firstAttempt = false;
      lastAttempt = millis();
      if (wifiNetworkCount > 0) {
        M5.Log.println("Trying saved WiFi networks...");
        wifiMulti.run(8000);
      }
      if (WiFi.status() == WL_CONNECTED) {
        M5.Log.printf("WiFi connected to %s\n", WiFi.SSID().c_str());
        onWiFiConnected();
      } else if (!portalTried && !serverStarted) {
        portalTried = true;
        M5.Log.println("Starting config portal Compass_Setup in background.");
        wm.setConfigPortalBlocking(false);
        wm.setConfigPortalTimeout(WIFI_PORTAL_TIMEOUT_S);
        wm.startConfigPortal("Compass_Setup");
        portalActive = true;
      }
    }
    vTaskDelay(pdMS_TO_TICKS(500));
  }
}

void runCalibration(); // Forward declaration

String calKey(const char* name) {
  return String(sensorKeyPrefix()) + name;
}

void loadCalibration() {
  preferences.begin("compass", true);
  calibrated = preferences.getBool(calKey("cal").c_str(), false);
  offsetX = preferences.getInt(calKey("offX").c_str(), 0);
  offsetY = preferences.getInt(calKey("offY").c_str(), 0);
  offsetZ = preferences.getInt(calKey("offZ").c_str(), 0);
  scaleX = preferences.getFloat(calKey("sclX").c_str(), 1.0);
  scaleY = preferences.getFloat(calKey("sclY").c_str(), 1.0);
  scaleZ = preferences.getFloat(calKey("sclZ").c_str(), 1.0);
  accOffX = preferences.getFloat(calKey("accX").c_str(), 0.0);
  accOffY = preferences.getFloat(calKey("accY").c_str(), 0.0);
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
  
  M5.Log.println("Loading WiFi credentials...");
  wifiNetworkCount = loadWiFiCredentials();
  M5.Log.printf("Loaded %d WiFi networks.\n", wifiNetworkCount);

  if(!LittleFS.begin(true)){
    canvas.setTextDatum(MC_DATUM);
    canvas.drawString("LittleFS Error", canvas.width() / 2, canvas.height() / 2);
    canvas.pushSprite(0,0);
    return;
  }

  preferences.begin("compass", false);
  magneticDeclination = preferences.getInt("decl", 5);
  preferences.end();

  sensorsBegin();
  loadCalibration();
  M5.Lcd.setRotation(0);

  ws.onEvent(onWsEvent);
  server.addHandler(&ws);

  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(LittleFS, "/index.html", "text/html");
  });

  server.on("/settings", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(LittleFS, "/settings.html", "text/html");
  });

  xTaskCreatePinnedToCore(wifiTask, "wifi", 8192, NULL, 1, NULL, 0);
}

void loop() {
  int a;
  M5.update();

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
    } else if (portalActive) {
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
      } else if (portalActive) {
          canvas.setTextColor(YELLOW);
          canvas.drawString("AP: Compass_Setup", canvas.width() / 2, canvas.height() - 2);
      } else {
          canvas.setTextColor(ORANGE);
          canvas.drawString("WiFi Disconnected", canvas.width() / 2, canvas.height() - 2);
      }
      canvas.setTextColor(WHITE); // Reset text color

      canvas.pushSprite(0, 0);
      // Raw sensor data is included for remote diagnostics of axis mapping and calibration
      char json_data[320];
      snprintf(json_data, sizeof(json_data),
               "{\"azimuth\":%d, \"elev\":%d, \"sensor\":\"%s\", \"acc\":[%.3f,%.3f,%.3f], \"raw\":[%d,%d,%d], \"off\":[%d,%d,%d], \"scl\":[%.3f,%.3f,%.3f], \"dip\":%.1f}",
               a, (int)elevation, sensorName(), accX, accY, accZ, lastRaw[0], lastRaw[1], lastRaw[2],
               offsetX, offsetY, offsetZ, scaleX, scaleY, scaleZ, dip);
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

