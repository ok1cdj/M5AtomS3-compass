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

int offsetX = 0;
int offsetY = 0;
int offsetZ = 0;
const unsigned long CALIBRATION_TIME_MS = 30000;
// Per-axis gain correction (soft-iron / sensor gain mismatch)
float scaleX = 1.0;
float scaleY = 1.0;
float scaleZ = 1.0;

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

void compass_init_on_wire1() {
  M5.Log.println("Initializing compass...");
  // Soft reset the sensor
  Wire1.beginTransmission(0x0D);
  Wire1.write(0x0B); // QMC5883L_REG_CONTROL_2
  Wire1.write(0x01); // Set the soft reset bit
  byte error = Wire1.endTransmission();
  M5.Log.printf("Compass soft reset, endTransmission status: %d\n", error);
  delay(10);

  // Configure the sensor for continuous measurement
  Wire1.beginTransmission(0x0D);
  Wire1.write(0x09); // QMC5883L_REG_CONTROL_1
  Wire1.write(0x1D); // ODR=200Hz, RNG=8G, OSR=64, Mode=Continuous
  error = Wire1.endTransmission();
  M5.Log.printf("Compass config, endTransmission status: %d\n", error);
  delay(10);
}

void readRawCompass_on_wire1(int* x, int* y, int* z) {
  // Re-assert continuous measurement mode. Some QMC5883L clones may need this to get fresh data.
  Wire1.beginTransmission(0x0D);
  Wire1.write(0x09); // QMC5883L_REG_CONTROL_1
  Wire1.write(0x1D); // ODR=200Hz, RNG=8G, OSR=512, Mode=Continuous
  Wire1.endTransmission();
  delay(10); // Give it time to take a measurement

  Wire1.beginTransmission(0x0D);
  Wire1.write(0x00); // Start reading from register 0
  Wire1.endTransmission();

  int bytes_received = Wire1.requestFrom(0x0D, 6);
  if (bytes_received >= 6) {
    *x = (int16_t)(Wire1.read() | (Wire1.read() << 8));
    *y = (int16_t)(Wire1.read() | (Wire1.read() << 8));
    *z = (int16_t)(Wire1.read() | (Wire1.read() << 8));
  } else {
    M5.Log.printf("Compass read failed. Bytes received: %d\n", bytes_received);
    *x = *y = *z = 0; // Return 0 if read failed
  }
}

void setup() {
  btStop();
  setCpuFrequencyMhz(80); //Set CPU clock to 80MHz fo example
  M5.begin();
  M5.Log.println("Starting setup...");
  M5.Imu.loadOffsetFromNVS();
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
  offsetX = preferences.getInt("offX", 0);
  offsetY = preferences.getInt("offY", 0);
  offsetZ = preferences.getInt("offZ", 0);
  scaleX = preferences.getFloat("sclX", 1.0);
  scaleY = preferences.getFloat("sclY", 1.0);
  scaleZ = preferences.getFloat("sclZ", 1.0);
  magneticDeclination = preferences.getInt("decl", 5);
  preferences.end();
  M5.Log.printf("Compass offsets: X=%d, Y=%d, Z=%d, scales: %.3f, %.3f, %.3f\n", offsetX, offsetY, offsetZ, scaleX, scaleY, scaleZ);

  Wire1.begin(38, 39);
  compass_init_on_wire1();
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
    // Read compass values
    unsigned long currentMillis = millis();
    if (currentMillis - previousMillis > interval)
    {
      M5.Imu.update();
      
      int rawX, rawY, rawZ;
      readRawCompass_on_wire1(&rawX, &rawY, &rawZ);
      M5.Log.printf("Compass Raw: X=%d, Y=%d, Z=%d\n", rawX, rawY, rawZ);

      float accX, accY, accZ;
      M5.Imu.getAccelData(&accX, &accY, &accZ);

      // Elevation (tilt of Y axis, the arrow direction) for display
      float elevation = atan2(accY, sqrt(accX * accX + accZ * accZ)) * 180.0 / M_PI;

      // Calibrated field; magnetometer axes match the IMU axes (verified on recorded data)
      float mx = (rawX - offsetX) * scaleX;
      float my = (rawY - offsetY) * scaleY;
      float mz = (rawZ - offsetZ) * scaleZ;

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
      M5.Log.printf("Diag: acc=%.2f,%.2f,%.2f mag=%.0f,%.0f,%.0f |m|=%.0f dip=%.1f\n",
                    accX, accY, accZ, mx, my, mz, mag_norm, dip);

      heading += magneticDeclination * M_PI / 180.0;

      a = (int)round(heading * 180 / M_PI) % 360;
      if (a < 0) a += 360;

      M5.Log.printf("Calculated Azimuth: %d, Elevation: %.1f\n", a, elevation);

      canvas.fillSprite(BLACK);

      // Status bar for calibration
      if (offsetX != 0 || offsetY != 0 || offsetZ != 0) {
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
               "{\"azimuth\":%d, \"elev\":%d, \"acc\":[%.3f,%.3f,%.3f], \"raw\":[%d,%d,%d], \"off\":[%d,%d,%d], \"scl\":[%.3f,%.3f,%.3f], \"dip\":%.1f}",
               a, (int)elevation, accX, accY, accZ, rawX, rawY, rawZ, offsetX, offsetY, offsetZ, scaleX, scaleY, scaleZ, dip);
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
        } else if (pressed_ms >= 2000) {
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
    readRawCompass_on_wire1(&rawX, &rawY, &rawZ);
    if (rawX == 0 && rawY == 0 && rawZ == 0) continue; // read failed

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

  preferences.begin("compass", false);
  preferences.putInt("offX", offsetX);
  preferences.putInt("offY", offsetY);
  preferences.putInt("offZ", offsetZ);
  preferences.putFloat("sclX", scaleX);
  preferences.putFloat("sclY", scaleY);
  preferences.putFloat("sclZ", scaleZ);
  preferences.end();
  
  // --- IMU Gyro Calibration Step ---
  canvas.fillSprite(BLUE);
  canvas.setTextDatum(MC_DATUM);
  canvas.setTextSize(2);
  canvas.drawString("Place device", canvas.width() / 2, 45);
  canvas.drawString("still & flat", canvas.width() / 2, 75);
  canvas.pushSprite(0, 0);
  delay(3000);

  canvas.fillSprite(BLUE);
  canvas.drawString("Calibrating", canvas.width() / 2, 45);
  canvas.drawString("Gyro...", canvas.width() / 2, 75);
  canvas.pushSprite(0, 0);

  // Start gyro calibration using M5Unified's built-in function
  M5.Imu.setCalibration(0, 64, 0); // Calibrate gyro only, strength 64

  unsigned long calibStartTime = millis();
  while (millis() - calibStartTime < 5000) { // Calibrate for 5 seconds
    M5.Imu.update(); // The library performs calibration during update
    delay(1);
  }

  // Stop calibration and save the results to NVS (Non-Volatile Storage)
  M5.Imu.setCalibration(0, 0, 0);
  M5.Imu.saveOffsetToNVS();

  canvas.fillSprite(GREEN);
  canvas.setTextSize(2);
  canvas.setTextDatum(MC_DATUM);
  canvas.drawString("DONE", canvas.width() / 2, canvas.height() / 2);
  canvas.pushSprite(0, 0);
  delay(2000);

  canvas.fillSprite(BLACK);
  canvas.pushSprite(0, 0);
}

