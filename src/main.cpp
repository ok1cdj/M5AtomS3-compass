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

static M5Canvas canvas(&M5.Lcd);
Preferences preferences;
WiFiMulti wifiMulti;

String azimuth;
String elevation;

unsigned long previousMillis = 0UL;
unsigned long interval = 250UL;

int offsetX = 0;
int offsetY = 0;
int offsetZ = 0;

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
  preferences.begin("wifi-config", false);
  String current_creds = preferences.getString("creds", "[]");
  preferences.end();

  JsonDocument doc;
  deserializeJson(doc, current_creds);
  JsonArray array = doc.as<JsonArray>();

  bool updated = false;
  for (JsonObject obj : array) {
    if (String(ssid) == obj["ssid"].as<String>()) {
      obj["password"] = password;
      updated = true;
      break;
    }
  }

  if (!updated) {
    JsonObject new_cred = array.add<JsonObject>();
    new_cred["ssid"] = ssid;
    new_cred["password"] = password;
  }

  String new_creds;
  serializeJson(doc, new_creds);

  preferences.begin("wifi-config", false);
  preferences.putString("creds", new_creds);
  preferences.end();
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
  for (JsonObject obj : array) {
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

void configModeCallback(WiFiManager *myWiFiManager) {
  canvas.fillSprite(BLACK);
  canvas.setTextDatum(MC_DATUM);
  canvas.setTextSize(2);
  canvas.drawString("WiFi Config", canvas.width() / 2, 40);
  canvas.setTextSize(1);
  canvas.drawString("Connect to AP:", canvas.width() / 2, 70);
  canvas.drawString(myWiFiManager->getConfigPortalSSID().c_str(), canvas.width() / 2, 90);
  canvas.pushSprite(0, 0);
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
  int n = loadWiFiCredentials();
  M5.Log.printf("Loaded %d WiFi networks.\n", n);

  if (n > 0) {
    canvas.fillSprite(BLACK);
    canvas.setTextDatum(MC_DATUM);
    canvas.setTextSize(1);
    canvas.drawString("Connecting...", canvas.width() / 2, canvas.height() / 2);
    canvas.pushSprite(0, 0);

    M5.Log.println("Connecting to WiFi with WiFiMulti...");
    uint8_t status = wifiMulti.run(10000); // 10 sekund timeout
    if (status == WL_CONNECTED) {
      M5.Log.printf("WiFi connected to %s\n", WiFi.SSID().c_str());
    } else {
      M5.Log.println("WiFiMulti connection failed. Starting WiFiManager.");
    }
  }
  
  if (WiFi.status() != WL_CONNECTED) {
    WiFiManager wm;
    wm.setAPCallback(configModeCallback);
    if (wm.autoConnect("Compass_Setup")) {
      M5.Log.println("WiFi connected via WiFiManager.");
      saveWiFiCredentials(WiFi.SSID().c_str(), WiFi.psk().c_str());
    } else {
      M5.Log.println("WiFiManager failed to connect.");
      canvas.fillSprite(BLACK);
      canvas.setTextDatum(MC_DATUM);
      canvas.drawString("WiFi FAILED", canvas.width() / 2, canvas.height() / 2);
      canvas.pushSprite(0, 0);
      while(true) { delay(1000); }
    }
  }

  if (!MDNS.begin("compass")) {
    M5.Log.println("Error setting up MDNS responder!");
  } else {
    M5.Log.println("mDNS responder started");
    MDNS.addService("http", "tcp", 80);
  }
  
  canvas.fillSprite(BLACK); // Clear config message
  canvas.pushSprite(0,0);

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
  magneticDeclination = preferences.getInt("decl", 5);
  preferences.end();

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

  server.begin();
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

      // Elevation and roll in radians for tilt compensation
      float roll_rad = atan2(-accX, sqrt(accY * accY + accZ * accZ));
      float elevation_rad = atan2(accY, accZ);

      // Convert to degrees for display/debug
      float elevation = elevation_rad * 180.0 / M_PI;
      float roll = roll_rad * 180.0 / M_PI;

      // Apply calibration offsets
      float cal_mag_x = rawX - offsetX;
      float cal_mag_y = rawY - offsetY;
      float cal_mag_z = rawZ - offsetZ;
      
      // Tilt compensation
      float comp_x = cal_mag_x * cos(elevation_rad) + cal_mag_z * sin(elevation_rad);
      float comp_y = cal_mag_x * sin(roll_rad) * sin(elevation_rad) + cal_mag_y * cos(roll_rad) - cal_mag_z * sin(roll_rad) * cos(elevation_rad);

      // Return Azimuth reading
      float heading = atan2(comp_y, comp_x);
      float declinationAngle = (magneticDeclination * M_PI / 180.0);
      heading += declinationAngle;

      if(heading < 0) heading += 2 * M_PI;
      if(heading > 2 * M_PI) heading -= 2 * M_PI;
      
      a = round(heading * 180 / M_PI);
      
      a = a - 90;
      if (a < 0) a = a + 360;

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
      } else {
          canvas.setTextColor(ORANGE);
          canvas.drawString("WiFi Disconnected", canvas.width() / 2, canvas.height() - 2);
      }
      canvas.setTextColor(WHITE); // Reset text color

      canvas.pushSprite(0, 0);
      String json_data = "{\"azimuth\":" + String(a) + ", \"elev\":" + String((int)elevation) + "}";
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

  canvas.fillSprite(RED);
  canvas.setTextDatum(MC_DATUM);
  canvas.setTextSize(3);
  canvas.drawString("CAL", canvas.width() / 2, 40);
  canvas.setTextSize(2);
  canvas.drawString("Rotate", canvas.width() / 2, 75);
  canvas.drawString("Device", canvas.width() / 2, 95);
  canvas.pushSprite(0, 0);

  unsigned long startTime = millis();
  while (millis() - startTime < 15000) {
    int rawX, rawY, rawZ;
    readRawCompass_on_wire1(&rawX, &rawY, &rawZ);

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
  
  preferences.begin("compass", false);
  preferences.putInt("offX", offsetX);
  preferences.putInt("offY", offsetY);
  preferences.putInt("offZ", offsetZ);
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

