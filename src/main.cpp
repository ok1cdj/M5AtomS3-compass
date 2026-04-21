#include <Arduino.h>
#include <M5Unified.h>
#define Wire Wire1
#include <QMC5883LCompass.h>
#include <WiFi.h>
#include <WiFiManager.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include "esp32-hal-cpu.h"
#include <Preferences.h>
#include <LittleFS.h>

AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

static M5Canvas canvas(&M5.Lcd);
QMC5883LCompass compass;
Preferences preferences;

String azimuth;
String elevation;

unsigned long previousMillis = 0UL;
unsigned long interval = 250UL;

const int MAGNETOMETER_STEPS = 10;
const bool MAGNETOMETER_ADVANCED_SMOOTHING = true;
const int MS_BETWEEN_SAMPLES = 100;

int offsetX = 0;
int offsetY = 0;
int offsetZ = 0;

int magneticDeclination = 5;
bool declinationMode = false;
unsigned long lastDeclinationSetTime = 0;

uint32_t pressStartTime = 0;

void onWsEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type, void *arg, uint8_t *data, size_t len) {
  // We don't need to handle any events from the client side for this project
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

void setup() {
  btStop();
  setCpuFrequencyMhz(80); //Set CPU clock to 80MHz fo example
  M5.begin();
  M5.Imu.loadOffsetFromNVS();
  canvas.createSprite(M5.Lcd.width(), M5.Lcd.height());
  
  WiFiManager wm;
  wm.setAPCallback(configModeCallback);
  wm.autoConnect("Compass_Setup");
  
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
  compass.init();
  compass.setSmoothing(MAGNETOMETER_STEPS, MAGNETOMETER_ADVANCED_SMOOTHING);
  M5.Lcd.setRotation(0);

  ws.onEvent(onWsEvent);
  server.addHandler(&ws);

  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(LittleFS, "/index.html", "text/html");
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
      
      compass.read();
      int rawX = compass.getX();
      int rawY = compass.getY();
      int rawZ = compass.getZ();

      float accX, accY, accZ;
      M5.Imu.getAccelData(&accX, &accY, &accZ);
      float pitch = atan2(-accY, accZ) * 180.0 / PI;
      float roll = atan2(accX, accZ) * 180.0 / PI;

      // Return Azimuth reading

      float heading = atan2(rawY - offsetY, rawX - offsetX);
      float declinationAngle = (magneticDeclination * M_PI / 180.0);
      heading += declinationAngle;

      if(heading < 0) heading += 2 * M_PI;
      if(heading > 2 * M_PI) heading -= 2 * M_PI;
      
      a = round(heading * 180 / M_PI);
      
	  previousMillis = currentMillis;
      a = a - 90;
      if (a < 0) a = a + 360;

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

      // Pitch & Roll values
      canvas.setTextSize(1);

      String pitch_str = "Pitch: " + String((int)pitch);
      canvas.drawString(pitch_str, centerX, 105);
      int pitchTextWidth = canvas.textWidth(pitch_str);
      canvas.drawCircle(centerX + pitchTextWidth/2 + 3, 105 - 3, 1, WHITE);

      String roll_str = "Roll: " + String((int)roll);
      canvas.drawString(roll_str, centerX, 115);
      int rollTextWidth = canvas.textWidth(roll_str);
      canvas.drawCircle(centerX + rollTextWidth/2 + 3, 115 - 3, 1, WHITE);

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
      String json_data = "{\"azimuth\":" + String(a) + ", \"pitch\":" + String((int)pitch) + ", \"roll\":" + String((int)roll) + "}";
      ws.textAll(json_data);
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
    compass.read();
    int rawX = compass.getX();
    int rawY = compass->getY();
    int rawZ = compass->getZ();

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

