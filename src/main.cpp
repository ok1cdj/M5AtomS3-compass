#include <Arduino.h>
#include "M5AtomS3.h"
#include <QMC5883LCompass.h>
#include <WiFi.h>
#include "esp32-hal-cpu.h"
#include <Preferences.h>

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

void runCalibration(); // Forward declaration

void readRawCompass(int* x, int* y, int* z) {
  Wire.beginTransmission(0x0D);
  Wire.write(0x00);
  Wire.endTransmission();

  Wire.requestFrom(0x0D, 6);
  if (Wire.available() >= 6) {
    *x = (int16_t)(Wire.read() | (Wire.read() << 8));
    *y = (int16_t)(Wire.read() | (Wire.read() << 8));
    *z = (int16_t)(Wire.read() | (Wire.read() << 8));
  }
}

void setup() {
  WiFi.mode(WIFI_OFF);
  btStop();
  setCpuFrequencyMhz(80); //Set CPU clock to 80MHz fo example
  M5.begin(true, true, true, false);  // Init AtomS3(Initialize LCD, serial port).
  preferences.begin("compass", false);
  offsetX = preferences.getInt("offX", 0);
  offsetY = preferences.getInt("offY", 0);
  offsetZ = preferences.getInt("offZ", 0);
  preferences.end();

  Wire.begin(38, 39);
  compass.init();
  compass.setMagneticDeclination(5, 15);
  compass.setSmoothing(MAGNETOMETER_STEPS, MAGNETOMETER_ADVANCED_SMOOTHING);
  M5.Lcd.setRotation(2);


}

void loop() {
  int a;
  M5.update();
  // Read compass values
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis > interval)
  {
    int rawX, rawY, rawZ;
    readRawCompass(&rawX, &rawY, &rawZ);
    // Return Azimuth reading

    float heading = atan2(rawY - offsetY, rawX - offsetX);
    float declinationAngle = (5.0 + (15.0 / 60.0)) * M_PI / 180.0;
    heading += declinationAngle;

    if(heading < 0) heading += 2 * M_PI;
    if(heading > 2 * M_PI) heading -= 2 * M_PI;
    
    a = round(heading * 180 / M_PI);
    
	  previousMillis = currentMillis;
    a = a - 90;
    if (a < 0) a = a + 360;

    M5.Lcd.setTextSize(5);
    M5.Lcd.fillRect(10, 15, 100, 40,
                    BLACK);
    M5.Lcd.setCursor(10, 15);
    if (a < 100)  M5.Lcd.print(" ");
    M5.Lcd.println(a);
  }

  if (M5.Btn.pressedFor(2000)) {
      runCalibration();
  }
}

void runCalibration() {
  int minX = 32767, maxX = -32767;
  int minY = 32767, maxY = -32767;
  int minZ = 32767, maxZ = -32767;

  M5.Lcd.fillScreen(RED);
  M5.Lcd.setTextSize(2);
  M5.Lcd.setCursor(5, 25);
  M5.Lcd.println("CALIBRATING...");

  unsigned long startTime = millis();
  while (millis() - startTime < 15000) {
    int rawX, rawY, rawZ;
    readRawCompass(&rawX, &rawY, &rawZ);

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
  
  M5.Lcd.fillScreen(GREEN);
  M5.Lcd.setTextSize(2);
  M5.Lcd.setCursor(35, 25);
  M5.Lcd.println("DONE");
  delay(2000);

  M5.Lcd.fillScreen(BLACK);
}

