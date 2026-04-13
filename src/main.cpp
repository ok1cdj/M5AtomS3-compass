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

int minX,maxX,minY,maxY, minZ,maxZ;


void setup() {
  WiFi.mode(WIFI_OFF);
  btStop();
  setCpuFrequencyMhz(80); //Set CPU clock to 80MHz fo example
  M5.begin(true, true, true, false);  // Init AtomS3(Initialize LCD, serial port).
  preferences.begin("compass_calibration", false);
  minX = preferences.getInt("minX", 0);
  minY = preferences.getInt("minY", 0);
  minZ = preferences.getInt("minZ", 0);
  maxX = preferences.getInt("maxX", 0);
  maxY = preferences.getInt("maxY", 0);
  maxZ = preferences.getInt("maxZ", 0);
  compass.setCalibration(minX, minY, minZ, maxX, maxY, maxZ);

  Wire.begin(38, 39);
  compass.init();
  compass.setMagneticDeclination(5, 15);
  compass.setSmoothing(MAGNETOMETER_STEPS, MAGNETOMETER_ADVANCED_SMOOTHING);
  M5.Lcd.setRotation(2);
  M5.Lcd.println("TEST");


}

void loop() {
  int a;
  M5.update();
  // Read compass values
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis > interval)
  {
    compass.read();
    // Return Azimuth reading
    a = compass.getAzimuth() - 90;
	    previousMillis = currentMillis;
    if (a < 0) a = a + 360;
    USBSerial.print("A: ");
    USBSerial.print(a);
    USBSerial.println();
    M5.Lcd.setTextSize(5);
    M5.Lcd.fillRect(10, 15, 100, 40,
                    BLACK);
    M5.Lcd.setCursor(10, 15);
    if (a < 100)  M5.Lcd.print(" ");
    M5.Lcd.println(a);

if (M5.Btn.wasReleased() || M5.Btn.pressedFor(1000)) {
        USBSerial.print('A');
        M5.Lcd.print("A");
    }

  }
}

void calibration() {
	 Serial.println("This will provide calibration settings for your QMC5883L chip. When prompted, move the magnetometer in all directions until the calibration is complete.");
  Serial.println("Calibration will begin in 5 seconds.");
  delay(5000);

  Serial.println("CALIBRATING. Keep moving your sensor...");
  compass.calibrate();

  Serial.println("DONE. Copy the lines below and paste it into your projects sketch.);");
  minX = compass.getCalibrationOffset(0);
  minY = compass.getCalibrationOffset(1);
  minZ = compass.getCalibrationOffset(2);
  maxX = compass.getCalibrationScale(0);
  maxY = compass.getCalibrationScale(1);
  maxZ = compass.getCalibrationScale(2);
  preferences.putInt("minX", minX);
  preferences.putInt("minY", minY);
  preferences.putInt("minZ", minZ);
  preferences.putInt("maxX", maxX);
  preferences.putInt("maxY", maxY);
  preferences.putInt("maxZ", maxZ);
	
}

