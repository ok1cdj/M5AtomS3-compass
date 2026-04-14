# M5AtomS3 Digital Compass

A digital compass project for the M5AtomS3 using a QMC5883L magnetometer sensor. It displays the magnetic heading (azimuth) in degrees on the built-in LCD.

## Features

- Real-time, centered display of the compass heading.
- On-device magnetometer calibration to compensate for magnetic distortions.
- Calibration data is saved to non-volatile memory and is loaded on startup.
- Status bar that shows whether the device is calibrated.
- Power optimization by disabling WiFi/Bluetooth and reducing CPU frequency.

## Hardware

- **M5AtomS3**
- **QMC5883L Magnetometer Module**

### Wiring

The QMC5883L module should be connected via I2C. The code is configured for the following GPIO pins on the M5AtomS3:

| QMC5883L | M5AtomS3 |
|----------|----------|
| SCL      | GPIO 38  |
| SDA      | GPIO 39  |
| VCC      | 3.3V     |
| GND      | GND      |

*Note: The code initializes the I2C bus with `Wire.begin(38, 39);`.*

## Software & Dependencies

This project is built using the [PlatformIO IDE](https://platformio.org/).

The main dependencies are configured in `platformio.ini`:
- `m5stack/M5Unified`: A comprehensive library for M5Stack devices, including M5GFX for display control.
- `ok1cdj/QMC5883LCompass`: For interfacing with the QMC5883L sensor.

## How to Use

### Building and Uploading

1. Clone this repository.
2. Open the project folder in PlatformIO (e.g., in VSCode with the PlatformIO extension).
3. Build and upload the project to your M5AtomS3.

### Operation

Once powered on, the device will start displaying the current compass heading. The display consists of:
- A status bar at the top showing `CALIBRATED` (green) or `UNCALIBRATED` (red).
- A static red triangular arrow below the status bar, which serves as a fixed pointer.
- The numerical heading value in degrees, centered on the lower half of the screen.

### Calibration

To ensure accurate readings, it's crucial to calibrate the magnetometer. This process compensates for hard-iron distortions from nearby magnetic objects.

1.  **Press and hold the built-in button for 2 seconds** to start the calibration mode. The screen will turn **RED**, and a "CALIBRATING..." message will appear.
2.  For the next **15 seconds**, slowly rotate the device in all directions, making sure to cover all axes (like drawing a figure-eight in the air).
3.  After 15 seconds, the calibration will complete automatically. The screen will turn **GREEN** to indicate success.
4.  The calculated offsets are saved automatically and will be used for all subsequent measurements. The device will then return to normal operation.

You should re-calibrate whenever the device's magnetic environment changes (e.g., if you mount it in a new location).
