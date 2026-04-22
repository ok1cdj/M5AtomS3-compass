# M5AtomS3 Digital Compass

A digital compass project for the M5AtomS3 using a QMC5883L magnetometer sensor. It displays the magnetic heading (azimuth) and elevation on the built-in LCD. The compass also features a web interface to view the data and configure settings.

## Features

- Real-time display of compass heading (azimuth) and elevation.
- Web interface accessible at `http://compass.local` for remote viewing and settings.
- On-device calibration for both magnetometer (hard-iron) and gyroscope (drift).
- Calibration data is saved to non-volatile memory and loaded on startup.
- Magnetic declination can be configured on the device or via the web interface.
- Status bars for calibration and WiFi status.
- Support for multiple WiFi networks; the device remembers previously used networks and automatically connects.
- Power optimization by reducing CPU frequency and disabling Bluetooth.

## Hardware

- **M5AtomS3**
- **QMC5883L Magnetometer Module**

### 3D Printed Mount

The repository includes a 3D model for a pipe mount designed for an M5Stack Atomic Proto Kit.

- **File:** `3D/atomic_proto_pipe_mount.scad`
- **Description:** A holder to mount the device on a 25mm diameter pipe. It uses M3 screws (16mm spacing) and zip ties. The design is in OpenSCAD.

### Wiring

The QMC5883L module should be connected via I2C. The code is configured for the following GPIO pins on the M5AtomS3:

| QMC5883L | M5AtomS3 |
|----------|----------|
| SCL      | GPIO 38  |
| SDA      | GPIO 39  |
| VCC      | 3.3V     |
| GND      | GND      |

*Note: The code initializes the I2C bus with `Wire1.begin(38, 39);`.*

## Software & Dependencies

This project is built using the [PlatformIO IDE](https://platformio.org/).

The main dependencies are configured in `platformio.ini`:
- `m5stack/M5Unified`: A comprehensive library for M5Stack devices, including M5GFX for display control.
- `tzapu/WiFiManager`: Used for initial WiFi configuration via a captive portal.
- `ESPAsyncWebServer` & `AsyncTCP`: For the web interface and WebSocket communication.
- `bblanchon/ArduinoJson`: For storing multiple WiFi credentials.

## How to Use

### Building and Uploading

1. Clone this repository.
2. Open the project folder in PlatformIO (e.g., in VSCode with the PlatformIO extension).
3. Build and upload the project to your M5AtomS3.

### Operation

Once powered on and connected to WiFi, the device starts displaying the current compass heading. The display consists of:
- A top status bar showing `CALIBRATED` (green) with the current magnetic declination, or `UNCALIBRATED` (red).
- A static red triangular arrow below the status bar, which serves as a fixed pointer.
- The numerical heading value in degrees, centered on the middle of the screen.
- Below the heading, the current elevation (`Elev`) is displayed.
- A bottom status bar showing the WiFi connection status and IP address (cyan) or a disconnected message (orange).

### WiFi Connection

The device supports multiple WiFi networks.
- On the first boot, or if no known network is in range, it will start a WiFi Access Point named `Compass_Setup`.
- Connect to this network with your phone or computer. A captive portal should open automatically, where you can select your WiFi network and enter the password.
- After connecting, the credentials are saved. The device will automatically try to connect to all saved networks on subsequent startups.

### Web Interface

The device is accessible on your local network via the address `http://compass.local`. The web interface provides:
- A large, real-time display of the azimuth and elevation.
- A settings page (accessible via the gear icon `⚙️`) to configure the magnetic declination.

### Calibration

To ensure accurate readings, it's crucial to calibrate both the magnetometer and the gyroscope. This process has two stages.

1.  **Press and hold the built-in button for 2 to 5 seconds, then release** to start the calibration mode.

**Stage 1: Magnetometer Calibration**
1.  The screen will turn **RED**. For the next **15 seconds**, slowly rotate the device in all directions, making sure to cover all axes (like drawing a figure-eight in the air). This compensates for magnetic distortions.

**Stage 2: Gyroscope Calibration**
1.  After the magnetometer calibration, the screen will turn **BLUE** and prompt you to place the device on a still, flat surface.
2.  Place the device down and wait. The device will automatically calibrate the gyroscope to remove any drift.
3.  Once both stages are complete, the screen will turn **GREEN** to indicate success. The calculated offsets are saved, and the device will return to normal operation.

You should re-calibrate whenever the device's magnetic environment changes (e.g., if you mount it in a new location).

### Magnetic Declination Setting

Magnetic declination is the angle between magnetic north and true north. For accurate headings, you must set this for your location. There are two ways to set it:

#### On-Device Method
1.  **Press and hold the built-in button for 5 seconds or more, then release** to enter declination setting mode.
2.  The screen will show the current declination value.
3.  **Short-press the button** to increment the value. It cycles from -10 to +10 degrees.
4.  Once you have set the desired value, **wait for 2 seconds without pressing the button**. The value will be saved, and the device will return to normal operation.

#### Web Interface Method
1.  Open `http://compass.local` in your browser.
2.  Click the gear icon (`⚙️`) to navigate to the settings page.
3.  Enter your local declination value and click "Set". The value is saved immediately.
