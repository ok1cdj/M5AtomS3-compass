# M5AtomS3 Digital Compass

A digital compass project for the M5AtomS3 using an external QMC5883L magnetometer or a GY-511 (LSM303DLHC) accelerometer + magnetometer module. It displays the magnetic heading (azimuth) and elevation on the built-in LCD. The compass also features a web interface to view the data and configure settings.

## Features

- Real-time display of compass heading (azimuth) and elevation.
- Web interface accessible at `http://compass.local` for remote viewing and settings.
- Automatic detection of the connected sensor (GY-511 / LSM303DLHC or QMC5883L).
- On-device calibration of the magnetometer (hard-iron offsets and per-axis gain) and of the accelerometer level; each sensor keeps its own calibration.
- Tilt-compensated heading, accurate to about ±10° for tilts up to 45°.
- Calibration data is saved to non-volatile memory and loaded on startup.
- Magnetic declination can be configured on the device or via the web interface.
- Status bars for calibration and WiFi status.
- Support for multiple WiFi networks; the device remembers up to 5 previously used networks and automatically connects (and reconnects if the connection drops).
- Works offline: if no WiFi is available, the compass keeps running without the web interface.
- Battery level on the display and in the web interface when powered by the Atomic Battery Base (voltage measured on G8); hidden with other power sources such as TailBat or USB.
- Power optimization by reducing CPU frequency and disabling Bluetooth.

## Hardware

- **M5AtomS3**
- One of the sensor modules (detected automatically at startup):
  - **GY-511 (LSM303DLHC)**: accelerometer and magnetometer in one chip. The AtomS3 internal IMU is not used, so there is no misalignment between the two sensors. Preferred when both modules are connected.
  - **QMC5883L magnetometer module**: used together with the AtomS3 internal accelerometer.

If no sensor is found, the display shows `NO SENSOR`; WiFi and the web interface keep running.

### Power

- **Atomic Battery Base** (200 mAh): the battery voltage is measured on G8 through a 1:2 divider. The display shows the charge level in the top right corner (orange at 25 % and below, red at 10 % and below); the web page shows the level and the voltage.
- **TailBat** or USB: no battery measurement; the battery indicator is hidden. The battery is detected automatically from a plausible and stable voltage on G8.

### 3D Printed Mount

The repository includes a 3D model for a pipe mount designed for an M5Stack Atomic Proto Kit.

- **File:** `3D/atomic_proto_pipe_mount.scad`
- **Description:** A holder to mount the device on a 25mm diameter pipe. It uses M3 screws (16mm spacing) and zip ties. The design is in OpenSCAD.

### Wiring

**GY-511 (LSM303DLHC)** is connected to the Grove port (I2C, `Wire`):

| GY-511 | M5AtomS3 Grove |
|--------|----------------|
| SDA    | G2             |
| SCL    | G1             |
| VIN    | 5V (the module has its own regulator) |
| GND    | GND            |

*The ESP32-S3 pins are not 5V tolerant. The Grove port supplies 5V: check that the I2C pull-up resistors on the GY-511 go to its 3.3V regulator output, not to VIN. If unsure, power the module from 3.3V.*

**QMC5883L** is connected to the header pins of the internal I2C bus (`Wire1`, shared with the internal IMU):

| QMC5883L | M5AtomS3 |
|----------|----------|
| SDA      | GPIO 38  |
| SCL      | GPIO 39  |
| VCC      | 3.3V     |
| GND      | GND      |

Mount the module so that its axes are aligned with the device: the heading is measured in the direction of the red arrow on the display.

## Software & Dependencies

This project is built using the [PlatformIO IDE](https://platformio.org/).

The main dependencies are configured in `platformio.ini`:
- `m5stack/M5Unified`: A comprehensive library for M5Stack devices, including M5GFX for display control.
- `ESPAsyncWebServer` & `AsyncTCP`: For the web interface and WebSocket communication.
- `bblanchon/ArduinoJson`: For storing multiple WiFi credentials.

## How to Use

### Web Installer

The easiest way: open **https://ok1cdj.github.io/M5AtomS3-compass/** in Chrome, Edge or Brave, connect the AtomS3 with a USB-C cable and click *Connect device*. The page flashes the latest released firmware; no IDE or drivers are needed. When updating, do not erase the device, otherwise the saved WiFi networks and the calibration are lost.

Releases are built by GitHub Actions: pushing a tag `v*` builds the firmware and attaches it to the release, then the install page is redeployed with it (see `docs/NOTES-for-ci.md`).

### Building and Uploading

1. Clone this repository.
2. Open the project folder in PlatformIO (e.g., in VSCode with the PlatformIO extension).
3. Build and upload the project to your M5AtomS3 (`pio run -t upload`).

The web pages in `data/` are embedded into the firmware at build time (gzip-compressed by `scripts/embed_web.py`), so there is no separate filesystem upload. Edit them as normal HTML files and rebuild.

### Operation

Once powered on and connected to WiFi, the device starts displaying the current compass heading. The display consists of:
- A top status bar showing `CALIBRATED` (green) with the current magnetic declination, or `UNCALIBRATED` (red).
- A static red triangular arrow below the status bar, which serves as a fixed pointer.
- The numerical heading value in degrees, centered on the middle of the screen.
- Below the heading, the current elevation (`Elev`) is displayed.
- A bottom status bar showing the WiFi connection status and IP address (cyan) or a disconnected message (orange).

### WiFi Connection

The device supports multiple WiFi networks. All WiFi handling runs in the background, so the compass works immediately after power-on, with or without WiFi.
- At startup the device tries all saved networks.
- If none of them is available (or none is saved), it starts a WiFi access point named `Compass_Setup`; the bottom status bar shows `AP: Compass_Setup`. Connect to it with your phone or computer: a captive portal opens the WiFi setup page (or open `http://192.168.4.1/wifi`).
- On the setup page you can scan for networks, add a network, and see and delete the saved networks. Up to 5 networks are remembered; when a 6th is added, the oldest one is dropped.
- After a successful connection the access point closes after 10 seconds. If nothing is configured within 5 minutes, it closes and the compass runs offline.
- If the connection drops while running, the device tries all saved networks again every 30 seconds.
- **WiFi off:** hold the button under the display while powering on or pressing the reset button on the side, until `WiFi OFF` appears. WiFi stays off until the next reset, which saves battery when no network is needed (e.g. in the field). The bottom status bar shows `WiFi OFF`.
- While connected, the setup page is available at `http://compass.local/wifi` (or via the settings page).

### Web Interface

The device is accessible on your local network via the address `http://compass.local`. The web interface provides:
- A large, real-time display of the azimuth and elevation.
- A settings page (accessible via the gear icon `⚙️`) to configure the magnetic declination.

### Calibration

To ensure accurate readings, calibrate the magnetometer and the accelerometer level. This process has two stages. The calibration is stored separately for each sensor module; a newly connected module shows `UNCALIBRATED` until it is calibrated.

1.  **Press and hold the built-in button for 2 to 5 seconds, then release** to start the calibration mode.

**Stage 1: Magnetometer Calibration**
1.  The screen will turn **RED** and show a countdown. For the next **30 seconds**, turn the sensor slowly. Keep away from metal and electronics.
    - **Sensor in hand:** rotate it in all directions, including on its sides and upside down (like drawing a figure-eight in the air). All three axes are calibrated (`CAL 3D OK`).
    - **Sensor mounted on an antenna boom:** keep the boom level and turn it at least one full turn (better two) around the mast. Only the horizontal axes can be measured this way; the vertical axis offset is derived from the magnetic inclination (`MAGNETIC_INCLINATION_DEG` in `src/main.cpp`, 66° for Central Europe) and the screen shows `CAL 2D OK`. Do not run the rotator motor during calibration, its magnet and current disturb the field.
2.  The offsets and axis gains are fitted to all samples (axis-aligned ellipsoid). If the sensor was not turned enough, the screen shows `CAL FAILED` and the previous calibration is kept.

**Stage 2: Level Calibration**
1.  After the magnetometer calibration, the screen will turn **BLUE** and prompt you to place the device on a still, flat surface.
2.  Place the device on a level surface (check it with a spirit level) and wait. The accelerometer zero is measured, which corrects the elevation and the tilt compensation.
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

## License

MIT, see [LICENSE](LICENSE).
