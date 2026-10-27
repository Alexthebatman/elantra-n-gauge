# Elantra N Gauge

Firmware project for the ESP32-S3 round AMOLED Elantra N gauge.

## Automatic phone OTA builds

Every push to `main` that changes the firmware automatically compiles the project with PlatformIO and replaces the GitHub Release named **Latest Gauge Firmware**.

The file to install on the gauge is:

`ElantraN_Gauge_OTA.bin`

Do **not** upload the source ZIP, bootloader, partition table, or any other binary through the gauge OTA page.

### Update from iPhone

1. Open this repository in GitHub.
2. Open **Releases**.
3. Open **Latest Gauge Firmware**.
4. Download `ElantraN_Gauge_OTA.bin` into the iPhone Files app.
5. With the gauge powered on, hold the RIGHT button for about 2.5 seconds to enter setup mode.
6. Join `ElantraN-Gauge-Setup` using the password shown on the gauge.
7. In Safari, open `http://192.168.4.1`.
8. Choose `ElantraN_Gauge_OTA.bin` and install it.
9. Keep power to the gauge until the update completes and it reboots.

## Local Windows build

Run `BUILD_ON_WINDOWS.bat` from this project directory if PlatformIO CLI is installed. It creates:

`OTA_FIRMWARE\ElantraN_Gauge_OTA.bin`

The cloud workflow means this local build should normally be unnecessary after GitHub is set up.
