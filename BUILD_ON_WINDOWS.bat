@echo off
setlocal
cd /d "%~dp0"
where pio >nul 2>nul
if errorlevel 1 (
  echo PlatformIO CLI was not found in PATH.
  echo Open this folder in VS Code with PlatformIO and click Build instead.
  pause
  exit /b 1
)

echo Building Elantra N Gauge...
pio run -e esp32s3_amoled
if errorlevel 1 (
  echo.
  echo BUILD FAILED.
  pause
  exit /b 1
)

set "BIN=.pio\build\esp32s3_amoled\firmware.bin"
if not exist "%BIN%" (
  echo.
  echo Build reported success but firmware.bin was not found.
  pause
  exit /b 1
)

if not exist "OTA_FIRMWARE" mkdir "OTA_FIRMWARE"
copy /y "%BIN%" "OTA_FIRMWARE\ElantraN_Gauge_OTA.bin" >nul
if errorlevel 1 (
  echo.
  echo BUILD SUCCEEDED, but the phone-update copy could not be created.
  echo Firmware is still at: %BIN%
  pause
  exit /b 1
)

echo.
echo BUILD SUCCESS.
echo USB/PlatformIO binary: %BIN%
echo Phone OTA binary: OTA_FIRMWARE\ElantraN_Gauge_OTA.bin
echo.
echo For future phone-only updates, copy ElantraN_Gauge_OTA.bin to your phone,
echo enter gauge setup mode, open 192.168.4.1, and use Firmware Update.
pause
