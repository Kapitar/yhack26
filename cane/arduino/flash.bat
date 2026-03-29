@echo off
REM LeadMe — Flash miniAuto firmware from Windows laptop
REM Run this once (via USB) whenever you update the Arduino sketch.
REM
REM Usage:
REM   flash.bat              auto-detect COM port
REM   flash.bat COM3         specify port explicitly

setlocal EnableDelayedExpansion

set SKETCH=%~dp0leadme_firmware\leadme_firmware.ino
set BOARD=arduino:avr:uno
set PORT=%1

REM ── Check / install arduino-cli ──────────────────────────────────────────────
where arduino-cli >nul 2>&1
if errorlevel 1 (
    echo arduino-cli not found. Downloading...
    powershell -Command "Invoke-WebRequest -Uri https://downloads.arduino.cc/arduino-cli/arduino-cli_latest_Windows_64bit.zip -OutFile '%~dp0arduino-cli.zip'"
    powershell -Command "Expand-Archive '%~dp0arduino-cli.zip' -DestinationPath '%~dp0'"
    del "%~dp0arduino-cli.zip"
    REM Add arduino-cli location to PATH using delayed expansion to avoid
    REM parenthesis collision with NVIDIA/CUDA paths in %PATH%
    set "EXTRA_PATH=%~dp0"
    set "PATH=!EXTRA_PATH!;!PATH!"
)

REM ── Install AVR core if missing ───────────────────────────────────────────────
arduino-cli core list | findstr "arduino:avr" >nul 2>&1
if errorlevel 1 (
    echo Installing Arduino AVR core...
    arduino-cli core update-index
    arduino-cli core install arduino:avr
)

REM ── Auto-detect COM port if not specified ─────────────────────────────────────
if "%PORT%"=="" (
    for /f "tokens=1" %%p in ('arduino-cli board list ^| findstr "COM"') do (
        if "!PORT!"=="" set PORT=%%p
    )
    if "!PORT!"=="" (
        echo ERROR: No Arduino found. Connect via USB and retry.
        echo        Or specify port: flash.bat COM3
        exit /b 1
    )
    echo Auto-detected Arduino on !PORT!
)

REM ── Compile ───────────────────────────────────────────────────────────────────
echo Compiling %SKETCH% ...
arduino-cli compile --fqbn %BOARD% "%SKETCH%"
if errorlevel 1 ( echo Compile failed. & exit /b 1 )

REM ── Upload ────────────────────────────────────────────────────────────────────
echo Uploading to %PORT% ...
arduino-cli upload --fqbn %BOARD% --port %PORT% "%SKETCH%"
if errorlevel 1 ( echo Upload failed. & exit /b 1 )

echo.
echo Done. miniAuto is running LeadMe firmware.
echo Test BLE: python cane\ble_driver.py scan
echo           python cane\ble_driver.py test
