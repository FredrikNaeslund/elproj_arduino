@echo off
echo === Building and uploading to ESP8266 ===
pio run -t upload
if %errorlevel% neq 0 (
    echo.
    echo Upload failed!
    pause
    exit /b 1
)
echo.
echo === Upload complete! Opening serial monitor ===
echo (Press Ctrl+C to exit monitor)
echo.
pio device monitor
pause
