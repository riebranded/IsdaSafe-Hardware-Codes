@echo off
REM Double-click to test the SEN0711 sensor through the USB-RS485 adapter.
REM Change COM24 below if Device Manager shows a different port.
set PORT=COM24

:again
cls
echo Testing SEN0711 on %PORT% (4800 baud, slave ID 1)...
echo.
powershell -NoProfile -ExecutionPolicy Bypass -File "%~dp0sen0711_test.ps1" -Port %PORT%
echo.
echo Press any key to test again, or close this window to stop.
pause >nul
goto again
