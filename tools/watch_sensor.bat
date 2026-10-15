@echo off
REM Double-click for a live view of the SEN0711 readings (one line every 3 s).
REM Change COM24 below if Device Manager shows a different port.
set PORT=COM24
powershell -NoProfile -ExecutionPolicy Bypass -File "%~dp0watch_sensor.ps1" -Port %PORT%
pause
