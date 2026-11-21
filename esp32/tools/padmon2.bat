@echo off
powershell -NoProfile -ExecutionPolicy Bypass -File "%~dp0padmon2.ps1" -Port COM10
pause
