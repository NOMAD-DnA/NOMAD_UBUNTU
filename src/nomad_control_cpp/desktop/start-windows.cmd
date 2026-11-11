@echo off
setlocal
"%SystemRoot%\System32\WindowsPowerShell\v1.0\powershell.exe" -NoProfile -ExecutionPolicy Bypass -File "%~dp0..\scripts\Launch-NomadGui.ps1" -NoBrowser
if errorlevel 1 (
  pause
  exit /b 1
)
start "" "%~dp0bin\nomad_desktop.exe"
