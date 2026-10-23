@echo off
setlocal
cd /d "%~dp0.."

"D://keil5_MDK//UV4//UV4.exe" -b "Template.uvprojx" -j0 -o "build_vscode.log"

if errorlevel 2 exit /b 1
findstr /C:"0 Error(s)" "build_vscode.log" >nul 2>nul
if errorlevel 1 exit /b 1
exit /b 0
