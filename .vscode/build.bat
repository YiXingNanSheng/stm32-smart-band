@echo off
setlocal
cd /d "%~dp0.."

"D://keil5_MDK//UV4//UV4.exe" -b "Template.uvprojx" -j0 -o "build_vscode.log"

set "ERR=0"
if errorlevel 2 set "ERR=1"

rem Echo build log to terminal, so problemMatcher can parse errors
type "build_vscode.log"

findstr /C:"0 Error(s)" "build_vscode.log" >nul 2>nul
if errorlevel 1 set "ERR=1"

exit /b %ERR%
