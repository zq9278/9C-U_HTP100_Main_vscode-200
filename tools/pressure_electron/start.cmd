@echo off
setlocal
cd /d "%~dp0"
set "ELECTRON_RUN_AS_NODE="
where npm.cmd >nul 2>nul
if errorlevel 1 (
  echo Node.js / npm not found. Please install Node.js first.
  pause
  exit /b 1
)
if not exist "node_modules\.bin\electron.cmd" (
  call npm.cmd ci
  if errorlevel 1 goto failed
)
call npm.cmd start
if errorlevel 1 goto failed
exit /b 0
:failed
echo.
echo Startup failed. See the error above.
pause
exit /b 1
