@echo off
setlocal
cd /d "%~dp0"
if not exist ".venv\Scripts\python.exe" (
  set "PYTHON_EXE="
  for %%P in ("%LocalAppData%\Programs\Python\Python3*\python.exe") do if exist "%%~fP" set "PYTHON_EXE=%%~fP"
  if not defined PYTHON_EXE (
    where py >nul 2>&1
    if errorlevel 1 (
      echo Python 3 was not found. Please install Python 3 first.
      pause
      exit /b 1
    )
    py -3 -m venv .venv
  ) else (
    "%PYTHON_EXE%" -m venv .venv
  )
  if errorlevel 1 pause & exit /b 1
  .venv\Scripts\python.exe -m pip install -r requirements.txt
  if errorlevel 1 pause & exit /b 1
)
.venv\Scripts\python.exe -m PyInstaller --noconfirm --clean --onefile --windowed --distpath dist_ram --workpath build_ram_v8 --name HTP100_Pressure_Tuning_Aging_EEPROM_TableSync_Host_v8 pressure_tuning_host.py
if errorlevel 1 pause
