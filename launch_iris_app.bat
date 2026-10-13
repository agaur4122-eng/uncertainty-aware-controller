@echo off
TITLE IRIS 2026-27 Autonomous Driving Research Workbench Launcher
COLOR 0B
echo ======================================================================
echo    IRIS NATIONAL FAIR 2026-27: CARLA RESEARCH APPLICATION LAUNCHER
echo ======================================================================
echo.

REM 1. Check Python Installation
python --version >nul 2>&1
if %errorlevel% neq 0 (
    echo [ERROR] Python 3 is not installed or added to PATH.
    echo Please install Python 3.10+ from python.org and check "Add to PATH".
    pause
    exit /b
)

echo [CHECK] Python environment detected successfully.

echo [CHECK] Verifying required scientific and GUI dependencies...
python -c "import PyQt5" >nul 2>&1
if %errorlevel% neq 0 (
    echo [INSTALLING] PyQt5 GUI library...
    pip install PyQt5
)

python -c "import matplotlib" >nul 2>&1
if %errorlevel% neq 0 (
    echo [INSTALLING] Matplotlib scientific plotting library...
    pip install matplotlib
)

echo [CHECK] Checking CARLA Simulator Server connection on port 2000...
python -c "import socket; s = socket.socket(); s.settimeout(1); res = s.connect_ex(('localhost', 2000)); exit(0 if res == 0 else 1)"
if %errorlevel% neq 0 (
    echo.
    echo [WARNING] CARLA Simulator server is not currently running on port 2000.
    echo The application will launch in HIGH-FIDELITY MATHEMATICAL SIMULATION mode.
    echo To use live 3D graphics, please launch your CARLA Unreal Engine server (CarlaUE4.exe).
    echo.
) else (
    echo [SUCCESS] Active CARLA Simulator server detected on port 2000!
)

echo.
echo [LAUNCHING] Starting IRIS Research Workbench Application...
start python iris_carla_app.py
exit