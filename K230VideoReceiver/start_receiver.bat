@echo off
setlocal
cd /d "%~dp0"
if errorlevel 1 goto :error

if not exist ".venv\Scripts\python.exe" (
    echo [setup] Creating Python virtual environment...
    python -m venv .venv
    if errorlevel 1 goto :error
)

".venv\Scripts\python.exe" -c "import cv2" >nul 2>&1
if errorlevel 1 (
    echo [setup] Installing OpenCV...
    ".venv\Scripts\python.exe" -m pip install -r requirements.txt
    if errorlevel 1 goto :error
)

echo [launcher] Starting K230 video receiver...
".venv\Scripts\python.exe" "pc_receiver.py" %*
if errorlevel 1 goto :error
endlocal
exit /b 0

:error
echo [error] Receiver failed. Review the message above.
pause
endlocal
exit /b 1
