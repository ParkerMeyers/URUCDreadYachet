@echo off
setlocal
cd /d "%~dp0"
if exist "C:\Program Files\gstreamer\1.0\msvc_x86_64\bin" (
    set "PATH=C:\Program Files\gstreamer\1.0\msvc_x86_64\bin;%PATH%"
)
if exist ".venv\Scripts\python.exe" (
    ".venv\Scripts\python.exe" main_control_ui.py %*
) else if exist "venv\Scripts\python.exe" (
    "venv\Scripts\python.exe" main_control_ui.py %*
) else (
    python main_control_ui.py %*
)
