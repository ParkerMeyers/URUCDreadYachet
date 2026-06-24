@echo off
setlocal
cd /d "%~dp0"
if exist ".venv\Scripts\python.exe" (
    ".venv\Scripts\python.exe" main_control_ui.py %*
) else if exist "venv\Scripts\python.exe" (
    "venv\Scripts\python.exe" main_control_ui.py %*
) else (
    python main_control_ui.py %*
)
