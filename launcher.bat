@echo off
setlocal enabledelayedexpansion

:: Compatible with Windows and WSL

echo.
echo Initializing SCARA Robot Control...
echo.

:: Configuration
set DEBUG=0
set VENV_DIRS=venv scara_venv .venv
set DEFAULT_VENV=venv

:: Check existing virtual environments
echo Checking for existing virtual environments...
for %%D in (%VENV_DIRS%) do (
    if exist "%%D\Scripts\python.exe" (
        set "FOUND=%%~fD\Scripts\python.exe"
        echo Found existing virtual environment: %%~fD
        goto :venv_found
    )
)

:: No venv found - find system Python
echo No virtual environment found, checking system installations...
set PYTHON_PATHS="C:\Python310\python.exe" "C:\Python39\python.exe" ^
 "C:\Program Files\Python310\python.exe" ^
 "%LOCALAPPDATA%\Programs\Python\Python310\python.exe" ^
 "%APPDATA%\Local\Programs\Python\Python310\python.exe"

echo Checking Python installations...
set FOUND=
for %%P in (%PYTHON_PATHS%) do (
    if exist %%P (
        set "FOUND=%%P"
        if %DEBUG% equ 1 echo Found candidate: %%P
        goto :system_python_found
    )
)

echo Checking system PATH...
python --version >nul 2>&1
if %errorlevel% equ 0 (
    set "FOUND=python"
    goto :system_python_found
)

:: Python not found
echo.
echo [ERROR] Python 3.8+ not found!
echo.
echo Please install Python from:
echo https://www.python.org/downloads/
echo.
start "" "https://www.python.org/downloads/"
pause
exit /b 1

:system_python_found
echo.
echo Using system Python at: %FOUND%
echo Creating virtual environment...
"%FOUND%" -m venv %DEFAULT_VENV%
if %errorlevel% neq 0 (
    echo Failed to create virtual environment
    pause
    exit /b 1
)
set "FOUND=%DEFAULT_VENV%\Scripts\python.exe"

:venv_found
echo.
echo Using Python at: %FOUND%
echo.

:: Install requirements
echo Checking dependencies...
"%FOUND%" -m pip show pyqt5 >nul 2>&1
if %errorlevel% neq 0 (
    echo Installing PyQt5...
    "%FOUND%" -m pip install pyqt5
)
if exist requirements.txt (
    echo Installing additional requirements...
    "%FOUND%" -m pip install -r requirements.txt
)

:: Launch application
echo.
echo Starting SCARA Robot Control...
echo.
cd /D "%~dp0"
"%FOUND%" launcher.py

:: Error handling
if %errorlevel% neq 0 (
    echo.
    echo Application exited with error %errorlevel%
    pause
)