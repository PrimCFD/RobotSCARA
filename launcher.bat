@echo off
setlocal enabledelayedexpansion

:: Change to script directory
cd /D "%~dp0"

:: ----------------------------------------
:: SCARA Robot Control Launcher
:: Compatible with Windows and WSL
:: ----------------------------------------

echo.
echo Initializing SCARA Robot Control...
echo.

:: ----------------------------------------
:: CONFIGURATION
:: ----------------------------------------
set DEBUG=0
set VENV_DIRS=venv scara_venv .venv
set DEFAULT_VENV=venv
set BACKEND_DIR=backend
set BUILD_DIR=%BACKEND_DIR%\build
set EXECUTABLE_PATH=bin\backend.exe

:: ----------------------------------------
:: STEP 1: Check existing virtual environments
:: ----------------------------------------
echo Checking for existing virtual environments...
set FOUND=
for %%D in (%VENV_DIRS%) do (
    if exist "%%D\Scripts\python.exe" (
        set "FOUND=%%~fD\Scripts\python.exe"
        echo Found existing virtual environment: %%~fD
        goto :venv_found
    )
)

:: If no venv found, try to create one
echo Creating new virtual environment in %DEFAULT_VENV%...
python -m venv "%DEFAULT_VENV%"
if not errorlevel 1 (
    set "FOUND=%cd%\%DEFAULT_VENV%\Scripts\python.exe"
    goto :venv_found
)

:: Fallback: Search for system Python installations
echo.
echo Searching for system Python installations...
set PYTHON_PATHS="C:\Python310\python.exe" "C:\Python39\python.exe" "C:\Python38\python.exe" "C:\Python37\python.exe" "C:\Program Files\Python310\python.exe" "C:\Program Files\Python39\python.exe" "C:\Program Files\Python38\python.exe" "C:\Program Files\Python37\python.exe" "%LOCALAPPDATA%\Programs\Python\Python310\python.exe" "%LOCALAPPDATA%\Programs\Python\Python39\python.exe" "%LOCALAPPDATA%\Programs\Python\Python38\python.exe" "%LOCALAPPDATA%\Programs\Python\Python37\python.exe" "%APPDATA%\Local\Programs\Python\Python310\python.exe" "%APPDATA%\Local\Programs\Python\Python39\python.exe" "%APPDATA%\Local\Programs\Python\Python38\python.exe" "%APPDATA%\Local\Programs\Python\Python37\python.exe"

set FOUND_PYTHON=
for %%P in (%PYTHON_PATHS%) do (
    if exist %%P (
        set "FOUND_PYTHON=%%P"
        echo Found Python at: %%P
        goto :create_venv_with_system
    )
)

:create_venv_with_system
if "!FOUND_PYTHON!"=="" (
    echo No Python installation found. Please install Python and try again.
    pause
    exit /b 1
)

echo Creating virtual environment using !FOUND_PYTHON!...
"!FOUND_PYTHON!" -m venv "%DEFAULT_VENV%"
if errorlevel 1 (
    echo Failed to create virtual environment
    pause
    exit /b 1
)
set "FOUND=%cd%\%DEFAULT_VENV%\Scripts\python.exe"

:venv_found
if "%FOUND%"=="" (
    echo Fatal: Python executable not found
    pause
    exit /b 1
)

echo.
echo Using Python at: %FOUND%
echo.

:: ----------------------------------------
:: STEP 2: Install dependencies
:: ----------------------------------------
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

:: ----------------------------------------
:: STEP 3: Clean and rebuild backend executable
:: ----------------------------------------
echo.
echo Cleaning and rebuilding backend executable...

:: Remove existing build directory
if exist "%BUILD_DIR%" (
    echo Removing existing build directory...
    rmdir /s /q "%BUILD_DIR%"
)

:: Create fresh build directory
mkdir "%BUILD_DIR%"
if errorlevel 1 (
    echo Failed to create build directory
    pause
    exit /b 1
)

:: Configure and build
pushd "%BUILD_DIR%"
cmake .. -DCMAKE_BUILD_TYPE=Release
if errorlevel 1 (
    popd
    echo CMake configuration failed
    pause
    exit /b 1
)

cmake --build . --config Release
if errorlevel 1 (
    popd
    echo Build failed
    pause
    exit /b 1
)
popd

:: Confirm executable exists
if not exist "%EXECUTABLE_PATH%" (
    echo.
    echo [ERROR] Backend build failed: %EXECUTABLE_PATH% not found.
    pause
    exit /b 1
)

:: ----------------------------------------
:: STEP 4: Launch Python application
:: ----------------------------------------
echo.
echo Starting SCARA Robot Control...
echo.

cd /D "%~dp0"
"%FOUND%" launcher.py

if %errorlevel% neq 0 (
    echo.
    echo Application exited with error %errorlevel%
    pause
)