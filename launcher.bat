@echo off
setlocal enabledelayedexpansion

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
set EXECUTABLE=.\bin\backend.exe
set EXECUTABLE_PATH=.\bin\backend.exe  // NEW: Path relative to project root

:: ----------------------------------------
:: STEP 1: Check existing virtual environments
:: ----------------------------------------
echo Checking for existing virtual environments...
for %%D in (%VENV_DIRS%) do (
    if exist "%%D\Scripts\python.exe" (
        set "FOUND=%%~fD\Scripts\python.exe"
        echo Found existing virtual environment: %%~fD
        goto :venv_found
    )
)

:: ... [existing Python discovery code remains the same] ...

:venv_found
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
:: STEP 3: Conditionally build backend executable
:: ----------------------------------------
echo.
:: NEW: Check if executable exists before building
if exist "%EXECUTABLE_PATH%" (
    echo Backend executable already exists. Skipping build.
) else (
    echo Building backend executable...

    :: Ensure build directory exists
    if not exist "%BUILD_DIR%" (
        mkdir "%BUILD_DIR%"
    )

    pushd "%BUILD_DIR%"
    cmake .. -DCMAKE_BUILD_TYPE=Release
    cmake --build . --config Release
    popd

    :: Confirm executable exists
    if not exist "%EXECUTABLE_PATH%" (
        echo.
        echo [ERROR] Backend build failed: %EXECUTABLE_PATH% not found.
        pause
        exit /b 1
    )
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