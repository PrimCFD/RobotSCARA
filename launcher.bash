#!/bin/bash

# SCARA Robot Control Launcher (Linux/macOS/WSL)

DEBUG=0
APP_NAME="SCARA Robot Control"
DESKTOP_ENTRY="${HOME}/Desktop/SCARA.desktop"
ICON_PATH="$(dirname "$0")/assets/scara_icon.png"
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
VENV_DIRS=("$SCRIPT_DIR/venv" "$SCRIPT_DIR/scara_venv" "$SCRIPT_DIR/.venv")
BUILD_DIR="$SCRIPT_DIR/backend/build"
BIN_PATH="$SCRIPT_DIR/bin/backend"  # Changed to project root path

echo -e "\nInitializing ${APP_NAME}...\n"
[ "$DEBUG" -eq 1 ] && set -x

# ----------------------------------------
# Check Python installation
# ----------------------------------------
check_python() {
    echo "Checking Python installation..."
    for cmd in python3 python; do
        if command -v "$cmd" >/dev/null 2>&1 && "$cmd" -c "import sys; exit(0) if sys.version_info >= (3,8) else exit(1)"; then
            FOUND_PYTHON=$(command -v "$cmd")
            return 0
        fi
    done
    echo -e "\n[ERROR] Python 3.8+ not found!"
    echo "Try: sudo apt install python3 python3-pip"
    return 1
}

# ----------------------------------------
# Setup Python virtual environment
# ----------------------------------------
setup_venv() {
    for venv in "${VENV_DIRS[@]}"; do
        if [ -f "$venv/bin/activate" ]; then
            VENV_DIR="$venv"
            echo "Found existing virtual environment: $VENV_DIR"
            source "$VENV_DIR/bin/activate"
            return 0
        fi
    done

    echo "Creating new virtual environment..."
    VENV_DIR="$SCRIPT_DIR/venv"
    "$FOUND_PYTHON" -m venv "$VENV_DIR" || return 1
    source "$VENV_DIR/bin/activate"
    python -m pip install --upgrade pip
}

# ----------------------------------------
# Install Python dependencies
# ----------------------------------------
install_dependencies() {
    echo "Checking dependencies..."
    if ! python -m pip show pyqt5 >/dev/null 2>&1; then
        echo "Installing PyQt5..."
        python -m pip install pyqt5 || return 1
    fi
    if [ -f "$SCRIPT_DIR/requirements.txt" ]; then
        echo "Installing requirements from requirements.txt..."
        python -m pip install -r "$SCRIPT_DIR/requirements.txt" || return 1
    fi
}

# ----------------------------------------
# WSL GUI check
# ----------------------------------------
check_wsl_gui() {
    if grep -qi microsoft /proc/version && [ -z "$DISPLAY" ]; then
        echo -e "\n[WARNING] No DISPLAY detected in WSL!"
        echo "Enable WSLg or start an X server (e.g., VcXsrv)"
        sleep 5
    fi
}

# ----------------------------------------
# Create desktop shortcut
# ----------------------------------------
create_shortcut() {
    echo "Creating desktop shortcut..."
    cat <<EOF > "$DESKTOP_ENTRY"
[Desktop Entry]
Version=1.0
Type=Application
Name=${APP_NAME}
Exec="$VENV_DIR/bin/python" "$SCRIPT_DIR/launcher.py"
Path="$SCRIPT_DIR"
Icon=${ICON_PATH}
Terminal=false
EOF
    chmod +x "$DESKTOP_ENTRY"
}

# ----------------------------------------
# Conditionally build C++ backend
# ----------------------------------------
build_backend() {
    # NEW: Check if backend already exists
    if [ -f "$BIN_PATH" ]; then
        echo "Backend executable already exists. Skipping build."
        return 0
    fi

    echo "Building C++ backend..."
    mkdir -p "$BUILD_DIR"
    pushd "$BUILD_DIR" >/dev/null
    cmake .. -DCMAKE_BUILD_TYPE=Release
    cmake --build . --config Release
    popd >/dev/null

    if [ ! -f "$BIN_PATH" ]; then
        echo "[ERROR] Backend build failed: $BIN_PATH not found"
        return 1
    fi
}

# ----------------------------------------
# Main Execution
# ----------------------------------------
if ! check_python; then read -p "Press enter to exit..."; exit 1; fi
cd "$SCRIPT_DIR" || exit 1

if ! setup_venv; then
    echo -e "\n[ERROR] Failed to set up virtual environment"
    read -p "Press enter to exit..."
    exit 1
fi

if ! install_dependencies; then
    echo -e "\n[ERROR] Failed to install Python dependencies"
    read -p "Press enter to exit..."
    exit 1
fi

check_wsl_gui

# Always try to build (function handles conditional)
if ! build_backend; then
    echo -e "\n[ERROR] Backend C++ build failed"
    read -p "Press enter to exit..."
    exit 1
fi

create_shortcut

# Launch app
echo -e "\nLaunching ${APP_NAME}..."
"$VENV_DIR/bin/python" launcher.py

if [ $? -ne 0 ]; then
    echo -e "\nApplication exited with error $?"
    read -p "Press enter to exit..."
fi