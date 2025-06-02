#!/bin/bash

# SCARA Robot Control Launcher (Linux/macOS/WSL)

DEBUG=0
APP_NAME="SCARA Robot Control"
DESKTOP_ENTRY="${HOME}/Desktop/SCARA.desktop"
ICON_PATH="$(dirname "$0")/assets/scara_icon.png"
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
VENV_DIRS=("$SCRIPT_DIR/venv" "$SCRIPT_DIR/scara_venv" "$SCRIPT_DIR/.venv")
BUILD_DIR="$SCRIPT_DIR/backend/build"
BIN_PATH="$SCRIPT_DIR/bin/backend"

echo -e "\nInitializing ${APP_NAME}...\n"
[ "$DEBUG" -eq 1 ] && set -x

# ----------------------------------------
# Enhanced Python installation check
# ----------------------------------------
check_python() {
    echo "Checking Python installation..."
    local min_version=3.8

    # First check standard commands
    for cmd in python3.8 python3.9 python3.10 python3.11 python3.12 python3 python; do
        if command -v "$cmd" >/dev/null 2>&1; then
            if "$cmd" -c "import sys; sys.exit(0) if sys.version_info >= (3,8) else sys.exit(1)"; then
                FOUND_PYTHON=$(command -v "$cmd")
                echo "Found compatible Python: $FOUND_PYTHON"
                return 0
            fi
        fi
    done

    # Fallback: Check common installation paths
    echo "Searching common Python installations..."
    local common_paths=(
        "/usr/bin/python3.8" "/usr/bin/python3.9" "/usr/bin/python3.10" "/usr/bin/python3.11" "/usr/bin/python3.12"
        "/usr/local/bin/python3.8" "/usr/local/bin/python3.9" "/usr/local/bin/python3.10" "/usr/local/bin/python3.11"
        "/opt/homebrew/bin/python3.8" "/opt/homebrew/bin/python3.9" "/opt/homebrew/bin/python3.10"  # macOS M1
        "/opt/python/bin/python3.8" "/opt/python/bin/python3.9"  # Custom installs
        "${HOME}/.local/bin/python3.8" "${HOME}/.local/bin/python3.9"
    )

    for path in "${common_paths[@]}"; do
        if [ -x "$path" ]; then
            if "$path" -c "import sys; sys.exit(0) if sys.version_info >= (3,8) else sys.exit(1)"; then
                FOUND_PYTHON="$path"
                echo "Found compatible Python at: $FOUND_PYTHON"
                return 0
            fi
        fi
    done

    echo -e "\n[ERROR] Python ${min_version}+ not found!"
    echo "For Debian/Ubuntu: sudo apt install python3 python3-pip"
    echo "For macOS: brew install python@3.8"
    echo "For manual install: https://www.python.org/downloads/"
    return 1
}

# ----------------------------------------
# Virtual environment setup (enhanced)
# ----------------------------------------
setup_venv() {
    for venv in "${VENV_DIRS[@]}"; do
        if [ -f "$venv/bin/activate" ]; then
            VENV_DIR="$venv"
            echo "Found existing virtual environment: $VENV_DIR"
            return 0
        fi
    done

    echo "Creating new virtual environment in $SCRIPT_DIR/venv..."
    "$FOUND_PYTHON" -m venv "$SCRIPT_DIR/venv" || {
        echo "[ERROR] Failed to create virtual environment"
        return 1
    }

    VENV_DIR="$SCRIPT_DIR/venv"
    return 0
}

# ----------------------------------------
# Dependency installation (robust)
# ----------------------------------------
install_dependencies() {
    echo "Checking dependencies..."
    local pip_exe="$VENV_DIR/bin/pip"

    # Ensure pip is up-to-date first
    echo "Upgrading pip..."
    "$pip_exe" install --upgrade pip || return 1

    # Install PyQt5 with fallback for different platforms
    if ! "$pip_exe" show pyqt5 >/dev/null 2>&1; then
        echo "Installing PyQt5..."
        if [[ "$OSTYPE" == "linux-gnu"* ]]; then
            "$pip_exe" install pyqt5 --no-cache-dir || return 1
        else
            "$pip_exe" install pyqt5 || return 1
        fi
    fi

    # Install other requirements
    if [ -f "$SCRIPT_DIR/requirements.txt" ]; then
        echo "Installing requirements from requirements.txt..."
        "$pip_exe" install -r "$SCRIPT_DIR/requirements.txt" || {
            echo "[WARNING] Some dependencies failed - attempting with --use-pep517"
            "$pip_exe" install --use-pep517 -r "$SCRIPT_DIR/requirements.txt" || return 1
        }
    fi
    return 0
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
# Build C++ backend (always rebuild)
# ----------------------------------------
build_backend() {
    # Clean previous build artifacts
    if [ -d "$BUILD_DIR" ]; then
        echo "Cleaning previous build artifacts..."
        rm -rf "$BUILD_DIR"
    fi

    echo "Building C++ backend..."
    mkdir -p "$BUILD_DIR"
    pushd "$BUILD_DIR" >/dev/null
    # Configure and build project
    cmake .. -DCMAKE_BUILD_TYPE=Release
    cmake --build . --config Release
    popd >/dev/null

    # Verify build success
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

# Build C++
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