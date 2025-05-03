#!/bin/bash

# Handles existing virtual environments

DEBUG=0
APP_NAME="SCARA Robot Control"
DESKTOP_ENTRY="${HOME}/Desktop/SCARA.desktop"
ICON_PATH="$(dirname "$0")/assets/scara_icon.png"

# Auto-detect common venv locations
VENV_DIRS=(
    "$(dirname "$0")/venv"
    "$(dirname "$0")/scara_venv"
    "$(dirname "$0")/.venv"
)

echo -e "\nInitializing ${APP_NAME}...\n"

# Debug mode
[ "$DEBUG" -eq 1 ] && set -x

# Check Python version
check_python() {
    echo "Checking Python installation..."
    local python_cmd=("python3" "python")

    for cmd in "${python_cmd[@]}"; do
        if command -v "$cmd" >/dev/null 2>&1; then
            if "$cmd" -c "import sys; sys.exit(0 if sys.version_info >= (3, 8) else 1)"; then
                FOUND_PYTHON=$(command -v "$cmd")
                return 0
            fi
        fi
    done

    echo -e "\n[ERROR] Python 3.8+ not found!"
    echo -e "Install Python using:"
    echo "sudo apt update && sudo apt install python3 python3-pip"
    return 1
}

# Find existing venv
find_venv() {
    for venv_path in "${VENV_DIRS[@]}"; do
        if [ -f "$venv_path/bin/activate" ]; then
            VENV_DIR="$venv_path"
            echo "Found existing virtual environment: $VENV_DIR"
            return 0
        fi
    done
    return 1
}

# Setup virtual environment
setup_venv() {
    if ! find_venv; then
        echo "Creating new virtual environment..."
        VENV_DIR="$(dirname "$0")/venv"
        "$FOUND_PYTHON" -m venv "$VENV_DIR" || return 1
    fi

    # Validate activation script
    if [ ! -f "$VENV_DIR/bin/activate" ]; then
        echo -e "\n[ERROR] Invalid virtual environment structure!"
        echo "Missing activation script: $VENV_DIR/bin/activate"
        return 1
    fi

    source "$VENV_DIR/bin/activate" || return 1
    python -m pip install --upgrade pip >/dev/null 2>&1
}

# Install requirements
install_dependencies() {
    echo "Checking dependencies..."
    local requirements_file="$(dirname "$0")/requirements.txt"

    if ! python -m pip show -q pyqt5; then
        echo "Installing PyQt5..."
        python -m pip install pyqt5 || return 1
    fi

    if [ -f "$requirements_file" ]; then
        echo "Installing requirements from $requirements_file..."
        python -m pip install -r "$requirements_file" || return 1
    fi
}

# WSL GUI check
check_wsl_gui() {
    if grep -qi microsoft /proc/version && [ -z "$DISPLAY" ]; then
        echo -e "\n[WARNING] No DISPLAY detected in WSL!"
        echo -e "GUI applications require either:"
        echo -e "1. Windows 11 with WSLg enabled"
        echo -e "2. X server like VcXsrv or Xming"
        echo -e "\nAttempting to launch in 5 seconds..."
        sleep 5
    fi
}

# Create desktop shortcut
create_shortcut() {
    echo "Creating desktop shortcut..."
    cat <<EOF > "$DESKTOP_ENTRY"
[Desktop Entry]
Version=1.0
Type=Application
Name=${APP_NAME}
Exec="$VENV_DIR/bin/python" "$(dirname "$0")/launcher.py"
Path="$(dirname "$0")"
Icon=${ICON_PATH}
Terminal=false
EOF
    chmod +x "$DESKTOP_ENTRY"
}

# Main execution
if ! check_python; then
    read -p "Press enter to exit..."
    exit 1
fi

# Change to script directory
cd "$(dirname "$0")" || exit 1

# Setup virtual environment
if ! setup_venv; then
    echo -e "\n[ERROR] Virtual environment setup failed"
    read -p "Press enter to exit..."
    exit 1
fi

# Install dependencies
if ! install_dependencies; then
    echo -e "\n[ERROR] Dependency installation failed"
    read -p "Press enter to exit..."
    exit 1
fi

# WSL GUI check
check_wsl_gui

# Create/update desktop shortcut
create_shortcut

# Launch application
echo -e "\nStarting ${APP_NAME}...\n"
"$VENV_DIR/bin/python" launcher.py

# Error handling
if [ $? -ne 0 ]; then
    echo -e "\nApplication exited with error $?"
    read -p "Press enter to exit..."
fi