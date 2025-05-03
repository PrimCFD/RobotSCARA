# launcher.py
import os
import sys
import subprocess
import shutil
from pathlib import Path
from time import sleep

try:
    from tqdm import tqdm
    import colorama
except ImportError:
    subprocess.check_call([sys.executable, "-m", "pip", "install", "tqdm", "colorama"])
    from tqdm import tqdm
    import colorama

# Initialize color output
colorama.init()
GREEN = colorama.Fore.GREEN
YELLOW = colorama.Fore.YELLOW
RESET = colorama.Style.RESET_ALL

REQUIRED_DIRS = [
    "assets",
    "configs",
    "robot",
    "bin",
    "data",
    "buffer"
]


def check_environment():
    """Verify basic Python requirements"""
    print(f"{YELLOW}Checking Python version...{RESET}")
    if sys.version_info < (3, 8):
        print("Python 3.8 or newer is required")
        sys.exit(1)


def install_dependencies():
    """Install required packages"""
    print(f"\n{YELLOW}Checking dependencies...{RESET}")
    subprocess.check_call([
        sys.executable, "-m", "pip", "install", "-r", "requirements.txt"
    ])


def setup_application():
    """Create directory structure with progress"""
    print(f"\n{YELLOW}Setting up application...{RESET}")

    # Create required directories with progress
    for directory in tqdm(REQUIRED_DIRS, desc="Creating directories"):
        Path(directory).mkdir(exist_ok=True)

    # Copy essential files
    essential_files = {
        "buffer/buffer.csv": "buffer/buffer.csv exists" or Path("buffer/buffer.csv").touch()
    }


def launch_app():
    """Start the main application"""
    print(f"\n{GREEN}Launching SCARA Robot Control...{RESET}")
    subprocess.Popen([sys.executable, "SCARA.py"])


def main():
    try:
        check_environment()
        install_dependencies()
        setup_application()
        launch_app()
    except Exception as e:
        print(f"\n{colorama.Fore.RED}Error: {str(e)}{RESET}")
        print("Please contact support with this error message")
        sleep(5)  # Keep window open


if __name__ == "__main__":
    main()