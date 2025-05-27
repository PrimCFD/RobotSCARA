import subprocess
import os
import signal
import platform
import shutil

class SILServerProcess:
    def __init__(self, server_path="./bin/backend.exe"):
        self.server_path = os.path.abspath(server_path)
        self.proc = None
        self.is_windows = platform.system() == "Windows"

    def start(self, debug=False):
        if self.proc is None or self.proc.poll() is not None:
            if not os.path.exists(self.server_path):
                raise FileNotFoundError(f"SIL server binary not found at: {self.server_path}")

            print("[Python] Launching SIL server...")
            kwargs = {
                "stdout": None if debug else subprocess.DEVNULL,
                "stderr": None if debug else subprocess.DEVNULL,
                "shell": False
            }

            if self.is_windows:
                kwargs["creationflags"] = subprocess.CREATE_NEW_PROCESS_GROUP
            else:
                kwargs["preexec_fn"] = os.setsid

            self.proc = subprocess.Popen([self.server_path], **kwargs)

    def stop(self):
        if self.proc and self.proc.poll() is None:
            print("[Python] Stopping SIL server...")
            try:
                if self.is_windows:
                    os.kill(self.proc.pid, signal.CTRL_BREAK_EVENT)
                else:
                    os.killpg(os.getpgid(self.proc.pid), signal.SIGTERM)
                self.proc.wait(timeout=5)
            except (ProcessLookupError, subprocess.TimeoutExpired):
                self.proc.kill()

    def is_running(self):
        return self.proc and self.proc.poll() is None
