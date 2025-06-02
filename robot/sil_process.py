import subprocess
import os
import signal
import platform
import shutil
import psutil


class SILServerProcess:
    def __init__(self, server_path="./bin/backend.exe"):
        self.server_path = os.path.abspath(server_path)
        self.proc = None
        self.is_windows = platform.system() == "Windows"

    def start(self, debug=False):
        # Kill any existing server process before starting new one
        self._kill_existing_instances()

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
                self.proc.wait(timeout=2)
            except (ProcessLookupError, subprocess.TimeoutExpired):
                self._kill_process_tree()
            finally:
                self.proc = None

    def _kill_existing_instances(self):
        """Kill any running instances of the server process"""
        server_name = os.path.basename(self.server_path)

        for proc in psutil.process_iter(['pid', 'name']):
            try:
                # Match by executable name
                if proc.name().lower() == server_name.lower():
                    print(f"[Python] Found existing server process (PID: {proc.pid}), terminating...")
                    self._kill_process_tree(proc)
            except (psutil.NoSuchProcess, psutil.AccessDenied):
                continue

    def _kill_process_tree(self, proc=None):
        """Recursively kill process tree starting with given process"""
        target = proc or psutil.Process(self.proc.pid)

        try:
            children = target.children(recursive=True)
            for child in children:
                try:
                    child.kill()
                except psutil.NoSuchProcess:
                    pass
            target.kill()
        except psutil.NoSuchProcess:
            pass
        except Exception as e:
            print(f"[Python] Error killing process tree: {e}")

    def is_running(self):
        return self.proc and self.proc.poll() is None