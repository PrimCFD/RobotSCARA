import serial, time
from serial.tools import list_ports  # NEW
from PyQt5 import QtCore

ARDUINO_IDS = {
    (0x2341, 0x0043),     # Original Arduino UNO
    (0x2A03, 0x0043),     # Arduino.org UNO
    (0x1A86, 0x7523),     # CH340/CH341 clone
}

def find_arduino():
    """Return the first serial.device that looks like an Arduino UNO (or None)."""
    for p in list_ports.comports():
        if (p.vid, p.pid) in ARDUINO_IDS:
            return p.device
    return None

class SensorThread(QtCore.QThread):
    new_angles = QtCore.pyqtSignal(float, float)

    def __init__(self, port='auto', baud=115200, parent=None):
        super().__init__(parent)
        self._requested_port = port     # keep the user request (“/dev/x” or “auto”)
        self._baud  = baud
        self._alive = True
        self._ser   = None

    def _open_port(self):
        if self._requested_port != 'auto':
            return serial.Serial(self._requested_port, self._baud, timeout=1)

        port = find_arduino()
        if port:
            return serial.Serial(port, self._baud, timeout=1)
        return None                     # nothing found yet

    def run(self):
        while self._alive:
            try:
                if self._ser is None or not self._ser.is_open:
                    self._ser = self._open_port()
                    if self._ser is None:
                        time.sleep(1)   # wait and retry
                        continue

                line = self._ser.readline().decode(errors='ignore').strip()
                if not line:
                    continue
                try:
                    _, _, th, ph = map(float, line.split(','))
                    self.new_angles.emit(th, ph)
                except ValueError:
                    pass                # malformed line, ignore
            except serial.SerialException:
                # Lost the device – close and try again
                if self._ser:
                    self._ser.close()
                self._ser = None
                time.sleep(1)

    def stop(self):
        self._alive = False
        self.wait()