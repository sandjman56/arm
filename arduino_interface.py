# arduino_interface.py -- Serial communication with Arduino
import time
import serial


class ArduinoInterface:
    def __init__(self, baudrate=115200):
        self.baudrate = baudrate
        self.ser = None

    def connect(self, port):
        try:
            self.ser = serial.Serial(port, self.baudrate, timeout=1)
            time.sleep(2)  # allow Arduino reset
            self.ser.reset_input_buffer()
            print(f"[INFO] Connected to {port}")
            return True
        except Exception as e:
            print("[ERROR] Connection failed:", e)
            return False

    def send(self, cmd):
        if not self.ser or not self.ser.is_open:
            return False
        try:
            print("TX:", cmd)
            self.ser.write((cmd + "\n").encode())
            return True
        except (serial.SerialException, OSError) as e:
            print(f"[ERROR] Serial write failed: {e}")
            try:
                self.ser.close()
            except Exception:
                pass
            self.ser = None
            return False

    def read_line(self):
        if not self.ser:
            return None
        try:
            line = self.ser.readline()
            if line:
                decoded = line.decode(errors="ignore").strip()
                print("RX:", decoded)
                return decoded
        except:
            return None
        return None

    def close(self):
        if self.ser:
            self.ser.close()


# --- Experiment-mode protocol helpers ---------------------------------------
# Format strings for commands used by experiment_backend.LiveBackend.
# Kept here next to the serial interface so protocol changes live together.

def fmt_set_module(module_id: int, hpa: float) -> str:
    return f"SET {module_id} {hpa:.1f}"


def fmt_tendon(servo_id: int, rate: float) -> str:
    # rate in [-1, 1]; scaled to a firmware-understood int 0-1000 with sign.
    scaled = int(round(max(-1.0, min(1.0, rate)) * 1000))
    return f"TEND {servo_id} {scaled}"
