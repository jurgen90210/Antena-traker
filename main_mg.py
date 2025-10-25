# main_mg.py
import time
import threading
from imu_main_mg import IMUSensor
from steeper_main_mg import StepperController, CW, CCW

class CombinedSystem:
    def __init__(self):
        self.imu = IMUSensor()
        self.step = StepperController()
        self.data = {}
        self._stop = threading.Event()

        # Uruchomienie wątków
        threading.Thread(target=self._update_loop, daemon=True).start()
        threading.Thread(target=self._display_loop, daemon=True).start()

    # --- Pętla odczytu danych z IMU i enkodera silnika (tylko PAN) ---
    def _update_loop(self):
        while not self._stop.is_set():
            d_imu = self.imu.read_all()
            pan = self.step.get_pan_angle()   # tylko enkoder PAN
            self.data = {"pan": pan, **d_imu}
            time.sleep(0.05)

    # --- Pętla wyświetlania wyników ---
    def _display_loop(self):
        while not self._stop.is_set():
            d = self.data
            if not d:
                time.sleep(0.1)
                continue
            left  = f"PAN: {d['pan']:7.2f}°"
            right = (f"Pitch: {d['pitch']:6.2f}° | Roll: {d['roll']:6.2f}° | "
                     f"Yaw: {d['yaw']:6.2f}° | T: {d['temperature']:5.2f}°C | "
                     f"P: {d['pressure']:7.2f}hPa | Alt: {d['altitude']:6.2f}m")
            print(f"{left:<25} || {right}", end="\r", flush=True)
            time.sleep(0.05)

    # --- Sekwencja ruchów silników ---
    def run_sequence(self):
        self.step.move_pan(CW, 300)
        self.step.move_pan(CCW, 300)
        self.step.move_tilt(CW, 200)
        self.step.move_tilt(CCW, 200)

    # --- Zatrzymanie systemu ---
    def stop(self):
        self._stop.set()
        self.step.stop()
        print("\n🛑 System zatrzymany.")


# =========================================================
#  MAIN
# =========================================================
if __name__ == "__main__":
    sys = CombinedSystem()
    try:
        sys.run_sequence()
        time.sleep(2)
    except KeyboardInterrupt:
        pass
    finally:
        sys.stop()
