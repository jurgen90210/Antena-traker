#!/usr/bin/env python3
import time
import lgpio


# -------------------------------
#   Klasa sterowania silnikami (TMC2209)
# -------------------------------
class StepperController:
    def __init__(self):
        # Otwórz dostęp do głównego kontrolera GPIO (RPi 5)
        self.chip = lgpio.gpiochip_open(0)

        # --- Piny silników ---
        # PAN (azymut)
        self.pan_step = 23
        self.pan_dir  = 24
        self.pan_en   = 18

        # TILT (elewacja)
        self.tilt_step = 26
        self.tilt_dir  = 20
        self.tilt_en   = 16

        # Skonfiguruj piny jako wyjścia
        for pin in [
            self.pan_step, self.pan_dir, self.pan_en,
            self.tilt_step, self.tilt_dir, self.tilt_en,
        ]:
            lgpio.gpio_claim_output(self.chip, pin)

        # Wyłącz silniki na start (EN=1 → nieaktywne)
        lgpio.gpio_write(self.chip, self.pan_en, 1)
        lgpio.gpio_write(self.chip, self.tilt_en, 1)

        print("✅ StepperController gotowy – tylko sterowanie silnikami")

    # --- Ruch silnika krokowego ---
    def _step_motor(self, step_pin, dir_pin, en_pin, direction, steps, speed_khz):
        """
        direction: 1 = CW, 0 = CCW
        speed_khz: częstotliwość impulsów krokowych w kHz
        """
        period = 1 / (speed_khz * 1500.0)
        lgpio.gpio_write(self.chip, en_pin, 0)           # aktywacja sterownika (EN = LOW)
        lgpio.gpio_write(self.chip, dir_pin, direction)  # kierunek
        for _ in range(steps):
            lgpio.gpio_write(self.chip, step_pin, 1)
            time.sleep(period)
            lgpio.gpio_write(self.chip, step_pin, 0)
            time.sleep(period)
        lgpio.gpio_write(self.chip, en_pin, 1)           # dezaktywacja sterownika (EN = HIGH)

    # --- Publiczne funkcje ruchu ---
    def move_pan(self, direction, steps, speed_khz=2):
        print(f"➡️  PAN: {'prawo' if direction == 1 else 'lewo'} | {steps} kroków")
        self._step_motor(self.pan_step, self.pan_dir, self.pan_en, direction, steps, speed_khz)

    def move_tilt(self, direction, steps, speed_khz=2):
        print(f"⬆️  TILT: {'góra' if direction == 1 else 'dół'} | {steps} kroków")
        self._step_motor(self.tilt_step, self.tilt_dir, self.tilt_en, direction, steps, speed_khz)

    # --- Zatrzymanie i zwolnienie GPIO ---
    def stop(self):
        lgpio.gpio_write(self.chip, self.pan_en, 0)
        lgpio.gpio_write(self.chip, self.tilt_en, 0)
        lgpio.gpiochip_close(self.chip)
        print("🛑 Silniki zatrzymane i GPIO zwolnione.")


# -------------------------------
#   Test działania
# -------------------------------
if __name__ == "__main__":
    sc = StepperController()
    try:
        # 1. PAN w lewo
        sc.move_pan(0, 600, 2)
        time.sleep(1)
        # 2. TILT w dół
        sc.move_tilt(0, 600, 2)
        time.sleep(1)
        # 3. PAN w prawo
        sc.move_pan(1, 600, 2)
        time.sleep(1)
        # 4. TILT w górę
        sc.move_tilt(1, 600, 2)
        time.sleep(1)
    except KeyboardInterrupt:
        pass
    finally:
        sc.stop()
