import time
import threading
import numpy as np
import lgpio
from smbus2 import SMBus

# -------------------------------
#   Klasa obsługi enkodera AS5600
# -------------------------------
RAW_ANGLE_HIGH = 0x0C
RAW_ANGLE_LOW = 0x0D

class AS5600:
    def __init__(self, bus_nr):
        self.bus = SMBus(bus_nr)
        self.kat_startowy = None

    def read_raw_angle(self):
        try:
            high = self.bus.read_byte_data(0x36, RAW_ANGLE_HIGH)
            low = self.bus.read_byte_data(0x36, RAW_ANGLE_LOW)
            raw = (high << 8) | low
            angle_deg = (raw & 0x0FFF) * 360.0 / 4096.0
            return round(angle_deg, 2)
        except Exception as e:
            print(f"Błąd odczytu AS5600: {e}")
            return None

    def set_zero(self):
        kat = self.read_raw_angle()
        if kat is not None:
            self.kat_startowy = kat
            print(f"Pozycja startowa ustawiona jako zero: {self.kat_startowy:.2f}°")

    def read_relative_angle(self):
        if self.kat_startowy is None:
            self.set_zero()
        kat_biezacy = self.read_raw_angle()
        if kat_biezacy is None:
            return None
        kat_wzgledny = (kat_biezacy - self.kat_startowy) % 360
        return round(kat_wzgledny, 2)

# -------------------------------
#   Klasa do sterowania silnikami
# -------------------------------
class StepperController:
    def __init__(self):
        # Inicjalizacja GPIO (lgpio)
        self.chip = lgpio.gpiochip_open(0)

        # Piny silników (PAN / TILT)
        self.pan_enable = 18
        self.pan_pwmpin = 23
        self.pan_dirpin = 24
        self.tilt_enable = 16
        self.tilt_pwmpin = 26
        self.tilt_dirpin = 20

        for pin in [self.pan_enable, self.pan_pwmpin, self.pan_dirpin,
                    self.tilt_enable, self.tilt_pwmpin, self.tilt_dirpin]:
            lgpio.gpio_claim_output(self.chip, pin)

        # Wyłączenie silników na start
        lgpio.gpio_write(self.chip, self.pan_enable, 0)
        lgpio.gpio_write(self.chip, self.tilt_enable, 0)

        # Inicjalizacja enkoderów (I2C-1 i I2C-13)
        self.as_pan = AS5600(bus_nr=1)
        self.as_tilt = AS5600(bus_nr=13)

        # Flaga do zatrzymania wątku
        self.stop_event = threading.Event()

        # Uruchom wątek monitorujący kąty
        self.kat_thread = threading.Thread(target=self._pokaz_katy, daemon=True)
        self.kat_thread.start()

        print("✅ StepperController gotowy – silniki i enkodery uruchomione.")

    # -------------------------------
    # Funkcje ruchu silników
    # -------------------------------
    def _step_motor(self, pwm_pin, dir_pin, speed_khz, direction, steps):
        period = 1 / (speed_khz * 1000)
        lgpio.gpio_write(self.chip, dir_pin, direction)
        for _ in range(steps):
            lgpio.gpio_write(self.chip, pwm_pin, 1)
            time.sleep(period)
            lgpio.gpio_write(self.chip, pwm_pin, 0)
            time.sleep(period)

    def move_pan(self, direction, steps, speed_khz=2):
        """Obrót w osi PAN"""
        print(f"\n➡️ Sterowany silnik: PAN | Kierunek: {'prawo' if direction == 1 else 'lewo'} | Kroki: {steps} | Prędkość: {speed_khz} kHz")
        self._step_motor(self.pan_pwmpin, self.pan_dirpin, speed_khz, direction, steps)

    def move_tilt(self, direction, steps, speed_khz=2):
        """Obrót w osi TILT"""
        print(f"\n⬆️ Sterowany silnik: TILT | Kierunek: {'góra' if direction == 1 else 'dół'} | Kroki: {steps} | Prędkość: {speed_khz} kHz")
        self._step_motor(self.tilt_pwmpin, self.tilt_dirpin, speed_khz, direction, steps)

    # -------------------------------
    # Odczyt kątów z enkoderów
    # -------------------------------
    def _pokaz_katy(self):
        while not self.stop_event.is_set():
            kat_pan = self.as_pan.read_relative_angle()
            kat_tilt = self.as_tilt.read_relative_angle()
            if kat_pan is not None and kat_tilt is not None:
                print(f"PAN: {kat_pan:.2f}° | TILT: {kat_tilt:.2f}°", end='\r')
            time.sleep(0.1)

    # -------------------------------
    # Zatrzymanie systemu
    # -------------------------------
    def stop(self):
        self.stop_event.set()
        self.kat_thread.join()
        lgpio.gpiochip_close(self.chip)
        print("\n🛑 Silniki zatrzymane i GPIO zwolnione.")


# -------------------------------
# Test samodzielny
# -------------------------------
if __name__ == "__main__":
    sc = StepperController()
    try:
        sc.move_pan(1, 200, 2)
        time.sleep(1)
        sc.move_tilt(1, 200, 2)
        time.sleep(1)
        sc.move_tilt(0, 200, 2)
        time.sleep(2)
        sc.move_pan(1, 200, 2)
        time.sleep(1)

 
 
    except KeyboardInterrupt:
        pass
    finally:
        sc.stop()
