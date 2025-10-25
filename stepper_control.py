import time
import threading
import lgpio
from smbus2 import SMBus

# -------------------------------
#   Klasa obsługi AS5600 (TILT)
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
#   Klasa enkodera optycznego E2 (PAN)
# -------------------------------
class OpticalEncoderE2:
    """Obsługa enkodera kwadraturowego x4, automatycznie wybiera tryb: alert/polling"""
    _transitions = {
        (0b00, 0b01): +1, (0b01, 0b11): +1, (0b11, 0b10): +1, (0b10, 0b00): +1,
        (0b00, 0b10): -1, (0b10, 0b11): -1, (0b11, 0b01): -1, (0b01, 0b00): -1,
    }

    def __init__(self, chip, pin_a, pin_b, cpr=200):
        self.chip = chip
        self.pin_a = pin_a
        self.pin_b = pin_b
        self.cpr = cpr
        self.counts_per_rev = cpr * 4
        self.position = 0
        self._running = True

        lgpio.gpio_claim_input(self.chip, self.pin_a)
        lgpio.gpio_claim_input(self.chip, self.pin_b)

        a = lgpio.gpio_read(self.chip, self.pin_a)
        b = lgpio.gpio_read(self.chip, self.pin_b)
        self._state = ((a & 1) << 1) | (b & 1)

        # Sprawdź, czy lgpio ma funkcję alertów
        if hasattr(lgpio, "gpio_set_alert_func"):
            self._mode = "alert"
            lgpio.gpio_set_alert_func(self.chip, self.pin_a, self._callback)
            lgpio.gpio_set_alert_func(self.chip, self.pin_b, self._callback)
            print("✅ Enkoder E2 działa w trybie ALERT (przerwania). CPR =", self.cpr)
        else:
            self._mode = "polling"
            self.thread = threading.Thread(target=self._poll_loop, daemon=True)
            self.thread.start()
            print("✅ Enkoder E2 działa w trybie POLLING (bez przerwań). CPR =", self.cpr)

    # --- Tryb ALERT (jeśli dostępny) ---
    def _callback(self, chip, gpio, level, tick):
        a = lgpio.gpio_read(self.chip, self.pin_a) & 1
        b = lgpio.gpio_read(self.chip, self.pin_b) & 1
        new_state = (a << 1) | b
        delta = self._transitions.get((self._state, new_state), 0)
        self.position += delta
        self._state = new_state

    # --- Tryb POLLING ---
    def _poll_loop(self):
        while self._running:
            a = lgpio.gpio_read(self.chip, self.pin_a) & 1
            b = lgpio.gpio_read(self.chip, self.pin_b) & 1
            new_state = (a << 1) | b
            delta = self._transitions.get((self._state, new_state), 0)
            self.position += delta
            self._state = new_state
            time.sleep(0.0005)  # 0.5 ms (2 kHz)

    def get_angle(self):
        return round((self.position / self.counts_per_rev) * 360.0, 2)

    def stop(self):
        self._running = False
        if self._mode == "polling":
            self.thread.join()


# -------------------------------
#   Klasa sterowania silnikami (TMC2209)
# -------------------------------
class StepperController:
    def __init__(self):
        self.chip = lgpio.gpiochip_open(0)

        # --- TMC2209: PAN ---
        self.pan_step = 23
        self.pan_dir  = 24
        self.pan_en   = 18

        # --- TMC2209: TILT ---
        self.tilt_step = 26
        self.tilt_dir  = 20
        self.tilt_en   = 16

        for pin in [self.pan_step, self.pan_dir, self.pan_en,
                    self.tilt_step, self.tilt_dir, self.tilt_en]:
            lgpio.gpio_claim_output(self.chip, pin)

        # Wyłącz silniki na start (EN=1)
        lgpio.gpio_write(self.chip, self.pan_en, 1)
        lgpio.gpio_write(self.chip, self.tilt_en, 1)

        # Enkodery
        self.encoder_pan = OpticalEncoderE2(self.chip, pin_a=17, pin_b=27, cpr=200)
        self.encoder_tilt = AS5600(bus_nr=13)

        # Wątek pokazujący kąty
        self.stop_event = threading.Event()
        self.kat_thread = threading.Thread(target=self._pokaz_katy, daemon=True)
        self.kat_thread.start()

        print("✅ StepperController gotowy – TMC2209 + E2(PAN) + AS5600(TILT)")

    # --- Ruch silnika krokowego ---
    def _step_motor(self, step_pin, dir_pin, en_pin, direction, steps, speed_khz):
        period = 1 / (speed_khz * 1000)
        lgpio.gpio_write(self.chip, en_pin, 0)
        lgpio.gpio_write(self.chip, dir_pin, direction)
        for _ in range(steps):
            lgpio.gpio_write(self.chip, step_pin, 1)
            time.sleep(period)
            lgpio.gpio_write(self.chip, step_pin, 0)
            time.sleep(period)
        lgpio.gpio_write(self.chip, en_pin, 1)

    def move_pan(self, direction, steps, speed_khz=2):
        print(f"\n➡️ Sterowany silnik: PAN | Kierunek: {'prawo' if direction == 1 else 'lewo'} | Kroki: {steps}")
        self._step_motor(self.pan_step, self.pan_dir, self.pan_en, direction, steps, speed_khz)

    def move_tilt(self, direction, steps, speed_khz=2):
        print(f"\n⬆️ Sterowany silnik: TILT | Kierunek: {'góra' if direction == 1 else 'dół'} | Kroki: {steps}")
        self._step_motor(self.tilt_step, self.tilt_dir, self.tilt_en, direction, steps, speed_khz)

    # --- Wyświetlanie kątów ---
    def _pokaz_katy(self):
        while not self.stop_event.is_set():
            kat_pan = self.encoder_pan.get_angle()
            kat_tilt = self.encoder_tilt.read_relative_angle()
            tilt_txt = f"{kat_tilt:.2f}°" if kat_tilt is not None else "—"
            print(f"PAN: {kat_pan:.2f}° | TILT: {tilt_txt}", end='\r')
            time.sleep(0.05)

    def stop(self):
        self.stop_event.set()
        self.kat_thread.join()
        self.encoder_pan.stop()
        lgpio.gpiochip_close(self.chip)
        print("\n🛑 Silniki zatrzymane i GPIO zwolnione.")


# -------------------------------
#   Test
# -------------------------------
if __name__ == "__main__":
    sc = StepperController()
    try:
        sc.move_pan(0, 600, 2)
        time.sleep(1)
        sc.move_tilt(0, 600, 2)
        time.sleep(1)
        sc.move_pan(1, 600, 2)
        time.sleep(1)
        sc.move_tilt(1, 600, 2)
        time.sleep(1)
    except KeyboardInterrupt:
        pass
    finally:
        sc.stop()
