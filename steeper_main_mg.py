# steeper_main.py
import time
import threading
import lgpio
from smbus2 import SMBus

# =========================================================
#  Enkoder magnetyczny AS5600 (TILT)
# =========================================================
RAW_ANGLE_HIGH = 0x0C
RAW_ANGLE_LOW  = 0x0D

class AS5600:
    def __init__(self, bus_nr=1):
        self.bus = SMBus(bus_nr)
        self.kat_startowy = None

    def read_raw_angle(self):
        high = self.bus.read_byte_data(0x36, RAW_ANGLE_HIGH)
        low  = self.bus.read_byte_data(0x36, RAW_ANGLE_LOW)
        raw = (high << 8) | low
        return round((raw & 0x0FFF) * 360.0 / 4096.0, 2)

    def set_zero(self):
        self.kat_startowy = self.read_raw_angle()
        print(f"Zero TILT = {self.kat_startowy:.2f}°")

    def read_relative_angle(self):
        if self.kat_startowy is None:
            self.set_zero()
        kat = self.read_raw_angle()
        return round((kat - self.kat_startowy) % 360, 2)


# =========================================================
#  Enkoder optyczny E2 (PAN)
# =========================================================
class OpticalEncoderE2:
    _transitions = {
        (0b00, 0b01): +1, (0b01, 0b11): +1, (0b11, 0b10): +1, (0b10, 0b00): +1,
        (0b00, 0b10): -1, (0b10, 0b11): -1, (0b11, 0b01): -1, (0b01, 0b00): -1,
    }

    def __init__(self, chip, pin_a, pin_b, cpr=200):
        self.chip = chip
        self.pin_a = pin_a
        self.pin_b = pin_b
        self.cpr = cpr
        self.position = 0
        self.counts_per_rev = cpr * 4
        self._running = True

        lgpio.gpio_claim_input(chip, pin_a)
        lgpio.gpio_claim_input(chip, pin_b)
        a = lgpio.gpio_read(chip, pin_a)
        b = lgpio.gpio_read(chip, pin_b)
        self._state = ((a & 1) << 1) | (b & 1)

        if hasattr(lgpio, "gpio_set_alert_func"):
            lgpio.gpio_set_alert_func(chip, pin_a, self._callback)
            lgpio.gpio_set_alert_func(chip, pin_b, self._callback)
        else:
            threading.Thread(target=self._poll_loop, daemon=True).start()

    def _callback(self, chip, gpio, level, tick):
        a = lgpio.gpio_read(chip, self.pin_a) & 1
        b = lgpio.gpio_read(chip, self.pin_b) & 1
        new_state = (a << 1) | b
        self.position += self._transitions.get((self._state, new_state), 0)
        self._state = new_state

    def _poll_loop(self):
        while self._running:
            a = lgpio.gpio_read(self.chip, self.pin_a) & 1
            b = lgpio.gpio_read(self.chip, self.pin_b) & 1
            new_state = (a << 1) | b
            self.position += self._transitions.get((self._state, new_state), 0)
            self._state = new_state
            time.sleep(0.0005)

    def get_angle(self):
        return round((self.position / self.counts_per_rev) * 360.0, 2)

    def stop(self):
        self._running = False


# =========================================================
#  Sterownik silników krokowych (TMC2209)
# =========================================================
CW, CCW = 1, 0

class StepperController:
    def __init__(self):
        self.chip = lgpio.gpiochip_open(0)
        self.pan_step, self.pan_dir, self.pan_en = 23, 24, 18
        self.tilt_step, self.tilt_dir, self.tilt_en = 26, 20, 16

        for p in [self.pan_step, self.pan_dir, self.pan_en,
                  self.tilt_step, self.tilt_dir, self.tilt_en]:
            lgpio.gpio_claim_output(self.chip, p)
        lgpio.gpio_write(self.chip, self.pan_en, 1)
        lgpio.gpio_write(self.chip, self.tilt_en, 1)

        self.encoder_pan = OpticalEncoderE2(self.chip, 17, 27, 200)
        self.encoder_tilt = AS5600(13)
        self.encoder_tilt.set_zero()
        print("✅ StepperController gotowy")

    def _step_motor(self, step_pin, dir_pin, en_pin, direction, steps, speed=2):
        period = 1 / (speed * 1000)
        lgpio.gpio_write(self.chip, en_pin, 0)
        lgpio.gpio_write(self.chip, dir_pin, direction)
        for _ in range(steps):
            lgpio.gpio_write(self.chip, step_pin, 1)
            time.sleep(period)
            lgpio.gpio_write(self.chip, step_pin, 0)
            time.sleep(period)
        lgpio.gpio_write(self.chip, en_pin, 1)

    def move_pan(self, direction, steps):
        print(f"➡️ PAN: {'CW' if direction else 'CCW'}, {steps} kroków")
        self._step_motor(self.pan_step, self.pan_dir, self.pan_en, direction, steps)

    def move_tilt(self, direction, steps):
        print(f"⬆️ TILT: {'CW' if direction else 'CCW'}, {steps} kroków")
        self._step_motor(self.tilt_step, self.tilt_dir, self.tilt_en, direction, steps)

    def get_angles(self):
        return self.encoder_pan.get_angle(), self.encoder_tilt.read_relative_angle()

    def stop(self):
        self.encoder_pan.stop()
        lgpio.gpiochip_close(self.chip)
        print("🛑 GPIO zwolnione.")
