#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import time
import math
import struct
import threading
import lgpio
from smbus2 import SMBus
from mpu9250_jmdev.mpu_9250 import MPU9250
from mpu9250_jmdev.registers import *

# =========================================================
#  AS5600 (TILT absolute)
# =========================================================
RAW_ANGLE_HIGH = 0x0C
RAW_ANGLE_LOW  = 0x0D

class AS5600:
    def __init__(self, bus_nr):
        self.bus = SMBus(bus_nr)
        self.kat_startowy = None

    def read_raw_angle(self):
        try:
            high = self.bus.read_byte_data(0x36, RAW_ANGLE_HIGH)
            low  = self.bus.read_byte_data(0x36, RAW_ANGLE_LOW)
            raw  = (high << 8) | low
            angle_deg = (raw & 0x0FFF) * 360.0 / 4096.0
            return round(angle_deg, 2)
        except Exception as e:
            print(f"Błąd odczytu AS5600: {e}")
            return None

    def set_zero(self):
        kat = self.read_raw_angle()
        if kat is not None:
            self.kat_startowy = kat
            print(f"Pozycja startowa TILT ustawiona jako zero: {self.kat_startowy:.2f}°")

    def read_relative_angle(self):
        if self.kat_startowy is None:
            self.set_zero()
        kat_biezacy = self.read_raw_angle()
        if kat_biezacy is None:
            return None
        kat_wzgledny = (kat_biezacy - self.kat_startowy) % 360
        return round(kat_wzgledny, 2)

# =========================================================
#  Enkoder kwadraturowy E2 (PAN)
# =========================================================
class OpticalEncoderE2:
    """Obsługa enkodera kwadraturowego x4, automatyczny wybór trybu alert/polling."""
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

    def _callback(self, chip, gpio, level, tick):
        a = lgpio.gpio_read(self.chip, self.pin_a) & 1
        b = lgpio.gpio_read(self.chip, self.pin_b) & 1
        new_state = (a << 1) | b
        delta = self._transitions.get((self._state, new_state), 0)
        self.position += delta
        self._state = new_state

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
        if getattr(self, "_mode", "") == "polling":
            self.thread.join()

# =========================================================
#  BMP280 (temp/press/alt)
# =========================================================
class BMP280:
    def __init__(self, bus=1, address=None):
        self.i2c = SMBus(bus)
        if address is None:
            for addr in [0x76, 0x77]:
                try:
                    self.i2c.read_byte_data(addr, 0xD0)
                    address = addr
                    break
                except Exception:
                    continue
        if address is None:
            raise OSError("BMP280 not found on I2C bus.")
        self.address = address

        chip_id = self.i2c.read_byte_data(self.address, 0xD0)
        if chip_id != 0x58:
            raise OSError(f"Unexpected chip ID: 0x{chip_id:02X} (expected 0x58)")

        # Soft reset
        self.i2c.write_byte_data(self.address, 0xE0, 0xB6)
        time.sleep(0.2)

        # Calibration data
        calib = self.i2c.read_i2c_block_data(self.address, 0x88, 24)
        self.dig_T1, self.dig_T2, self.dig_T3, \
        self.dig_P1, self.dig_P2, self.dig_P3, self.dig_P4, \
        self.dig_P5, self.dig_P6, self.dig_P7, self.dig_P8, self.dig_P9 = struct.unpack("<HhhHhhhhhhhh", bytes(calib))
        self.t_fine = 0

        # Config: temp x2, press x8, normal mode; standby 250ms, IIR filter x2
        self.i2c.write_byte_data(self.address, 0xF4, 0x57)
        self.i2c.write_byte_data(self.address, 0xF5, 0x10)
        time.sleep(0.1)

        print(f"✅ BMP280 zainicjalizowany (adres 0x{self.address:02X})")

    def _read_raw_temp(self):
        data = self.i2c.read_i2c_block_data(self.address, 0xFA, 3)
        return (data[0] << 12) | (data[1] << 4) | (data[2] >> 4)

    def _read_raw_press(self):
        data = self.i2c.read_i2c_block_data(self.address, 0xF7, 3)
        return (data[0] << 12) | (data[1] << 4) | (data[2] >> 4)

    def _compensate_temp(self, adc_T):
        var1 = (adc_T / 16384.0 - self.dig_T1 / 1024.0) * self.dig_T2
        var2 = ((adc_T / 131072.0 - self.dig_T1 / 8192.0) ** 2) * self.dig_T3
        self.t_fine = int(var1 + var2)
        return (var1 + var2) / 5120.0

    def _compensate_press(self, adc_P):
        var1 = self.t_fine / 2.0 - 64000.0
        var2 = var1 * var1 * self.dig_P6 / 32768.0
        var2 = var2 + var1 * self.dig_P5 * 2.0
        var2 = var2 / 4.0 + self.dig_P4 * 65536.0
        var1 = (self.dig_P3 * var1 * var1 / 524288.0 + self.dig_P2 * var1) / 524288.0
        var1 = (1.0 + var1 / 32768.0) * self.dig_P1
        if var1 == 0:
            return 0
        p = 1048576.0 - adc_P
        p = ((p - var2 / 4096.0) * 6250.0) / var1
        var1 = self.dig_P9 * p * p / 2147483648.0
        var2 = p * self.dig_P8 / 32768.0
        return (p + (var1 + var2 + self.dig_P7) / 16.0) / 100.0

    @property
    def temperature(self):
        adc_T = self._read_raw_temp()
        return round(self._compensate_temp(adc_T), 2)

    @property
    def pressure(self):
        adc_P = self._read_raw_press()
        return round(self._compensate_press(adc_P), 2)

    @property
    def altitude(self):
        p = self.pressure
        return round(44330.0 * (1.0 - (p / 1013.25) ** (1.0 / 5.255)), 2)

# =========================================================
#  Madgwick (IMU)
# =========================================================
class MadgwickAHRS:
    def __init__(self, sampleperiod=0.05, beta=0.1):
        self.sampleperiod = sampleperiod
        self.beta = beta
        self.q = [1, 0, 0, 0]

    def update_imu(self, gyro, accel):
        gx, gy, gz = gyro
        ax, ay, az = accel
        q1, q2, q3, q4 = self.q

        norm = math.sqrt(ax * ax + ay * ay + az * az)
        if norm == 0:
            return
        ax, ay, az = ax / norm, ay / norm, az / norm

        _2q1 = 2 * q1; _2q2 = 2 * q2; _2q3 = 2 * q3; _2q4 = 2 * q4
        _4q1 = 4 * q1; _4q2 = 4 * q2; _4q3 = 4 * q3
        _8q2 = 8 * q2; _8q3 = 8 * q3
        q1q1 = q1*q1; q2q2 = q2*q2; q3q3 = q3*q3; q4q4 = q4*q4

        s1 = _4q1*q3q3 + _2q3*ax + _4q1*q2q2 - _2q2*ay
        s2 = _4q2*q4q4 - _2q4*ax + 4*q1q1*q2 - _2q1*ay - _4q2 + _8q2*q2q2 + _8q2*q3q3 + _4q2*az
        s3 = 4*q1q1*q3 + _2q1*ax + _4q3*q4q4 - _2q4*ay - _4q3 + _8q3*q2q2 + _8q3*q3q3 + _4q3*az
        s4 = 4*q2q2*q4 - _2q2*ax + 4*q3q3*q4 - _2q3*ay

        norm_s = math.sqrt(s1*s1 + s2*s2 + s3*s3 + s4*s4)
        if norm_s == 0:
            return
        s1, s2, s3, s4 = s1/norm_s, s2/norm_s, s3/norm_s, s4/norm_s

        q_dot1 = 0.5 * (-q2*gx - q3*gy - q4*gz) - self.beta*s1
        q_dot2 = 0.5 * (q1*gx + q3*gz - q4*gy) - self.beta*s2
        q_dot3 = 0.5 * (q1*gy - q2*gz + q4*gx) - self.beta*s3
        q_dot4 = 0.5 * (q1*gz + q2*gy - q3*gx) - self.beta*s4

        q1 += q_dot1 * self.sampleperiod
        q2 += q_dot2 * self.sampleperiod
        q3 += q_dot3 * self.sampleperiod
        q4 += q_dot4 * self.sampleperiod

        norm_q = math.sqrt(q1*q1 + q2*q2 + q3*q3 + q4*q4)
        self.q = [q1/norm_q, q2/norm_q, q3/norm_q, q4/norm_q]

    @property
    def quaternion(self):
        return self.q

# =========================================================
#  IMU Sensor (MPU9250 + BMP280 + Madgwick)
# =========================================================
class IMUSensor:
    def __init__(self, bus=1):
        self.mpu = MPU9250(bus=bus, address_ak=0x0C)
        self.mpu.configure()
        print("✅ MPU9250 zainicjalizowany")
        self.bmp = BMP280(bus)
        self.filter = MadgwickAHRS(sampleperiod=0.05, beta=0.1)

    def read_all(self):
        accel = self.mpu.readAccelerometerMaster()               # g
        gyro_dps = self.mpu.readGyroscopeMaster()                # °/s
        gyro = [math.radians(g) for g in gyro_dps]               # rad/s
        # mag = self.mpu.readMagnetometerMaster()                # dostępny, nieużywany w tej wersji

        self.filter.update_imu(gyro, accel)
        w, x, y, z = self.filter.quaternion

        pitch = math.degrees(math.asin(2.0 * (w*y - z*x)))
        roll  = math.degrees(math.atan2(2.0 * (w*x + y*z), 1.0 - 2.0 * (x*x + y*y)))
        yaw   = math.degrees(math.atan2(2.0 * (w*z + x*y), 1.0 - 2.0 * (y*y + z*z)))

        temp = self.bmp.temperature
        press = self.bmp.pressure
        alt = self.bmp.altitude

        return {
            "pitch": round(pitch, 2),
            "roll": round(roll, 2),
            "yaw": round(yaw, 2),
            "temperature": temp,
            "pressure": press,
            "altitude": alt
        }

# =========================================================
#  Sterownik krokowców (TMC2209) + sekwencja ruchu
# =========================================================
CW  = 1  # zgodnie z zegarem (zmień na 0 jeśli kierunek odwrotny)
CCW = 0

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

        # Wyłącz silniki na start (EN=1, TMC2209 aktywne niskim stanem)
        lgpio.gpio_write(self.chip, self.pan_en, 1)
        lgpio.gpio_write(self.chip, self.tilt_en, 1)

        # Enkodery
        self.encoder_pan  = OpticalEncoderE2(self.chip, pin_a=17, pin_b=27, cpr=200)
        self.encoder_tilt = AS5600(bus_nr=13)  # zostawiam bus 13 jak w Twoim kodzie

        # IMU
        self.imu = IMUSensor(bus=1)

        # Wspólne dane do wyświetlania
        self._telemetry = {
            "pan_deg": 0.0, "tilt_deg": None,
            "pitch": 0.0, "roll": 0.0, "yaw": 0.0,
            "temperature": 0.0, "pressure": 0.0, "altitude": 0.0
        }
        self._lock = threading.Lock()
        self._stop_event = threading.Event()

        # Wątek IMU
        self._imu_thread = threading.Thread(target=self._imu_loop, daemon=True)
        self._imu_thread.start()

        # Wątek wyświetlania
        self._disp_thread = threading.Thread(target=self._display_loop, daemon=True)
        self._disp_thread.start()

        print("✅ StepperController gotowy – TMC2209 + E2(PAN) + AS5600(TILT) + IMU(10DoF)")

    # --- Silniki ---
    def _step_motor(self, step_pin, dir_pin, en_pin, direction, steps, speed_khz):
        period = 1.0 / (speed_khz * 1000.0)  # półokres
        lgpio.gpio_write(self.chip, en_pin, 0)      # EN low = enable
        lgpio.gpio_write(self.chip, dir_pin, direction)
        for _ in range(steps):
            lgpio.gpio_write(self.chip, step_pin, 1)
            time.sleep(period)
            lgpio.gpio_write(self.chip, step_pin, 0)
            time.sleep(period)
        lgpio.gpio_write(self.chip, en_pin, 1)      # disable po ruchu

    def move_pan(self, direction, steps, speed_khz=2.0):
        print(f"\n➡️ PAN | Kierunek: {'CW' if direction==CW else 'CCW'} | Kroki: {steps}")
        self._step_motor(self.pan_step, self.pan_dir, self.pan_en, direction, steps, speed_khz)

    def move_tilt(self, direction, steps, speed_khz=2.0):
        print(f"\n⬆️ TILT | Kierunek: {'CW' if direction==CW else 'CCW'} | Kroki: {steps}")
        self._step_motor(self.tilt_step, self.tilt_dir, self.tilt_en, direction, steps, speed_khz)

    # --- IMU loop ---
    def _imu_loop(self):
        # Ustaw zero dla TILT (AS5600) przy starcie
        self.encoder_tilt.set_zero()
        while not self._stop_event.is_set():
            try:
                d = self.imu.read_all()
                pan = self.encoder_pan.get_angle()
                tilt = self.encoder_tilt.read_relative_angle()

                with self._lock:
                    self._telemetry.update({
                        "pan_deg": pan,
                        "tilt_deg": tilt,
                        "pitch": d["pitch"], "roll": d["roll"], "yaw": d["yaw"],
                        "temperature": d["temperature"], "pressure": d["pressure"], "altitude": d["altitude"]
                    })
            except Exception as e:
                # nie zatrzymujemy pracy — tylko log
                print(f"\n[IMU] Błąd odczytu: {e}")
            time.sleep(0.05)  # 20 Hz

    # --- Display loop ---
    def _display_loop(self):
        while not self._stop_event.is_set():
            with self._lock:
                pan = self._telemetry["pan_deg"]
                tilt = self._telemetry["tilt_deg"]
                pitch = self._telemetry["pitch"]
                roll  = self._telemetry["roll"]
                yaw   = self._telemetry["yaw"]
                t     = self._telemetry["temperature"]
                p     = self._telemetry["pressure"]
                a     = self._telemetry["altitude"]
            tilt_txt = f"{tilt:.2f}°" if tilt is not None else "—"
            line = (
                f"PAN: {pan:7.2f}° | TILT: {tilt_txt:>7} | "
                f"Pitch: {pitch:7.2f}° | Roll: {roll:7.2f}° | Yaw: {yaw:7.2f}° | "
                f"T: {t:5.2f}°C | P: {p:7.2f} hPa | Alt: {a:7.2f} m"
            )
            print(line, end="\r", flush=True)
            time.sleep(0.05)  # 20 Hz

    # --- Sekwencja z zadania ---
    def run_sequence(self, pan_steps=300, tilt_steps=200, speed_khz=2.0):
        # PAN: 300 CW -> powrót
        self.move_pan(CW, pan_steps, speed_khz)
        time.sleep(0.5)
        self.move_pan(CCW, pan_steps, speed_khz)
        time.sleep(0.8)

        # TILT: 200 CW -> powrót
        self.move_tilt(CW, tilt_steps, speed_khz)
        time.sleep(0.5)
        self.move_tilt(CCW, tilt_steps, speed_khz)
        time.sleep(0.8)

    def stop(self):
        self._stop_event.set()
        try:
            self._imu_thread.join(timeout=1.0)
            self._disp_thread.join(timeout=1.0)
        except Exception:
            pass
        self.encoder_pan.stop()
        try:
            lgpio.gpio_write(self.chip, self.pan_en, 1)
            lgpio.gpio_write(self.chip, self.tilt_en, 1)
        except Exception:
            pass
        lgpio.gpiochip_close(self.chip)
        print("\n🛑 Zatrzymano wątki, wyłączono silniki i zwolniono GPIO.")

# =========================================================
#  Main / Test
# =========================================================
if __name__ == "__main__":
    sc = StepperController()
    try:
        sc.run_sequence(pan_steps=3300, tilt_steps=3200, speed_khz=2.0)
        # Możesz powtórzyć sekwencję lub dodać pętlę:
        # for _ in range(3):
        #     sc.run_sequence(300, 200, 2.0)
        #     time.sleep(1.0)
        time.sleep(1.0)
    except KeyboardInterrupt:
        pass
    finally:
        sc.stop()
