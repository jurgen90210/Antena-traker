# imu_main.py
import time, math, struct
from smbus2 import SMBus
from mpu9250_jmdev.mpu_9250 import MPU9250
from mpu9250_jmdev.registers import *

# =========================================================
#  Klasa BMP280 — wersja z obsługą błędu I2C i retry
# =========================================================
class BMP280:
    def __init__(self, bus=1, address=None):
        self.i2c = SMBus(bus)
        self.address = None

        for attempt in range(3):
            try:
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

                # Calibration
                calib = self.i2c.read_i2c_block_data(self.address, 0x88, 24)
                self.dig_T1, self.dig_T2, self.dig_T3, \
                self.dig_P1, self.dig_P2, self.dig_P3, self.dig_P4, \
                self.dig_P5, self.dig_P6, self.dig_P7, self.dig_P8, self.dig_P9 = struct.unpack("<HhhHhhhhhhhh", bytes(calib))
                self.t_fine = 0

                # Normal mode, oversampling
                self.i2c.write_byte_data(self.address, 0xF4, 0x57)
                self.i2c.write_byte_data(self.address, 0xF5, 0x10)
                time.sleep(0.1)

                print(f"✅ BMP280 zainicjalizowany (adres 0x{self.address:02X})")
                return
            except Exception as e:
                print(f"⚠️  Błąd inicjalizacji BMP280 (próba {attempt+1}/3): {e}")
                time.sleep(0.5)
        raise OSError("Nie udało się zainicjalizować BMP280 po 3 próbach.")

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
#  Klasa główna IMUSensor (MPU9250 + BMP280 + Madgwick)
# =========================================================
class MadgwickAHRS:
    def __init__(self, sampleperiod=0.05, beta=0.1):
        self.sampleperiod = sampleperiod
        self.beta = beta
        self.q = [1, 0, 0, 0]

    def update_imu(self, gyro, accel):
        gx, gy, gz = gyro
        ax, ay, az = accel
        norm = math.sqrt(ax * ax + ay * ay + az * az)
        if norm == 0:
            return
        ax, ay, az = ax / norm, ay / norm, az / norm
        q1, q2, q3, q4 = self.q
        q_dot1 = 0.5 * (-q2 * gx - q3 * gy - q4 * gz)
        q_dot2 = 0.5 * (q1 * gx + q3 * gz - q4 * gy)
        q_dot3 = 0.5 * (q1 * gy - q2 * gz + q4 * gx)
        q_dot4 = 0.5 * (q1 * gz + q2 * gy - q3 * gx)
        q1 += q_dot1 * self.sampleperiod
        q2 += q_dot2 * self.sampleperiod
        q3 += q_dot3 * self.sampleperiod
        q4 += q_dot4 * self.sampleperiod
        norm_q = math.sqrt(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4)
        self.q = [q1 / norm_q, q2 / norm_q, q3 / norm_q, q4 / norm_q]

    @property
    def quaternion(self):
        return self.q


class IMUSensor:
    def __init__(self, bus=1):
        self.mpu = MPU9250(bus=bus, address_ak=0x0C)
        self.mpu.configure()
        print("✅ MPU9250 zainicjalizowany")
        self.bmp = BMP280(bus)
        self.filter = MadgwickAHRS(sampleperiod=0.05, beta=0.1)

    def read_all(self):
        accel = self.mpu.readAccelerometerMaster()
        gyro = [math.radians(g) for g in self.mpu.readGyroscopeMaster()]
        self.filter.update_imu(gyro, accel)
        w, x, y, z = self.filter.quaternion
        pitch = math.degrees(math.asin(2.0 * (w * y - z * x)))
        roll = math.degrees(math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y)))
        yaw = math.degrees(math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z)))
        return {
            "pitch": round(pitch, 2),
            "roll": round(roll, 2),
            "yaw": round(yaw, 2),
            "temperature": self.bmp.temperature,
            "pressure": self.bmp.pressure,
            "altitude": self.bmp.altitude
        }
