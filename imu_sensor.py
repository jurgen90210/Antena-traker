import time, math, struct
from smbus2 import SMBus
from mpu9250_jmdev.mpu_9250 import MPU9250
from mpu9250_jmdev.registers import *

# =========================================================
#  Klasa BMP280 — przepisana z Twojej wersji Pico, ale dla SMBus (RPi)
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
#  Filtr Madgwicka (Pythonowa wersja)
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

        _2q1 = 2 * q1
        _2q2 = 2 * q2
        _2q3 = 2 * q3
        _2q4 = 2 * q4
        _4q1 = 4 * q1
        _4q2 = 4 * q2
        _4q3 = 4 * q3
        _8q2 = 8 * q2
        _8q3 = 8 * q3
        q1q1 = q1 * q1
        q2q2 = q2 * q2
        q3q3 = q3 * q3
        q4q4 = q4 * q4

        s1 = _4q1 * q3q3 + _2q3 * ax + _4q1 * q2q2 - _2q2 * ay
        s2 = _4q2 * q4q4 - _2q4 * ax + 4 * q1q1 * q2 - _2q1 * ay - _4q2 + _8q2 * q2q2 + _8q2 * q3q3 + _4q2 * az
        s3 = 4 * q1q1 * q3 + _2q1 * ax + _4q3 * q4q4 - _2q4 * ay - _4q3 + _8q3 * q2q2 + _8q3 * q3q3 + _4q3 * az
        s4 = 4 * q2q2 * q4 - _2q2 * ax + 4 * q3q3 * q4 - _2q3 * ay

        norm_s = math.sqrt(s1 * s1 + s2 * s2 + s3 * s3 + s4 * s4)
        if norm_s == 0:
            return
        s1, s2, s3, s4 = s1 / norm_s, s2 / norm_s, s3 / norm_s, s4 / norm_s

        q_dot1 = 0.5 * (-q2 * gx - q3 * gy - q4 * gz) - self.beta * s1
        q_dot2 = 0.5 * (q1 * gx + q3 * gz - q4 * gy) - self.beta * s2
        q_dot3 = 0.5 * (q1 * gy - q2 * gz + q4 * gx) - self.beta * s3
        q_dot4 = 0.5 * (q1 * gz + q2 * gy - q3 * gx) - self.beta * s4

        q1 += q_dot1 * self.sampleperiod
        q2 += q_dot2 * self.sampleperiod
        q3 += q_dot3 * self.sampleperiod
        q4 += q_dot4 * self.sampleperiod

        norm_q = math.sqrt(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4)
        self.q = [q1 / norm_q, q2 / norm_q, q3 / norm_q, q4 / norm_q]

    @property
    def quaternion(self):
        return self.q


# =========================================================
#  Klasa główna: IMUSensor
# =========================================================
class IMUSensor:
    def __init__(self, bus=1):
        self.mpu = MPU9250(bus=bus, address_ak=0x0C)
        self.mpu.configure()
        print("✅ MPU9255 zainicjalizowany")
        self.bmp = BMP280(bus)
        self.filter = MadgwickAHRS(sampleperiod=0.05, beta=0.1)

    def read_all(self):
        accel = self.mpu.readAccelerometerMaster()
        gyro = [math.radians(g) for g in self.mpu.readGyroscopeMaster()]
        mag = self.mpu.readMagnetometerMaster()

        self.filter.update_imu(gyro, accel)
        w, x, y, z = self.filter.quaternion

        pitch = math.degrees(math.asin(2.0 * (w * y - z * x)))
        roll  = math.degrees(math.atan2(2.0 * (w * x + y * z),
                                        1.0 - 2.0 * (x * x + y * y)))
        yaw   = math.degrees(math.atan2(2.0 * (w * z + x * y),
                                        1.0 - 2.0 * (y * y + z * z)))

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

    def test_loop(self):
        print("▶️ Start testu IMU + Madgwick + BMP280 (Ctrl+C aby zakończyć)\n")
        try:
            while True:
                d = self.read_all()
                print(f"Pitch: {d['pitch']}° | Roll: {d['roll']}° | Yaw: {d['yaw']}° | "
                      f"Temp: {d['temperature']}°C | Press: {d['pressure']}hPa | Alt: {d['altitude']}m")
                time.sleep(0.05)
        except KeyboardInterrupt:
            print("\n🛑 Zakończono test.")


# --- Uruchomienie ---
if __name__ == "__main__":
    imu = IMUSensor(bus=1)
    imu.test_loop()
