# as5600_sensor.py
from smbus2 import SMBus

AS5600_ADDR = 0x36
RAW_ANGLE_HIGH = 0x0C
RAW_ANGLE_LOW  = 0x0D

class AS5600:
    def __init__(self, bus_nr):
        self.bus = SMBus(bus_nr)
        self.kat_startowy = None

    def read_angle(self):
        high = self.bus.read_byte_data(AS5600_ADDR, RAW_ANGLE_HIGH)
        low  = self.bus.read_byte_data(AS5600_ADDR, RAW_ANGLE_LOW)
        raw = (high << 8) | low
        angle = (raw & 0x0FFF) * 360.0 / 4096.0
        return angle

    def zero(self):
        self.kat_startowy = self.read_angle()

    def relative(self):
        if self.kat_startowy is None:
            self.zero()
        angle = (self.read_angle() - self.kat_startowy) % 360.0
        return round(angle, 2)
