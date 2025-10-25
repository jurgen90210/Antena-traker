# limit_switches.py
import pigpio

class LimitSwitches:
    def __init__(self, pan_pin=6, tilt_pin=13):
        self.pi = pigpio.pi()
        if not self.pi.connected:
            raise RuntimeError("Nie można połączyć się z demone pigpio. Uruchom: sudo systemctl start pigpiod")
        self.pan_pin = pan_pin
        self.tilt_pin = tilt_pin
        for p in [self.pan_pin, self.tilt_pin]:
            self.pi.set_mode(p, pigpio.INPUT)
            self.pi.set_pull_up_down(p, pigpio.PUD_UP)  # włącz pull-up

    def pan_triggered(self):
        return not self.pi.read(self.pan_pin)  # aktywne po zwarciu do GND

    def tilt_triggered(self):
        return not self.pi.read(self.tilt_pin)
