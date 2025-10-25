import time
from imu_sensor import IMUSensor
from as5600_sensor import AS5600
from stepper_control import StepperController

# =========================================================
#  KONFIGURACJA SYSTEMU ANTENY (RPi 5)
# =========================================================
# IMU (MPU9255 + BMP280) – I2C-14 (GPIO8=SDA, GPIO9=SCL)
# AS5600 (TILT) – I2C-13 (GPIO0=SDA, GPIO1=SCL)
# OPTYCZNY czujnik PAN – obsługiwany w StepperController
# Silniki krokowe (TMC2208): GPIO zgodne z Twoją konfiguracją

print("🛰️  Uruchamianie systemu Antenna Tracker – Raspberry Pi 5")
time.sleep(1)

# --- Inicjalizacja IMU i BMP280 ---
try:
    imu = IMUSensor(bus=1)
except Exception as e:
    print(f"⚠️  Błąd inicjalizacji IMU: {e}")
    imu = None

# --- Inicjalizacja AS5600 (TILT) ---
try:
    as_tilt = AS5600(bus_nr=13)
except Exception as e:
    print(f"⚠️  Błąd inicjalizacji AS5600 (TILT): {e}")
    as_tilt = None

# --- Inicjalizacja sterowania silnikami i czujnika optycznego ---
try:
    steppers = StepperController()  # Zawiera również obsługę czujnika optycznego (PAN)
except Exception as e:
    print(f"⚠️  Błąd inicjalizacji silników lub czujnika optycznego: {e}")
    steppers = None

print("✅ System gotowy do pracy.\n")

# =========================================================
#  PĘTLA GŁÓWNA TRACKERA
# =========================================================
try:
    while True:
        # --- Kąt PAN z czujnika optycznego ---
        if steppers:
            try:
                pan = steppers.get_optical_angle()
            except Exception:
                pan = 0.0
        else:
            pan = 0.0

        # --- Kąt TILT z AS5600 ---
        tilt = as_tilt.relative() if as_tilt else 0.0

        # --- Dane z IMU + BMP280 ---
        if imu:
            dane = imu.read_all()
            pitch = dane.get("pitch", 0.0)
            roll = dane.get("roll", 0.0)
            yaw = dane.get("yaw", 0.0)
            temp = dane.get("temperature", 0.0)
            press = dane.get("pressure", 0.0)
            alt = dane.get("altitude", 0.0)
        else:
            pitch = roll = yaw = temp = press = alt = 0.0

        # --- Status systemu ---
        print(
            f"PAN={pan:6.2f}° | TILT={tilt:6.2f}° | "
            f"PITCH={pitch:6.2f}° | ROLL={roll:6.2f}° | YAW={yaw:6.2f}° | "
            f"T={temp:4.1f}°C | P={press:7.1f} hPa | ALT={alt:6.1f} m     ",
            end="\r"
        )

        time.sleep(0.1)

except KeyboardInterrupt:
    print("\n🛑 Zatrzymano system Antenna Tracker.")
finally:
    if steppers:
        steppers.stop()
