#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import time
from smbus2 import SMBus

AS5600_ADDR = 0x36
RAW_ANGLE_HIGH = 0x0C
RAW_ANGLE_LOW = 0x0D

def read_angle(bus):
    """Odczytuje kąt w stopniach z czujnika AS5600"""
    high = bus.read_byte_data(AS5600_ADDR, RAW_ANGLE_HIGH)
    low = bus.read_byte_data(AS5600_ADDR, RAW_ANGLE_LOW)
    raw = ((high << 8) | low) & 0x0FFF
    return round(raw * 360.0 / 4096.0, 2)

def detect_bus():
    """Automatyczne wykrycie, czy AS5600 jest na I2C-1 lub I2C-13"""
    for bus_nr in [1, 13]:
        try:
            bus = SMBus(bus_nr)
            # próba odczytu jednego bajtu
            bus.read_byte_data(AS5600_ADDR, RAW_ANGLE_HIGH)
            print(f"✅ Wykryto czujnik AS5600 na magistrali I2C-{bus_nr}")
            return bus_nr
        except FileNotFoundError:
            print(f"❌ Magistrala I2C-{bus_nr} nie istnieje (nieaktywna w systemie)")
        except OSError:
            print(f"⚠️  Brak odpowiedzi z AS5600 na I2C-{bus_nr}")
        time.sleep(0.3)
    return None

if __name__ == "__main__":
    print("🔍 Automatyczne wykrywanie czujnika AS5600 na I²C-1 lub I²C-3 (bus 13)...\n")

    bus_nr = detect_bus()

    if bus_nr is None:
        print("\n❌ Nie wykryto czujnika AS5600 na żadnej z magistral (I2C-1 ani I2C-13).")
        print("👉 Sprawdź połączenia lub adres (domyślnie 0x36).")
        exit(1)

    try:
        bus = SMBus(bus_nr)
        print(f"\n✅ Test czujnika AS5600 rozpoczęty (I2C-{bus_nr}, adres 0x36)\n"
              f"➡️  Obracaj magnes, wartości powinny zmieniać się płynnie.\n"
              f"➡️  Naciśnij Ctrl+C, aby zakończyć.\n")

        while True:
            try:
                angle = read_angle(bus)
                print(f"Kąt magnetyczny: {angle:7.2f}°", end="\r", flush=True)
                time.sleep(0.1)
            except OSError:
                print("⚠️  Błąd odczytu – czujnik nie odpowiada!", end="\r")
                time.sleep(0.5)

    except KeyboardInterrupt:
        print("\n🛑 Zakończono test.")
