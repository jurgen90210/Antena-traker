ANTENNA TRACKER RPi 5 (bez Pico W)
==================================

Sprzęt:
- Raspberry Pi 5
- 2x AS5600 (enkodery): I2C-1 (GPIO2/3) i I2C-3 (GPIO4/5)
- IMU 10DoF (MPU9255 + BMP280): I2C-4 (GPIO8/9)
- 2x TMC2208 V3.0: PAN (GPIO18,23,24), TILT (GPIO16,26,20)
- Krańcówki: PAN GPIO6, TILT GPIO13 (pull-up włączony programowo)
- Zasilanie VMOT: 12V (wspólna masa z RPi)

Konfiguracja systemu (RPi 5):
-----------------------------
Edytuj /boot/firmware/config.txt i dodaj linie:
    dtparam=i2c_arm=on
    dtoverlay=i2c3,pins_4_5
    dtoverlay=i2c4,pins_8_9

Następnie zrestartuj urządzenie.

Instalacja zależności:
----------------------
    sudo apt update
    sudo apt install -y python3-pip python3-smbus i2c-tools python3-pigpio
    sudo systemctl enable pigpiod
    sudo systemctl start pigpiod

    pip install -r requirements.txt

Test magistral I2C:
-------------------
    sudo i2cdetect -y 1   # AS5600 PAN -> adres 0x36
    sudo i2cdetect -y 3   # AS5600 TILT -> adres 0x36
    sudo i2cdetect -y 4   # IMU/BMP -> 0x68 (MPU), 0x76 (BMP)

Uruchomienie:
-------------
    python3 main_tracker.py

Uwaga:
------
- Domyślne opóźnienie kroków w stepper_control.py to 0.001 s.
  Dostosuj w zależności od mechaniki i mikrokroków.
- Jeśli pigpio nie łączy się, uruchom:
    sudo pigpiod

Miłej pracy!
