#!/usr/bin/env python3
import serial
import pynmea2
from datetime import datetime

PORT = "/dev/ttyACM0"   # <-- ZMIEŃ jeśli potrzeba, np. "/dev/ttyUSB0"
BAUD = 9600             # typowo dla u-blox 6

def nice(v, nd=6):
    return f"{v:.{nd}f}" if isinstance(v, (int, float)) else str(v)

def main():
    print(f"[{datetime.now().isoformat(timespec='seconds')}] Start, port={PORT}, baud={BAUD}")
    print("Naciśnij Ctrl+C, aby zakończyć.\n")

    # timeout ułatwia łagodne przerwanie pętlą
    with serial.Serial(PORT, BAUD, timeout=1) as ser:
        buffer = b""
        while True:
            try:
                line = ser.readline().decode(errors="ignore").strip()
                if not line or not line.startswith("$"):
                    continue

                # u-blox może wysyłać GPRMC/GNRMC, GPGGA/GNGGA itd.
                # Próbujemy sparsować wszystko, ale do pozycji wystarczą RMC lub GGA.
                try:
                    msg = pynmea2.parse(line)
                except pynmea2.ParseError:
                    continue

                # Preferuj RMC (ma status fixu i prędkość), w drugiej kolejności GGA.
                if isinstance(msg, pynmea2.RMC):  # Recommended Minimum
                    if msg.status and msg.status.upper() == "A":  # A=Active, V=Void
                        lat = msg.latitude
                        lon = msg.longitude
                        spd_knots = msg.spd_over_grnd or 0.0
                        spd_kmh = float(spd_knots) * 1.852 if spd_knots else 0.0
                        cog = msg.true_course
                        tstamp = msg.datetime.isoformat() if msg.datetime else "—"
                        print(f"RMC  lat={nice(lat,6)}  lon={nice(lon,6)}  v={nice(spd_kmh,2)} km/h  kurs={cog or '—'}°  czas={tstamp}")
                elif isinstance(msg, pynmea2.GGA):  # Fix Data
                    lat = msg.latitude
                    lon = msg.longitude
                    qual = int(msg.gps_qual or 0)    # 0=no fix, 1=GPS, 2=DGPS, itd.
                    sats = int(msg.num_sats or 0)
                    alt  = float(msg.altitude or 0.0)
                    print(f"GGA  lat={nice(lat,6)}  lon={nice(lon,6)}  alt={nice(alt,1)} m  fix={qual}  sats={sats}")
                # Inne zdania (GSA/GSV/VTG) możesz też obsłużyć, ale nie są konieczne
            except KeyboardInterrupt:
                print("\nZakończono.")
                break
            except serial.SerialException as e:
                print(f"[BŁĄD portu] {e}")
                break
            except Exception as e:
                # Złap niespodzianki, ale kontynuuj
                print(f"[WARN] {e}")

if __name__ == "__main__":
    main()
