import serial
import pynmea2
import folium
import math
import time
import csv
import os
from datetime import datetime

# === KONFIGURACJA PORTU GPS ===
GPS_PORT = "/dev/ttyACM0"   # lub /dev/ttyUSB0
BAUD_RATE = 9600

# === FUNKCJE POMOCNICZE ===
def calculate_heading(lat1, lon1, lat2, lon2):
    """Oblicza kierunek (azymut) między dwoma punktami GPS"""
    if None in (lat1, lon1, lat2, lon2):
        return 0
    lat1, lon1, lat2, lon2 = map(math.radians, [lat1, lon1, lat2, lon2])
    dlon = lon2 - lon1
    x = math.sin(dlon) * math.cos(lat2)
    y = math.cos(lat1) * math.sin(lat2) - math.sin(lat1) * math.cos(lat2) * math.cos(dlon)
    bearing = math.degrees(math.atan2(x, y))
    return (bearing + 360) % 360


def create_map(lat, lon, heading, altitude, speed, sats, hdop, vdop, pdop, track_points):
    """Tworzy mapę z markerem, kompasem i śladem trasy"""
    m = folium.Map(location=[lat, lon], zoom_start=17, tiles="OpenStreetMap")

    if len(track_points) > 1:
        folium.PolyLine(track_points, color="blue", weight=3, opacity=0.7).add_to(m)

    folium.Marker(
        [lat, lon],
        popup=f"<b>Lat:</b> {lat:.6f}<br>"
              f"<b>Lon:</b> {lon:.6f}<br>"
              f"<b>Altitude:</b> {altitude:.1f} m<br>"
              f"<b>Speed:</b> {speed:.2f} km/h<br>"
              f"<b>Satellites:</b> {sats}<br>"
              f"<b>HDOP:</b> {hdop}<br>"
              f"<b>VDOP:</b> {vdop}<br>"
              f"<b>PDOP:</b> {pdop}<br>"
              f"<b>Heading:</b> {heading:.1f}°",
        icon=folium.Icon(color='red', icon='map-marker', prefix='fa')
    ).add_to(m)

    compass_html = f"""
    <div style="position: fixed; bottom: 40px; left: 40px; width: 140px; height: 160px;
                background: rgba(255,255,255,0.9); border-radius: 10px; 
                text-align:center; padding-top: 5px; font-family: sans-serif;
                box-shadow: 0 0 6px rgba(0,0,0,0.3);">
        <h4 style="margin:5px;">Heading</h4>
        <img src="https://upload.wikimedia.org/wikipedia/commons/3/3c/Compass_rose_simple.svg"
             style="width:80px;height:80px;transform: rotate({-heading}deg);">
        <p style="margin:0;"><b>{heading:.1f}°</b></p>
        <hr style="margin:4px 10px;">
        <p style="font-size:12px;margin:0;">HDOP: {hdop:.1f}</p>
        <p style="font-size:12px;margin:0;">VDOP: {vdop:.1f}</p>
        <p style="font-size:12px;margin:0;">PDOP: {pdop:.1f}</p>
    </div>
    """
    m.get_root().html.add_child(folium.Element(compass_html))
    return m


def read_gps_data(ser):
    """Czyta dane NMEA z odbiornika"""
    data = {
        "lat": None, "lon": None, "alt": 0, "speed": 0,
        "sats": 0, "hdop": 0, "vdop": 0, "pdop": 0
    }
    try:
        while True:
            line = ser.readline().decode('ascii', errors='replace').strip()
            if not line:
                continue

            if line.startswith("$GPGGA"):
                msg = pynmea2.parse(line)
                data["lat"] = msg.latitude
                data["lon"] = msg.longitude
                data["alt"] = msg.altitude
                data["sats"] = msg.num_sats
                data["hdop"] = float(msg.horizontal_dil or 0)

            elif line.startswith("$GPGSA"):
                # GSA zawiera PDOP, HDOP, VDOP
                parts = line.split(",")
                try:
                    data["pdop"] = float(parts[15]) if len(parts) > 15 and parts[15] else 0
                    data["hdop"] = float(parts[16]) if len(parts) > 16 and parts[16] else data["hdop"]
                    data["vdop"] = float(parts[17].split("*")[0]) if len(parts) > 17 else 0
                except (ValueError, IndexError):
                    pass

            elif line.startswith("$GPRMC"):
                msg = pynmea2.parse(line)
                data["speed"] = float(msg.spd_over_grnd) * 1.852

            if data["lat"] and data["lon"]:
                return data
    except Exception:
        return None


def write_to_csv(file, data):
    """Dopisuje dane GPS do pliku CSV"""
    file_exists = os.path.isfile(file)
    with open(file, "a", newline="") as f:
        writer = csv.writer(f)
        if not file_exists:
            writer.writerow([
                "Timestamp", "Latitude", "Longitude", "Altitude_m", "Speed_kmh",
                "Satellites", "HDOP", "VDOP", "PDOP", "Heading_deg"
            ])
        writer.writerow([
            datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
            f"{data['lat']:.6f}", f"{data['lon']:.6f}",
            f"{data['alt']:.1f}", f"{data['speed']:.2f}",
            data['sats'], data['hdop'], data['vdop'], data['pdop'], f"{data['heading']:.1f}"
        ])


def append_to_gpx(file, lat, lon, alt):
    """Dopisuje punkt do pliku GPX"""
    timestamp = datetime.utcnow().strftime("%Y-%m-%dT%H:%M:%SZ")
    if not os.path.exists(file):
        # utwórz nowy plik GPX z nagłówkiem
        with open(file, "w") as f:
            f.write('<?xml version="1.0" encoding="UTF-8"?>\n')
            f.write('<gpx version="1.1" creator="RaspberryPi GPS Logger"\n')
            f.write('     xmlns="http://www.topografix.com/GPX/1/1"\n')
            f.write('     xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance"\n')
            f.write('     xsi:schemaLocation="http://www.topografix.com/GPX/1/1 '
                    'http://www.topografix.com/GPX/1/1/gpx.xsd">\n')
            f.write('  <trk>\n    <name>GPS Track</name>\n    <trkseg>\n')
    # dopisz punkt
    with open(file, "a") as f:
        f.write(f'      <trkpt lat="{lat:.6f}" lon="{lon:.6f}">\n')
        f.write(f'        <ele>{alt:.1f}</ele>\n')
        f.write(f'        <time>{timestamp}</time>\n')
        f.write('      </trkpt>\n')


def close_gpx(file):
    """Zamyka otwarty plik GPX (na koniec programu)"""
    if os.path.exists(file):
        with open(file, "a") as f:
            f.write('    </trkseg>\n  </trk>\n</gpx>\n')


# === GŁÓWNY PROGRAM ===
def main():
    session_time = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    CSV_FILE = f"gps_log_{session_time}.csv"
    GPX_FILE = f"gps_track_{session_time}.gpx"

    print(f"🚀 Uruchamianie GPS tracker (zapis do {CSV_FILE} i {GPX_FILE})...")

    try:
        ser = serial.Serial(GPS_PORT, BAUD_RATE, timeout=1)
    except serial.SerialException:
        print("❌ Nie można otworzyć portu GPS.")
        return

    last_lat, last_lon = None, None
    track_points = []

    try:
        while True:
            gps = read_gps_data(ser)
            if gps is None:
                print("⚠️ Brak danych GPS...")
                time.sleep(2)
                continue

            heading = 0
            if last_lat and last_lon:
                heading = calculate_heading(last_lat, last_lon, gps["lat"], gps["lon"])
            gps["heading"] = heading

            track_points.append((gps["lat"], gps["lon"]))
            if len(track_points) > 500:
                track_points.pop(0)

            print(f"[{datetime.now().strftime('%H:%M:%S')}] "
                  f"Lat:{gps['lat']:.6f}, Lon:{gps['lon']:.6f}, Alt:{gps['alt']:.1f}m, "
                  f"Spd:{gps['speed']:.2f}km/h, Sat:{gps['sats']}, "
                  f"HDOP:{gps['hdop']:.1f}, VDOP:{gps['vdop']:.1f}, PDOP:{gps['pdop']:.1f}, "
                  f"Head:{heading:.1f}°")

            # zapis CSV i GPX
            write_to_csv(CSV_FILE, gps)
            append_to_gpx(GPX_FILE, gps["lat"], gps["lon"], gps["alt"])

            # aktualizacja mapy
            m = create_map(gps["lat"], gps["lon"], heading, gps["alt"],
                           gps["speed"], gps["sats"], gps["hdop"],
                           gps["vdop"], gps["pdop"], track_points)
            m.save("map.html")

            last_lat, last_lon = gps["lat"], gps["lon"]
            time.sleep(5)

    except KeyboardInterrupt:
        print("\n🛑 Zatrzymano program. Zamykanie pliku GPX...")
        close_gpx(GPX_FILE)
        print("✅ Dane zapisane poprawnie do plików:")
        print(f"   - {CSV_FILE}")
        print(f"   - {GPX_FILE}")


if __name__ == "__main__":
    main()
