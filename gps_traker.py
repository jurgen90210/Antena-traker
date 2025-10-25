#!/usr/bin/env python3
import serial
import pynmea2
import threading
import queue
import tkinter as tk
from tkintermapview import TkinterMapView

# --- KONFIGURACJA ---
PORT = "/dev/ttyACM0"   # zmień jeśli Twój GPS to np. /dev/ttyUSB0
BAUD = 9600
REFRESH_TIME = 1000      # co ile ms odświeżać mapę

class GPSReader(threading.Thread):
    def __init__(self, port, baud, out_q):
        super().__init__(daemon=True)
        self.port = port
        self.baud = baud
        self.q = out_q
        self._stop = threading.Event()

    def stop(self):
        self._stop.set()

    def run(self):
        try:
            with serial.Serial(self.port, self.baud, timeout=1) as ser:
                while not self._stop.is_set():
                    line = ser.readline().decode(errors="ignore").strip()
                    if not line.startswith("$"):
                        continue
                    try:
                        msg = pynmea2.parse(line)
                    except pynmea2.ParseError:
                        continue
                    if isinstance(msg, pynmea2.RMC):
                        if msg.status == "A":  # aktywny fix
                            lat = msg.latitude
                            lon = msg.longitude
                            self.q.put((lat, lon))
        except Exception as e:
            self.q.put(("error", str(e)))

class GPSMapApp(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("GPS Tracker - Mapa (u-blox 6)")
        self.geometry("800x600")

        self.map_widget = TkinterMapView(self, width=800, height=600, corner_radius=0)
        self.map_widget.pack(fill="both", expand=True)

        self.marker = None
        self.q = queue.Queue()

        # uruchom wątek GPS
        self.gps_thread = GPSReader(PORT, BAUD, self.q)
        self.gps_thread.start()

        self.after(REFRESH_TIME, self.update_position)

    def update_position(self):
        try:
            while True:
                item = self.q.get_nowait()
                if item[0] == "error":
                    print(f"Błąd GPS: {item[1]}")
                    continue
                lat, lon = item
                if lat and lon:
                    print(f"Nowa pozycja: {lat:.6f}, {lon:.6f}")
                    # ustaw marker
                    if self.marker is None:
                        self.marker = self.map_widget.set_position(lat, lon, marker=True, text="Ja 📍")
                        self.map_widget.set_zoom(16)
                    else:
                        self.marker.set_position(lat, lon)
                        self.map_widget.set_position(lat, lon)
        except queue.Empty:
            pass
        self.after(REFRESH_TIME, self.update_position)

    def destroy(self):
        try:
            self.gps_thread.stop()
        except:
            pass
        super().destroy()

if __name__ == "__main__":
    app = GPSMapApp()
    app.mainloop()
