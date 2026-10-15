# CYD28 Plane Spotter ✈️

A real-time aircraft tracker for the **ESP32 Cheap Yellow Display (CYD)** that renders a live radar PPI (Plan Position Indicator), shows nearby aircraft details, and provides weather & system status — all on a 2.8" 240×320 TFT touchscreen.

## Features

- **Radar PPI** — rotating sweep line with blips for the 20 nearest aircraft, each showing callsign, flight level (FL), and speed in knots. Configurable range with on-screen −/+ buttons.
- **Target Intel** — detailed view of the nearest aircraft: callsign, airline, route (origin → destination), aircraft type and registration, distance/bearing, altitude with climb/descent rate, speed, and heading arrow.
- **Top 5 In Range** — list of the 5 closest flights with route and altitude info.
- **Weather & System** — live temperature, humidity, and wind from Open-Meteo, plus system stats (uptime, WiFi RSSI, free RAM, API request counters).
- **Settings screen** — shows the WiFi network, the device's web address and the current location, with on-screen buttons to start WiFi setup or locate the device by its IP address.
- **WiFi setup portal** — no credentials in code needed: on first boot (or when the saved network is unreachable) the device opens a `PlaneSpotter-Setup` hotspot where you pick your network and enter your city from a phone.
- **Web Configuration** — built-in web server with a city search, plus home location, search radius, radar range and a "Forget WiFi" button, from any browser on the same network.
- **Dual-core architecture** — all HTTP/API calls (aircraft, routes, weather) run on Core 0 in a FreeRTOS task; UI rendering and touch handling run on Core 1, keeping the display smooth.
- **Imperial or metric units** — miles/°F/mph by default, switchable to km/°C/km/h.

## Hardware

Both CYD variants are supported — pick the matching PlatformIO environment:

| Variant | How to recognize | Panel | PlatformIO env |
|---------|------------------|-------|----------------|
| ESP32-2432S028R (original) | one micro-USB port | ILI9341 | `cyd` (default) |
| ESP32-2432S028R "CYD2USB" | micro-USB **and** USB-C | ST7789 | `cyd2usb` |

- **Touch**: XPT2046 resistive touch controller
- **Display**: 240×320 pixels, rotation 1 (landscape)

If the screen is white or the colors look inverted, try the other environment; if colors are still inverted, add `-D DISPLAY_INVERT=1` to that environment's `build_flags`.

## Data Sources

No API keys or accounts required.

| Source | Purpose |
|--------|---------|
| [airplanes.live](https://airplanes.live/) | Real-time ADS-B aircraft around your location (position, altitude, speed, type, registration) |
| [adsbdb.com](https://www.adsbdb.com/) | Callsign → airline and route (origin/destination airports) |
| [Open-Meteo](https://open-meteo.com/) | Weather (temperature, humidity, wind speed) and city search (geocoding) |
| [ip-api.com](https://ip-api.com/) | Approximate location from IP ("Locate by IP" button) |

airplanes.live is free for non-commercial use and allows about one request per second; the default 10-second poll is well within that.

## Project Structure

```
CYD28/
├── platformio.ini          # PlatformIO build configuration (envs: cyd, cyd2usb)
├── config.env.example      # Configuration template (copy to src/config.env)
├── .gitignore
├── README.md
└── src/
    ├── main.cpp            # Main application code
    └── config.env          # Your private config (gitignored)
```

## Setup

1. **Clone & open** in VS Code with the PlatformIO extension.
2. *(Optional)* **Copy the config template** — only needed to hard-code WiFi or change units/timezone:
   ```
   cp config.env.example src/config.env
   ```
3. *(Optional)* **Edit `src/config.env`** with your WiFi credentials, location and preferences:
   ```c
   #define WIFI_SSID "YourWiFi"
   #define WIFI_PASS "YourPassword"
   #define UNITS_IMPERIAL 1                       // 0 for km / °C
   #define TIMEZONE "CST6CDT,M3.2.0,M11.1.0"      // POSIX TZ string
   #define DEFAULT_HOME_LAT  41.8800              // your latitude
   #define DEFAULT_HOME_LON -88.0078              // your longitude
   ```
4. **Build & upload** for your board variant:
   ```
   pio run -e cyd -t upload        # single micro-USB
   pio run -e cyd2usb -t upload    # micro-USB + USB-C
   ```
5. **WiFi setup** (if no credentials in `config.env`): the screen shows instructions. On your phone, join the `PlaneSpotter-Setup` network; the setup page opens by itself (or browse to `http://192.168.4.1`). Tap *Configure WiFi*, choose your network, enter the password and optionally your city (or `lat,lon`), and save. If nothing happens for 5 minutes the device restarts and tries again.
6. Once connected, the web address is shown on the Settings screen; open it in a browser to change settings live.

## Usage

- **Tap the screen** to cycle through the 5 screens.
- **On the Settings screen**: *WiFi setup* (tap twice to confirm) reopens the setup portal to switch networks; *Locate by IP* sets an approximate location automatically.
- **Changing networks from the browser**: *Forget WiFi & restart* on the web page erases the saved network and reboots into the setup portal. (Credentials hard-coded in `config.env` are always tried first.)
- **On the Radar screen**, tap the **−/+ buttons** at the bottom-left to adjust range (5–150 mi in 5 mi steps, or 10–250 km when metric).
- **Open the device IP** in a browser to search for your city, or set coordinates, search radius (nautical miles, up to 250) and radar range by hand.

Aircraft on the ground are hidden, so taxiing traffic at a nearby airport doesn't crowd the screen.

## Dependencies

- [TFT_eSPI](https://github.com/Bodmer/TFT_eSPI) — TFT display driver
- [ArduinoJson](https://arduinojson.org/) — JSON parsing
- [XPT2046_Touchscreen](https://github.com/PaulStoffregen/XPT2046_Touchscreen) — Touch controller
- [WiFiManager](https://github.com/tzapu/WiFiManager) — WiFi setup portal

All managed automatically via PlatformIO `lib_deps`.

## License

MIT
