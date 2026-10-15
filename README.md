# CYD28 Plane Spotter ✈️

A real-time aircraft tracker for the **ESP32 Cheap Yellow Display (CYD)** that renders a live radar PPI (Plan Position Indicator), shows nearby aircraft details, and provides weather & system status — all on a 2.8" 240×320 TFT touchscreen.

## Features

- **Radar PPI** — rotating sweep line with blips for the 20 nearest aircraft, each showing callsign, flight level (FL), and speed in knots. Configurable range with on-screen −/+ buttons.
- **Target Intel** — detailed view of the nearest aircraft: callsign, airline, route (origin → destination), aircraft type and registration, distance/bearing, altitude with climb/descent rate, speed, and heading arrow.
- **Top 5 In Range** — list of the 5 closest flights with route and altitude info.
- **Weather & System** — live temperature, humidity, and wind from Open-Meteo, plus system stats (uptime, WiFi RSSI, free RAM, API request counters).
- **Settings screen** — shows the WiFi network, the device's web address and the current location, with on-screen buttons to start WiFi setup or locate the device by its IP address.
- **WiFi setup portal** — no credentials in code needed: on first boot (or when the saved network is unreachable) the device opens a `PlaneSpotter-Setup` hotspot and shows a QR code: scan it with the phone camera to join, then pick your network and enter your city on the page that opens.
- **Web Configuration** — built-in web server with a city search, plus home location, search radius, radar range and a "Forget WiFi" button, from any browser on the same network.
- **Dual-core architecture** — all HTTP/API calls (aircraft, routes, weather) run on Core 0 in a FreeRTOS task; UI rendering and touch handling run on Core 1, keeping the display smooth.
- **Back RGB LED alerts** — the LED on the back of the board can glow green/yellow/red by distance to the nearest aircraft, and/or blink when a special aircraft comes within the alert range: helicopters (blue), military (magenta), government/rare aircraft (orange), low-flying aircraft (cyan), or anything on your own watch list of type codes and callsign prefixes (white). Each aircraft blinks once, then the LED glows steadily while it stays in range.
- **Imperial or metric units** — miles/°F/mph by default, switchable to km/°C/km/h.

## Hardware

CYD boards ship with different display controllers, so there are three builds. The USB ports are only a hint; the firmware logs the panel ID at boot (`0xD3: xx 93 41` = ILI9341, `0x04: 85 85 52` = ST7789), and at power-on the screen flashes red/green/blue if the build matches.

| Board | Panel | PlatformIO env |
|-------|-------|----------------|
| ESP32-2432S028R, one micro-USB | ILI9341 | `cyd` (default) |
| ESP32-2432S028R, micro-USB + USB-C, panel label HSD028309 | ILI9341 | `cyd-alt` |
| ESP32-2432S028R "CYD2USB", micro-USB + USB-C | ST7789 | `cyd2usb` |

If the screen stays white or black after flashing, try the next build. If colors look inverted, add `-D DISPLAY_INVERT=1` to that environment's `build_flags`.

- **Touch**: XPT2046 resistive touch controller
- **RGB LED** (back side): red GPIO4, green GPIO16, blue GPIO17, active LOW
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
   pio run -e cyd -t upload        # see the table above for which env
   pio run -e cyd-alt -t upload
   pio run -e cyd2usb -t upload
   ```
5. **WiFi setup** (if no credentials in `config.env`): the screen shows instructions and a QR code. Scan it with your phone camera to join the `PlaneSpotter-Setup` network (or join it manually); the setup page opens by itself (or browse to `http://192.168.4.1`). Tap *Configure WiFi*, choose your network, enter the password and optionally your city (or `lat,lon`), and save. If nothing happens for 5 minutes the device restarts and tries again.
6. Once connected, the web address is shown on the Settings screen; open it in a browser to change settings live.

### Flashing a prebuilt image (no PlatformIO)

Every pull request and push to `master` is built by GitHub Actions. Open the run on the **Actions** tab and download the `firmware-<env>` artifact for your board. Flash `<env>-merged.bin` at offset **0x0** with [ESP Tool](https://espressif.github.io/esptool-js/) in Chrome or Edge: *Connect*, pick the board's serial port, set *Flash Address* to `0x0`, choose the file, *Program*. WiFi and location are then set up through the setup portal.

## Usage

- **Tap the screen** to cycle through the 5 screens.
- **On the Settings screen**: *WiFi setup* (tap twice to confirm) reopens the setup portal to switch networks; *Locate by IP* sets an approximate location automatically.
- **Changing networks from the browser**: *Forget WiFi & restart* on the web page erases the saved network and reboots into the setup portal. (Credentials hard-coded in `config.env` are always tried first.)
- **On the Radar screen**, tap the **−/+ buttons** at the bottom-left to adjust range (5–150 mi in 5 mi steps, or 10–250 km when metric).
- **Back LED** settings are on the web page: distance colors, which aircraft trigger a blink, the alert range, the watch list (e.g. `B52, C17, A388, N911`) and brightness, plus a *Test LED* button. The last alert is shown on the Settings screen.
- **Open the device IP** in a browser to search for your city, or set coordinates, search radius (nautical miles, up to 250) and radar range by hand.

Aircraft on the ground are hidden, so taxiing traffic at a nearby airport doesn't crowd the screen.

## Dependencies

- [TFT_eSPI](https://github.com/Bodmer/TFT_eSPI) — TFT display driver
- [ArduinoJson](https://arduinojson.org/) — JSON parsing
- [XPT2046_Touchscreen](https://github.com/PaulStoffregen/XPT2046_Touchscreen) — Touch controller
- [WiFiManager](https://github.com/tzapu/WiFiManager) — WiFi setup portal
- [QR Code generator](https://github.com/nayuki/QR-Code-generator) by Project Nayuki (MIT) — vendored in `lib/cydqr` with renamed symbols

All managed automatically via PlatformIO `lib_deps`.

## License

MIT
