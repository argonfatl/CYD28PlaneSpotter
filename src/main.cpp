/*
 * CYD28 (ESP32) Plane Spotter - Direct TFT Rendering
 * --------------------------------------------------------------------------
 * Data sources (no API keys needed):
 *   - airplanes.live, adsb.lol, adsb.fi : live ADS-B aircraft (tried in turn)
 *   - adsbdb.com      : callsign -> airline + route, ICAO hex -> aircraft model
 *   - Open-Meteo      : current weather and city search
 *   - ip-api.com      : approximate location from the public IP
 * All network I/O runs in a FreeRTOS task on Core 0; Core 1 only draws.
 */

#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <HTTPClient.h>
#include <WebServer.h>
#include <Preferences.h>
#include <ArduinoJson.h>
#include <TFT_eSPI.h>
#include <SPI.h>
#include <XPT2046_Touchscreen.h>
#include <WiFiManager.h>
#include <Update.h>
#include <HTTPUpdate.h>
#include <esp_ota_ops.h>
#include <LittleFS.h>
#include <ArduinoOTA.h>
#include <ESPmDNS.h>
#include <cydqr.h>
#include <math.h>
#include <algorithm>
#include <time.h>
#include <sys/time.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>

// =======================================================================
// CONFIGURATION — loaded from config.env (optional: WiFi and location can
// also be set up on the device through the setup portal)
// =======================================================================
#if __has_include("config.env")
#include "config.env"
#endif

// Defaults for anything config.env doesn't define (keeps older configs building)
#ifndef WIFI_SSID
#define WIFI_SSID ""
#endif
#ifndef WIFI_PASS
#define WIFI_PASS ""
#endif
#ifndef FW_VERSION
#define FW_VERSION "dev"          // CI sets this to the commit hash
#endif
#ifndef BUILD_ENV
#define BUILD_ENV "custom"
#endif
#ifndef GITHUB_REPO
#define GITHUB_REPO "argonfatl/CYD28PlaneSpotter"   // releases for "Update from GitHub"
#endif
#ifndef DEVICE_HOSTNAME
#define DEVICE_HOSTNAME "planespotter"   // http://planespotter.local
#endif
#ifndef SETUP_AP_NAME
#define SETUP_AP_NAME "PlaneSpotter-Setup"
#endif
#ifndef DEFAULT_HOME_NAME
#define DEFAULT_HOME_NAME "Lombard, Illinois"
#endif
#ifndef UNITS_IMPERIAL
#define UNITS_IMPERIAL 1
#endif
#ifndef TIMEZONE
#define TIMEZONE "CST6CDT,M3.2.0,M11.1.0"
#endif
#ifndef UPDATE_INTERVAL_MS
#define UPDATE_INTERVAL_MS 10000
#endif
#ifndef WEATHER_INTERVAL_MS
#define WEATHER_INTERVAL_MS 600000
#endif
#ifndef DEFAULT_HOME_LAT
#define DEFAULT_HOME_LAT 41.8800
#endif
#ifndef DEFAULT_HOME_LON
#define DEFAULT_HOME_LON -88.0078
#endif
#ifndef DEFAULT_SEARCH_RADIUS_NM
#define DEFAULT_SEARCH_RADIUS_NM 25
#endif
#ifndef DEFAULT_RADAR_RANGE
#define DEFAULT_RADAR_RANGE 30
#endif
#ifndef DISPLAY_INVERT
#define DISPLAY_INVERT 0
#endif

#if UNITS_IMPERIAL
  #define DIST_UNIT "mi"
  #define TEMP_UNIT "F"
  #define WIND_UNIT "mph"
  static const double KM_PER_UNIT = 1.609344;
  static const float  RANGE_MIN = 5, RANGE_MAX = 150;
  static const float  DEFAULT_ALERT_RANGE = 3;
#else
  #define DIST_UNIT "km"
  #define TEMP_UNIT "C"
  #define WIND_UNIT "km/h"
  static const double KM_PER_UNIT = 1.0;
  static const float  RANGE_MIN = 10, RANGE_MAX = 250;
  static const float  DEFAULT_ALERT_RANGE = 5;
#endif
static const float MAX_SEARCH_NM = 250;   // airplanes.live point-query limit

// Runtime-configurable location & radar (persisted via NVS)
double homeLat = DEFAULT_HOME_LAT;
double homeLon = DEFAULT_HOME_LON;
float  searchRadiusNm = DEFAULT_SEARCH_RADIUS_NM;  // API query radius, nautical miles
float  radarRange = DEFAULT_RADAR_RANGE;           // radar scope radius, display units
char   homeName[48] = DEFAULT_HOME_NAME;           // shown on the settings screen

SemaphoreHandle_t configMutex = NULL;  // protects homeLat/Lon/Name/searchRadiusNm/radarRange + prefs writes

// Requests handed from the UI (Core 1) to the fetch task (Core 0)
volatile bool refreshRequested = false;   // skip the rest of the poll wait
volatile bool ipLocateRequested = false;  // "Locate by IP" button
char statusMsg[48] = "";                  // one-line feedback on the settings screen
volatile bool otaActive = false;          // firmware update in progress: pause fetching/drawing
volatile bool portalActive = false;       // WiFi setup portal running: pause fetching

// Update from GitHub releases (web page). Checks and the download run in the fetch task.
enum GhState : uint8_t { GH_IDLE, GH_CHECKING, GH_DOWNLOADING, GH_FAILED };
volatile GhState ghState = GH_IDLE;
volatile bool ghCheckRequested = false, ghUpdateRequested = false;
volatile int ghPct = 0;
char ghLatest[24] = "";                   // tag of the latest release, e.g. "v1.1.0"
char ghMsg[64] = "";                      // last check / update error
uint32_t ghCheckedMs = 0;
const uint32_t GH_CHECK_INTERVAL_MS = 12UL * 3600 * 1000;
bool ghAvailable() { return ghLatest[0] && strcmp(ghLatest, FW_VERSION) != 0; }

// ---- Back RGB LED (CYD: R=GPIO4, G=GPIO16, B=GPIO17, common anode = active LOW) ----
#define LED_PIN_R 4
#define LED_PIN_G 16
#define LED_PIN_B 17
#define LED_CH_R 1
#define LED_CH_G 2
#define LED_CH_B 3

struct LedConfig {                 // protected by configMutex, persisted in NVS
  bool proximity = false;          // color by distance to the nearest aircraft
  bool alerts = true;              // blink when a special aircraft comes within alertRange
  bool heli = true;                // trigger: rotorcraft (ADS-B category A7)
  bool military = true;            // trigger: military (database flag)
  bool interesting = false;        // trigger: "interesting" (government, rare, etc.)
  int  lowAltFt = 0;               // trigger: below this altitude, 0 = off
  float alertRange = DEFAULT_ALERT_RANGE;  // display units
  uint8_t brightness = 40;         // percent
  char watch[96] = "";             // comma-separated type codes / callsign prefixes
} ledCfg;

// Phone notifications through ntfy (https://ntfy.sh): protected by configMutex, persisted in NVS
enum { NT_WATCH = 1, NT_MIL = 2, NT_INTR = 4, NT_HELI = 8, NT_LOW = 16, NT_EMERG = 32, NT_NEWTYPE = 64 };
struct NotifyConfig {
  bool on = false;
  char server[64] = "https://ntfy.sh";
  char topic[65] = "";
  float range = 5;                 // display units
  uint8_t cats = NT_WATCH | NT_MIL | NT_INTR | NT_HELI | NT_EMERG | NT_NEWTYPE;
  int lowAltFt = 1000;             // for NT_LOW
  uint8_t coolH = 6;               // the same aircraft again only after this many hours
  bool quietNight = true;          // nothing while night mode is active
} notifyCfg;
volatile bool notifyTestRequested = false;
char notifyStatus[64] = "";        // last result, for the web page

enum AlertKind : uint8_t { ALERT_NONE, ALERT_LOWALT, ALERT_HELI, ALERT_INTERESTING, ALERT_MILITARY, ALERT_WATCH, ALERT_EMERG };
struct AlertInfo {
  AlertKind kind = ALERT_NONE;
  char callsign[10] = ""; char type[6] = "";
  double distKm = 0;
};
AlertInfo currentAlert;            // highest-priority alert in range now (dataMutex)
AlertInfo lastAlert;               // most recent alert, for the settings screen (dataMutex)
uint32_t lastAlertMs = 0;
volatile bool ledTestRequested = false;

const char* alertName(AlertKind k) {
  switch (k) {
    case ALERT_EMERG: return "EMERGENCY";
    case ALERT_WATCH: return "WATCH";
    case ALERT_MILITARY: return "MIL";
    case ALERT_INTERESTING: return "SPECIAL";
    case ALERT_HELI: return "HELI";
    case ALERT_LOWALT: return "LOW";
    default: return "";
  }
}
WebServer server(80);
Preferences prefs;
// =======================================================================

// CYD Touch Pins
#define XPT2046_IRQ 36
#define XPT2046_MOSI 32
#define XPT2046_MISO 39
#define XPT2046_CLK 25
#define XPT2046_CS 33

SPIClass touchSpi = SPIClass(VSPI);
XPT2046_Touchscreen ts(XPT2046_CS);

TFT_eSPI tft = TFT_eSPI();
TFT_eSprite radarSpr = TFT_eSprite(&tft);

// Data models
struct Aircraft {
  char callsign[10]; char type[6]; char reg[10]; char airline[28];
  double lat; double lon; int altFt; float gsKt;
  float trackDeg; int vrateFpm; bool onGround;
  char category[3]; uint8_t dbFlags;   // ADS-B emitter category ("A7" = rotorcraft); bit0 military, bit1 interesting
  double distanceKm; double bearingDeg; bool valid;
  char dep[5]; char arr[5]; bool hasRoute;
  char hex[7];                          // ICAO 24-bit address, for model lookups
  char flightIata[9];                   // e.g. "UA1288"
  char depCity[20]; char arrCity[20];
  double depLat, depLon, arrLat, arrLon; bool hasCoords;
  char model[32];                       // e.g. "Boeing 737-7V3" (nearest aircraft only)
};
Aircraft nearest;
Aircraft selAc;                     // the tapped aircraft (any on the radar), under dataMutex


// Top 5 Tracker
const uint8_t MAX_TOP5 = 5;
Aircraft top5[MAX_TOP5];
uint8_t top5Count = 0;

// Blips for Radar (up to MAX_BLIPS inside the radar range, spread over distance rings)
struct Blip { double lat; double lon; float track; float speedKt; char callsign[10]; int altFt; double distKm; char hex[8]; bool heli; uint8_t ring; };

// Emergency squawk (7500 / 7600 / 7700 or a readsb emergency state) anywhere in the search area
struct Emergency {
  bool on = false;
  char cs[10] = "", hex[8] = "", type[6] = "", squawk[5] = "", what[16] = "";
  double km = 0, brg = 0; int alt = 0;
};
Emergency emergency;                // published under dataMutex
bool emergencyBanner = true;        // red banner on the screen (NVS "emg_on")
bool heliOnly = false;              // radar, traffic and nearest show helicopters only (NVS "heli_only")
bool screenMirror = false;          // web page shows the screen and passes taps (NVS "mirror")

// Selected aircraft (tap a traffic row or a radar blip): followed by ICAO address
char selectedHex[8] = "";
uint32_t selectedUntil = 0;
const uint32_t SELECT_MS = 120000;
volatile bool selectRequested = false;   // poll now to fill in the details

// Radar trails: last positions of the aircraft on the radar
const int TRAIL_PTS = 8;
struct Trail { char hex[8]; uint8_t n; float lat[TRAIL_PTS], lon[TRAIL_PTS]; uint32_t seenMs; };
Trail trails[40];                   // under dataMutex

// Extra screens and page rotation (web page "Screens" card)
enum { SHOW_OFF, SHOW_PAGE, SHOW_IDLE };
struct ScreensConfig {
  bool rotate = false; uint16_t rotateSec = 15;
  uint8_t clockShow = SHOW_IDLE, clockStyle = 0; bool clock24 = false;
  bool clockAtNight = false;        // clock only while night mode has the screen dimmed
  uint8_t metarShow = SHOW_OFF; char metarIcao[5] = "";
  float idleRange = 10;             // "no aircraft nearby" = none within this (display units)
} scr;
volatile bool metarRequested = false;

struct Metar {
  bool valid = false;
  char id[5] = "", name[40] = "", cat[5] = "", visib[8] = "", ceiling[16] = "";
  uint32_t obs = 0; float temp = 0, dewp = 0, altim = 0; int wdir = 0, wspd = 0, wgst = 0;
  char raw[168] = "";
} metar;                            // under dataMutex
char metarErr[48] = "";
const uint8_t MAX_BLIPS = 40;
Blip blips[MAX_BLIPS];
uint8_t blipCount = 0;
uint32_t lastDataMs = 0;

// Async data fetch (FreeRTOS task on Core 0)
SemaphoreHandle_t dataMutex = NULL;
TaskHandle_t fetchTaskHandle = NULL;

// Route Cache
const uint8_t ROUTE_CACHE_SIZE = 15;
struct CachedRoute {
  char callsign[10];
  char dep[5]; char arr[5]; char airline[28];
  char flightIata[9]; char depCity[20]; char arrCity[20];
  double depLat, depLon, arrLat, arrLon; bool hasCoords;
  bool valid; uint32_t fetchTime;
};
CachedRoute routeCache[ROUTE_CACHE_SIZE];

struct Weather {
  float temp; float wind; int humidity; int code; float pressure; int windDir; bool valid = false;
  uint32_t sunrise = 0, sunset = 0;   // today's, unix time (for night mode)
} weather;

struct Stats {
  uint32_t requestsOk = 0; uint32_t requestsFail = 0;
  uint16_t inView = 0; uint16_t maxInView = 0;
  uint32_t lastUpdateMs = 0;
} stats;

uint8_t screen = 0;
uint8_t lastScreen = 255;
uint32_t lastTouchMs = 0;
uint32_t wifiConfirmUntil = 0;   // "WiFi setup" needs a second tap within 3 s
uint32_t updConfirmUntil = 0;    // "Update" (install) needs a second tap within 3 s
volatile bool portalRequested = false;
volatile bool calibrateRequested = false;   // from the settings menu or the web page
enum { S_NEAREST, S_TRAFFIC, S_RADAR, S_WEATHER, S_CLOCK, S_METAR };
const uint8_t NUM_SCREENS = 6;              // pages; clock and METAR only when enabled
const uint8_t SCREEN_SETTINGS = 6;          // settings menu, opened from the clock / WiFi icon
uint8_t screenBeforeSettings = 0;

// Optional on-screen arrows: a tap shows them, they hide after 15 s without touches
bool navButtons = false;
uint32_t navShownUntil = 0;
const uint32_t NAV_TIMEOUT_MS = 15000;

// Touch calibration: screen = affine(raw). Default matches a typical CYD panel.
float touchCal[6] = {0.0f, -0.086486f, 337.30f,    // sx = a*rx + b*ry + c
                     -0.064865f, 0.0f, 252.97f};   // sy = d*rx + e*ry + f

// Screen backlight (GPIO21) via LEDC channel 0
#define BL_CH 0
uint8_t backlightPct = 100;       // day brightness, set from the settings menu
uint8_t backlightApplied = 255;   // what the PWM currently outputs

// Night mode: dim (or turn off) the screen at night; a tap wakes it for 30 s
struct NightConfig {
  uint8_t mode = 0;               // 0 off, 1 schedule, 2 sunset to sunrise
  uint16_t fromMin = 22 * 60;     // schedule start, minutes after midnight
  uint16_t toMin = 7 * 60;        // schedule end
  uint8_t level = 10;             // night brightness in percent, 0 = screen off
  bool ledOff = true;             // back LED off at night
} nightCfg;
uint32_t nightWakeUntil = 0;
bool nightNow();
const uint32_t NIGHT_WAKE_MS = 30000;

// Previous firmware (the other OTA slot) for "switch back"; names kept in NVS
String prevFwVersion;

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------
static double deg2rad(double d) { return d * (PI / 180.0); }
static double rad2deg(double r) { return r * (180.0 / PI); }

double haversineKm(double lat1, double lon1, double lat2, double lon2) {
  const double R = 6371.0;
  double dLat = deg2rad(lat2 - lat1); double dLon = deg2rad(lon2 - lon1);
  double a = sin(dLat / 2) * sin(dLat / 2) + cos(deg2rad(lat1)) * cos(deg2rad(lat2)) * sin(dLon / 2) * sin(dLon / 2);
  return R * 2 * atan2(sqrt(a), sqrt(1 - a));
}

double bearingDeg(double lat1, double lon1, double lat2, double lon2) {
  double y = sin(deg2rad(lon2 - lon1)) * cos(deg2rad(lat2));
  double x = cos(deg2rad(lat1)) * sin(deg2rad(lat2)) - sin(deg2rad(lat1)) * cos(deg2rad(lat2)) * cos(deg2rad(lon2 - lon1));
  double b = rad2deg(atan2(y, x));
  return fmod(b + 360.0, 360.0);
}

const char* compass(double bearing) {
  static const char* dirs[] = {"N", "NE", "E", "SE", "S", "SW", "W", "NW"};
  return dirs[(int)((bearing + 22.5) / 45.0) % 8];
}

void projectLatLon(double lat, double lon, float trackDeg, double distM, double& outLat, double& outLon) {
  double dr = distM / 6371000.0; double b = deg2rad(trackDeg);
  double la = deg2rad(lat), lo = deg2rad(lon);
  double nla = asin(sin(la) * cos(dr) + cos(la) * sin(dr) * cos(b));
  double nlo = lo + atan2(sin(b) * sin(dr) * cos(la), cos(dr) - sin(la) * sin(nla));
  outLat = rad2deg(nla); outLon = rad2deg(nlo);
}

// Copy, dropping non-ASCII bytes (built-in TFT fonts are ASCII only) and trailing spaces
void copyAscii(char* dst, const char* src, size_t n) {
  size_t j = 0;
  for (size_t i = 0; src && src[i] && j < n - 1; i++)
    if ((uint8_t)src[i] < 128) dst[j++] = src[i];
  dst[j] = '\0';
  while (j > 0 && dst[j - 1] == ' ') dst[--j] = '\0';
}

double toDisplayDist(double km) { return km / KM_PER_UNIT; }

// ---------------------------------------------------------------------------
// Network & APIs
// ---------------------------------------------------------------------------
// GET a URL and stream-parse the JSON body (no full-payload String in RAM).
// Returns the HTTP status code, or a negative value on connection/parse failure.
// If err is given, a short reason is written there on failure.
int httpGetJson(const String& url, JsonDocument& doc, JsonDocument* filter,
                char* err = nullptr, size_t errLen = 0, const char* postBody = nullptr) {
  WiFiClientSecure secureClient; secureClient.setInsecure();
  WiFiClient plainClient;
  WiFiClient& client = url.startsWith("https://") ? (WiFiClient&)secureClient : plainClient;
  HTTPClient https;
  https.setReuse(false);
  https.useHTTP10(true);   // no chunked encoding -> getStream() is plain JSON
  https.setTimeout(15000);
  // NB: addHeader("User-Agent", ...) is silently ignored by HTTPClient; without this
  // the request goes out as "ESP32HTTPClient", which some APIs reject.
  https.setUserAgent("CYD28PlaneSpotter/1.2 (+https://github.com/argonfatl/CYD28PlaneSpotter)");
  if (!https.begin(client, url)) {
    if (err) snprintf(err, errLen, "begin failed");
    return -1;
  }

  int code;
  if (postBody) { https.addHeader("Content-Type", "application/json"); code = https.POST((uint8_t*)postBody, strlen(postBody)); }
  else code = https.GET();
  if (code != HTTP_CODE_OK) {
    if (code < 0) {
      // HTTPClient reports every connect failure as "connection refused"; for HTTPS
      // the real reason (DNS, TCP, TLS handshake, out of memory) is in the TLS client
      char tls[64] = "";
      if (url.startsWith("https://")) {
        int e = secureClient.lastError(tls, sizeof(tls));
        if (e == 0) tls[0] = '\0';
      }
      Serial.printf("HTTP error %d (%s) [%s] heap %u max block %u: %s\n", code,
                    https.errorToString(code).c_str(), tls, (unsigned)ESP.getFreeHeap(),
                    (unsigned)ESP.getMaxAllocHeap(), url.c_str());
      if (err) {
        if (tls[0]) snprintf(err, errLen, "TLS: %s", tls);
        else snprintf(err, errLen, "%s", https.errorToString(code).c_str());
      }
    } else {
      Serial.printf("HTTP %d: %s\n", code, url.c_str());
      if (err) snprintf(err, errLen, "HTTP %d", code);
    }
    https.end();
    return code;
  }

  DeserializationError jerr = filter
      ? deserializeJson(doc, https.getStream(), DeserializationOption::Filter(*filter))
      : deserializeJson(doc, https.getStream());
  https.end();
  if (jerr) {
    Serial.printf("JSON error (%s): %s\n", jerr.c_str(), url.c_str());
    if (err) snprintf(err, errLen, "JSON %s", jerr.c_str());
    return -2;
  }
  return code;
}

// ---------------------------------------------------------------------------
// Location: set / geocode / locate by IP
// ---------------------------------------------------------------------------
String urlEncode(const String& s) {
  String out;
  const char* hex = "0123456789ABCDEF";
  for (unsigned i = 0; i < s.length(); i++) {
    uint8_t c = (uint8_t)s[i];
    if (isalnum(c) || c == '-' || c == '_' || c == '.' || c == '~') out += (char)c;
    else if (c == ' ') out += "%20";
    else { out += '%'; out += hex[c >> 4]; out += hex[c & 15]; }
  }
  return out;
}

void setStatus(const char* msg) {
  strncpy(statusMsg, msg, sizeof(statusMsg) - 1);
  statusMsg[sizeof(statusMsg) - 1] = '\0';
}

// Apply and persist a new home location, then trigger an immediate refresh
void setLocation(double lat, double lon, const char* name) {
  if (configMutex) xSemaphoreTake(configMutex, portMAX_DELAY);
  homeLat = lat; homeLon = lon;
  copyAscii(homeName, (name && name[0]) ? name : "Custom location", sizeof(homeName));
  prefs.putDouble("lat", lat);
  prefs.putDouble("lon", lon);
  prefs.putString("name", homeName);
  if (configMutex) xSemaphoreGive(configMutex);
  Serial.printf("Location set: %s (%.4f, %.4f)\n", homeName, lat, lon);
  refreshRequested = true;
}

struct GeoResult { char name[48]; double lat; double lon; };

// City search via the Open-Meteo geocoding API. Returns the number of results.
int geocodeCity(const String& query, GeoResult* out, int maxResults) {
  String url = "https://geocoding-api.open-meteo.com/v1/search?count=" + String(maxResults) +
               "&language=en&format=json&name=" + urlEncode(query);
  JsonDocument filter;
  filter["results"][0]["name"] = true;
  filter["results"][0]["admin1"] = true;
  filter["results"][0]["country_code"] = true;
  filter["results"][0]["latitude"] = true;
  filter["results"][0]["longitude"] = true;

  JsonDocument doc;
  if (httpGetJson(url, doc, &filter) != HTTP_CODE_OK) return -1;

  int n = 0;
  for (JsonObject r : doc["results"].as<JsonArray>()) {
    if (n >= maxResults) break;
    char buf[96];
    const char* admin = r["admin1"] | "";
    snprintf(buf, sizeof(buf), admin[0] ? "%s, %s, %s" : "%s, %s",
             (const char*)(r["name"] | "?"),
             admin[0] ? admin : (const char*)(r["country_code"] | ""),
             (const char*)(r["country_code"] | ""));
    copyAscii(out[n].name, buf, sizeof(out[n].name));
    out[n].lat = r["latitude"] | 0.0;
    out[n].lon = r["longitude"] | 0.0;
    n++;
  }
  return n;
}

// Approximate location from the public IP (ip-api.com, free, HTTP only)
bool locateByIp() {
  JsonDocument doc;
  int code = httpGetJson("http://ip-api.com/json/?fields=status,city,regionName,lat,lon", doc, nullptr);
  if (code != HTTP_CODE_OK || strcmp(doc["status"] | "", "success") != 0) return false;
  char name[64];
  snprintf(name, sizeof(name), "%s, %s", (const char*)(doc["city"] | "?"),
           (const char*)(doc["regionName"] | ""));
  setLocation(doc["lat"] | 0.0, doc["lon"] | 0.0, name);
  return true;
}

// Text typed in the setup portal: either "lat, lon" or a city name
bool applyLocationInput(const char* input) {
  String s = String(input); s.trim();
  if (s.isEmpty()) return false;

  double lat, lon;
  if (sscanf(s.c_str(), "%lf , %lf", &lat, &lon) == 2 &&
      lat >= -90 && lat <= 90 && lon >= -180 && lon <= 180) {
    setLocation(lat, lon, "Custom location");
    return true;
  }
  GeoResult r[1];
  if (geocodeCity(s, r, 1) == 1) { setLocation(r[0].lat, r[0].lon, r[0].name); return true; }
  Serial.printf("Could not find location '%s'\n", s.c_str());
  return false;
}

// Fills a.dep / a.arr / a.airline from adsbdb (cached). Returns true if a route is known.
void routeToAircraft(const CachedRoute& c, Aircraft& a) {
  strcpy(a.dep, c.dep); strcpy(a.arr, c.arr); strcpy(a.airline, c.airline);
  strcpy(a.flightIata, c.flightIata); strcpy(a.depCity, c.depCity); strcpy(a.arrCity, c.arrCity);
  a.depLat = c.depLat; a.depLon = c.depLon; a.arrLat = c.arrLat; a.arrLon = c.arrLon;
  a.hasCoords = c.hasCoords;
}

bool getRoute(Aircraft& a) {
  CachedRoute r = {};
  routeToAircraft(r, a);
  const char* cs = a.callsign;
  if (strlen(cs) < 3 || strcmp(cs, "(no id)") == 0) return false;

  for (int i = 0; i < ROUTE_CACHE_SIZE; i++) {
    if (routeCache[i].valid && strcmp(routeCache[i].callsign, cs) == 0) {
      routeToAircraft(routeCache[i], a);
      return a.dep[0] != '\0';
    }
  }

  JsonDocument filter;
  JsonObject fr = filter["response"]["flightroute"].to<JsonObject>();
  fr["callsign_iata"] = true;
  fr["airline"]["name"] = true;
  for (const char* end : {"origin", "destination"})
    for (const char* f : {"iata_code", "icao_code", "municipality", "latitude", "longitude"})
      fr[end][f] = true;

  JsonDocument d;
  int code = httpGetJson(String("https://api.adsbdb.com/v0/callsign/") + cs, d, &filter);
  if (code != HTTP_CODE_OK && code != HTTP_CODE_NOT_FOUND) return false;  // transient: don't cache

  if (code == HTTP_CODE_OK) {
    JsonObject f = d["response"]["flightroute"];
    if (!f.isNull()) {
      JsonObject o = f["origin"], t = f["destination"];
      const char* oc = o["iata_code"] | ""; if (!oc[0]) oc = o["icao_code"] | "";
      const char* tc = t["iata_code"] | ""; if (!tc[0]) tc = t["icao_code"] | "";
      if (oc[0] && tc[0]) {
        copyAscii(r.dep, oc, sizeof(r.dep));
        copyAscii(r.arr, tc, sizeof(r.arr));
        copyAscii(r.depCity, o["municipality"] | "", sizeof(r.depCity));
        copyAscii(r.arrCity, t["municipality"] | "", sizeof(r.arrCity));
        if (o["latitude"].is<double>() && t["latitude"].is<double>()) {
          r.depLat = o["latitude"]; r.depLon = o["longitude"];
          r.arrLat = t["latitude"]; r.arrLon = t["longitude"];
          r.hasCoords = true;
        }
      }
      copyAscii(r.airline, f["airline"]["name"] | "", sizeof(r.airline));
      copyAscii(r.flightIata, f["callsign_iata"] | "", sizeof(r.flightIata));
    }
  }

  // Cache both hits and "unknown callsign" answers
  int replaceIdx = 0;
  uint32_t oldest = 0xFFFFFFFF;
  for (int i = 0; i < ROUTE_CACHE_SIZE; i++) {
    if (!routeCache[i].valid) { replaceIdx = i; break; }
    if (routeCache[i].fetchTime < oldest) { oldest = routeCache[i].fetchTime; replaceIdx = i; }
  }
  strcpy(r.callsign, cs);
  r.valid = true;
  r.fetchTime = millis();
  routeCache[replaceIdx] = r;
  routeToAircraft(r, a);
  return a.dep[0] != '\0';
}

// ---------------------------------------------------------------------------
// Route sanity. Route databases are keyed by callsign and go stale when airlines
// reuse flight numbers, so every route is checked against where the aircraft is.
// Returns 1 = fits, 0 = doesn't fit (hide it), -1 = fits but reversed (swap ends).
// ---------------------------------------------------------------------------
double angleDiff(double a, double b) { return fabs(fmod(a - b + 540.0, 360.0) - 180.0); }

int routeFit(const Aircraft& a, double oLat, double oLon, double dLat, double dLon) {
  double dO = haversineKm(a.lat, a.lon, oLat, oLon), dD = haversineKm(a.lat, a.lon, dLat, dLon);
  double L = haversineKm(oLat, oLon, dLat, dLon);
  const double NEAR = 40;
  // Below 10,000 ft an airliner is climbing out of or descending into an airport, not
  // cruising between two distant ones: it must be near one of them (short hops excepted)
  if (dO > 120 && dD > 120 && a.altFt < 10000 &&
      (a.vrateFpm > 500 || a.vrateFpm < -500 || a.altFt < 5000 || L > 400)) return 0;
  // Off the corridor between the two airports (an ellipse around the great circle)
  if (dO > NEAR && dD > NEAR && dO + dD > L + (L * 0.25 > 150 ? L * 0.25 : 150)) return 0;
  if (dO <= NEAR || dD <= NEAR) {                 // around one of the airports: climb or descent tells
    bool atOrigin = dO <= dD;
    if (a.vrateFpm > 600) return atOrigin ? 1 : -1;
    if (a.vrateFpm < -600) return atOrigin ? -1 : 1;
    return 1;
  }
  // En route: heading away from the destination and toward the origin means reversed
  if (a.gsKt > 50 && angleDiff(a.trackDeg, bearingDeg(a.lat, a.lon, dLat, dLon)) > 100 &&
      angleDiff(a.trackDeg, bearingDeg(a.lat, a.lon, oLat, oLon)) < 80) return -1;
  return 1;
}

// Second route source: adsb.lol routeset (POST, several callsigns per request)
struct RsRoute {
  char cs[10]; bool found;
  char dep[5], arr[5], depCity[20], arrCity[20];
  double oLat, oLon, dLat, dLon; uint32_t t;
};
RsRoute rsCache[16]; int rsNext = 0;
char routesetStatus[72] = "not used yet";   // shown on the web page

RsRoute* rsFind(const char* cs) {
  for (auto& r : rsCache)
    if (r.cs[0] && strcmp(r.cs, cs) == 0 && millis() - r.t < (r.found ? 7200000UL : 1800000UL)) return &r;
  return nullptr;
}

bool usableCallsign(const char* cs) { return strlen(cs) >= 3 && strcmp(cs, "(no id)") != 0 && isalpha((uint8_t)cs[0]); }

void routesetLookup(Aircraft** list, int n) {
  String body = "{\"planes\":[";
  char asked[6][10]; int nAsk = 0;
  for (int i = 0; i < n && nAsk < 6; i++) {
    const Aircraft& a = *list[i];
    if (!usableCallsign(a.callsign) || rsFind(a.callsign)) continue;
    bool dup = false; for (int k = 0; k < nAsk; k++) if (!strcmp(asked[k], a.callsign)) dup = true;
    if (dup) continue;
    strcpy(asked[nAsk++], a.callsign);
    char e[96];
    snprintf(e, sizeof(e), "%s{\"callsign\":\"%s\",\"lat\":%.4f,\"lng\":%.4f}", nAsk > 1 ? "," : "", a.callsign, a.lat, a.lon);
    body += e;
  }
  if (!nAsk) return;
  body += "]}";

  JsonDocument filter;
  filter[0]["callsign"] = true;
  for (const char* f : {"iata", "icao", "lat", "lon", "location", "name"}) filter[0]["_airports"][0][f] = true;
  JsonDocument d; char err[48] = "";
  int code = httpGetJson("https://api.adsb.lol/api/0/routeset", d, &filter, err, sizeof(err), body.c_str());
  if (code != HTTP_CODE_OK || !d.is<JsonArray>()) {
    Serial.printf("routeset: %d %s\n", code, err);
    snprintf(routesetStatus, sizeof(routesetStatus), "error %d %s", code, err);
    return;
  }

  bool got[6] = {};
  for (JsonObject o : d.as<JsonArray>()) {
    const char* cs = o["callsign"] | "";
    int k = -1; for (int j = 0; j < nAsk; j++) if (!strcmp(asked[j], cs)) k = j;
    JsonArray ap = o["_airports"].as<JsonArray>();
    if (k < 0 || ap.size() < 2) continue;
    // Multi-leg routes: take the leg the aircraft is closest to flying
    const Aircraft* ac = nullptr;
    for (int i = 0; i < n; i++) if (!strcmp(list[i]->callsign, cs)) { ac = list[i]; break; }
    int best = 0; double bestX = 1e18;
    for (size_t i = 0; i + 1 < ap.size(); i++) {
      double oLa = ap[i]["lat"] | 0.0, oLo = ap[i]["lon"] | 0.0, dLa = ap[i + 1]["lat"] | 0.0, dLo = ap[i + 1]["lon"] | 0.0;
      double x = ac ? haversineKm(ac->lat, ac->lon, oLa, oLo) + haversineKm(ac->lat, ac->lon, dLa, dLo) - haversineKm(oLa, oLo, dLa, dLo) : 0;
      if (x < bestX) { bestX = x; best = i; }
    }
    JsonObject o1 = ap[best], o2 = ap[best + 1];
    RsRoute& r = rsCache[rsNext]; rsNext = (rsNext + 1) % 16;
    r = RsRoute(); strcpy(r.cs, cs); r.found = true; r.t = millis();
    const char* c1 = o1["iata"] | ""; if (!c1[0]) c1 = o1["icao"] | "";
    const char* c2 = o2["iata"] | ""; if (!c2[0]) c2 = o2["icao"] | "";
    copyAscii(r.dep, c1, sizeof(r.dep)); copyAscii(r.arr, c2, sizeof(r.arr));
    const char* l1 = o1["location"] | ""; if (!l1[0]) l1 = o1["name"] | "";
    const char* l2 = o2["location"] | ""; if (!l2[0]) l2 = o2["name"] | "";
    copyAscii(r.depCity, l1, sizeof(r.depCity)); copyAscii(r.arrCity, l2, sizeof(r.arrCity));
    r.oLat = o1["lat"] | 0.0; r.oLon = o1["lon"] | 0.0; r.dLat = o2["lat"] | 0.0; r.dLon = o2["lon"] | 0.0;
    got[k] = true;
    Serial.printf("routeset %s: %s-%s\n", cs, r.dep, r.arr);
  }
  { int ng = 0; for (int k = 0; k < nAsk; k++) ng += got[k];
    snprintf(routesetStatus, sizeof(routesetStatus), "OK, last answer %d of %d", ng, nAsk); }
  for (int k = 0; k < nAsk; k++) if (!got[k]) {        // remember "unknown" too, so we don't ask every poll
    RsRoute& r = rsCache[rsNext]; rsNext = (rsNext + 1) % 16;
    r = RsRoute(); strcpy(r.cs, asked[k]); r.found = false; r.t = millis();
  }
}

// Picks the route to show: adsb.lol if it fits, else adsbdb if it fits, else none
void chooseRoute(Aircraft& a) {
  RsRoute* r = rsFind(a.callsign);
  if (r && r->found && r->dep[0] && r->arr[0]) {
    int f = routeFit(a, r->oLat, r->oLon, r->dLat, r->dLon);
    if (f != 0) {
      bool sw = f < 0;
      strcpy(a.dep, sw ? r->arr : r->dep); strcpy(a.arr, sw ? r->dep : r->arr);
      strcpy(a.depCity, sw ? r->arrCity : r->depCity); strcpy(a.arrCity, sw ? r->depCity : r->arrCity);
      a.depLat = sw ? r->dLat : r->oLat; a.depLon = sw ? r->dLon : r->oLon;
      a.arrLat = sw ? r->oLat : r->dLat; a.arrLon = sw ? r->oLon : r->dLon;
      a.hasCoords = true; a.hasRoute = true;
      return;
    }
  }
  if (!a.hasRoute || !a.hasCoords) return;
  int f = routeFit(a, a.depLat, a.depLon, a.arrLat, a.arrLon);
  if (f == 0) {
    Serial.printf("Route %s %s-%s doesn't fit the position, hidden\n", a.callsign, a.dep, a.arr);
    a.hasRoute = false; a.hasCoords = false;
    a.dep[0] = a.arr[0] = a.depCity[0] = a.arrCity[0] = '\0';
  } else if (f < 0) {
    char t[20];
    strcpy(t, a.dep); strcpy(a.dep, a.arr); strcpy(a.arr, t);
    strcpy(t, a.depCity); strcpy(a.depCity, a.arrCity); strcpy(a.arrCity, t);
    double x = a.depLat; a.depLat = a.arrLat; a.arrLat = x;
    x = a.depLon; a.depLon = a.arrLon; a.arrLon = x;
  }
}

// Full aircraft model ("Boeing 737-7V3") and owner from adsbdb, by ICAO hex (small cache)
void getAircraftModel(Aircraft& a) {
  struct ModelCache { char hex[7]; char model[32]; char owner[28]; };
  static ModelCache cache[8] = {};
  static int next = 0;
  a.model[0] = '\0';
  if (!a.hex[0]) return;
  for (auto& c : cache) {
    if (strcmp(c.hex, a.hex) == 0) {
      strcpy(a.model, c.model);
      if (!a.airline[0]) strcpy(a.airline, c.owner);
      return;
    }
  }
  JsonDocument filter;
  for (const char* f : {"manufacturer", "type", "registered_owner"})
    filter["response"]["aircraft"][f] = true;
  JsonDocument d;
  int code = httpGetJson(String("https://api.adsbdb.com/v0/aircraft/") + a.hex, d, &filter);
  if (code != HTTP_CODE_OK && code != HTTP_CODE_NOT_FOUND) return;   // transient: retry later

  ModelCache& c = cache[next]; next = (next + 1) % 8;
  strcpy(c.hex, a.hex); c.model[0] = c.owner[0] = '\0';
  JsonObject ac = d["response"]["aircraft"];
  if (code == HTTP_CODE_OK && !ac.isNull()) {
    char mf[24], ty[28];
    copyAscii(mf, ac["manufacturer"] | "", sizeof(mf));
    copyAscii(ty, ac["type"] | "", sizeof(ty));
    char full[56];   // intentionally larger than c.model; shortened by copyAscii
    if (mf[0] && strncasecmp(ty, mf, strlen(mf)) != 0) snprintf(full, sizeof(full), "%s %s", mf, ty);
    else snprintf(full, sizeof(full), "%s", ty);
    copyAscii(c.model, full, sizeof(c.model));
    copyAscii(c.owner, ac["registered_owner"] | "", sizeof(c.owner));
  }
  strcpy(a.model, c.model);
  if (!a.airline[0]) strcpy(a.airline, c.owner);
}

// Aircraft data sources (all free, keyless, readsb JSON format), tried in order
struct Source { const char* name; const char* fmt; };
const Source SOURCES[] = {
  {"airplanes.live", "https://api.airplanes.live/v2/point/%.4f/%.4f/%d"},
  {"adsb.lol",       "https://api.adsb.lol/v2/point/%.4f/%.4f/%d"},
  {"adsb.fi",        "https://opendata.adsb.fi/api/v3/lat/%.4f/lon/%.4f/dist/%d"},
};
const int NUM_SOURCES = sizeof(SOURCES) / sizeof(SOURCES[0]);
volatile int activeSource = 0;      // index of the last source that worked
char lastFetchErr[96] = "";         // last aircraft fetch error, for the status screen
uint32_t lastOkMs = 0;              // when aircraft data last arrived

String sourceUrl(int i, double lat, double lon, int radiusNm) {
  char buf[128];
  snprintf(buf, sizeof(buf), SOURCES[i].fmt, lat, lon, radiusNm);
  return String(buf);
}

// Watch list: comma/space separated tokens; a token matches the ICAO type code
// exactly (e.g. "B52") or the start of the callsign (e.g. "N911", "RCH")
struct WatchList {
  char tok[12][10]; int n = 0;
  void parse(const char* s) {
    n = 0;
    while (*s && n < 12) {
      while (*s == ',' || *s == ' ') s++;
      int j = 0;
      while (*s && *s != ',' && *s != ' ') { if (j < 9) tok[n][j++] = toupper(*s); s++; }
      tok[n][j] = '\0';
      if (j) n++;
    }
  }
  bool match(const char* callsign, const char* type) const {
    for (int i = 0; i < n; i++) {
      if (type[0] && strcmp(type, tok[i]) == 0) return true;
      if (strncmp(callsign, tok[i], strlen(tok[i])) == 0) return true;
    }
    return false;
  }
};

AlertKind classifyAlert(const LedConfig& lc, const WatchList& wl, const char* cs, const char* type,
                        const char* category, uint8_t dbFlags, int altFt) {
  if (wl.match(cs, type)) return ALERT_WATCH;
  if (lc.military && (dbFlags & 1)) return ALERT_MILITARY;
  if (lc.interesting && (dbFlags & 2)) return ALERT_INTERESTING;
  if (lc.heli && strcmp(category, "A7") == 0) return ALERT_HELI;
  if (lc.lowAltFt > 0 && altFt < lc.lowAltFt) return ALERT_LOWALT;
  return ALERT_NONE;
}

// ---------------------------------------------------------------------------
// Aircraft list download: stream the readsb JSON and hand over one aircraft at a time
// ---------------------------------------------------------------------------
bool findAircraftArray(Stream& st) {
  char last[12] = ""; char c;
  while (st.readBytes(&c, 1) == 1) {
    memmove(last, last + 1, 10); last[10] = c; last[11] = '\0';
    if (strcmp(last + 7, "\"ac\"") == 0 || strcmp(last + 1, "\"aircraft\"") == 0) {
      while (st.readBytes(&c, 1) == 1) {
        if (c == '[') return true;
        if (c != ' ' && c != ':' && c != '\n' && c != '\r' && c != '\t') break;
      }
    }
  }
  return false;
}

template <typename F>
int streamAircraft(const String& url, JsonDocument& filter, F onAircraft, char* err, size_t errLen) {
  WiFiClientSecure client; client.setInsecure();
  HTTPClient https;
  https.setReuse(false);
  https.useHTTP10(true);
  https.setTimeout(15000);
  https.setUserAgent("CYD28PlaneSpotter/1.2 (+https://github.com/argonfatl/CYD28PlaneSpotter)");
  if (!https.begin(client, url)) { snprintf(err, errLen, "begin failed"); return -1; }
  int code = https.GET();
  if (code != HTTP_CODE_OK) {
    if (code < 0) {
      char tls[64] = "";
      if (client.lastError(tls, sizeof(tls)) == 0) tls[0] = '\0';
      Serial.printf("HTTP error %d (%s) [%s] heap %u max block %u: %s\n", code, https.errorToString(code).c_str(), tls,
                    (unsigned)ESP.getFreeHeap(), (unsigned)ESP.getMaxAllocHeap(), url.c_str());
      if (tls[0]) snprintf(err, errLen, "TLS: %s", tls); else snprintf(err, errLen, "%s", https.errorToString(code).c_str());
    } else snprintf(err, errLen, "HTTP %d", code);
    https.end();
    return code;
  }
  Stream& st = https.getStream();
  st.setTimeout(15000);
  if (!findAircraftArray(st)) { snprintf(err, errLen, "no aircraft list"); https.end(); return -3; }
  int n = 0; char c;
  auto peekWait = [&]() {                  // peek() doesn't wait for data to arrive
    uint32_t t0 = millis();
    while (!st.available() && millis() - t0 < 15000) delay(2);
    return st.peek();
  };
  while (true) {
    // Empty list, or the end after the last element
    while (peekWait() == ' ' || st.peek() == '\n' || st.peek() == '\r' || st.peek() == '\t') st.read();
    if (st.peek() == ']') break;
    JsonDocument el;
    DeserializationError e = deserializeJson(el, st, DeserializationOption::Filter(filter));
    if (e) {
      if (n == 0) { snprintf(err, errLen, "JSON %s", e.c_str()); https.end(); return -2; }
      Serial.printf("Aircraft list cut short after %d (%s)\n", n, e.c_str());
      break;
    }
    onAircraft(el.as<JsonObject>());
    n++;
    do { if (st.readBytes(&c, 1) != 1) c = ']'; } while (c == ' ' || c == '\n' || c == '\r' || c == '\t');
    if (c != ',') break;
  }
  https.end();
  return HTTP_CODE_OK;
}

// ---------------------------------------------------------------------------
// Today's log: unique aircraft per hour, records and notable sightings.
// Reset at local midnight; kept in RAM (starts over after a restart).
// ---------------------------------------------------------------------------
const int LOG_NOTABLE = 30;
struct LogRecord { char cs[10] = ""; char type[6] = ""; int alt = 0; float kt = 0; double km = 0; time_t at = 0; };
struct LogNotable { uint32_t hex; char cs[10]; char type[6]; AlertKind kind; time_t first; double km; int alt; };
struct DayLog {
  int yday = -1; time_t since = 0;
  uint8_t seenBits[4096];           // 32768-bit Bloom filter of ICAO addresses (a few % low at 3000/day)
  uint32_t total = 0;
  uint16_t hours[24];
  LogRecord lowest, fastest, closest;
  LogNotable notable[LOG_NOTABLE]; int nNotable = 0, moreNotable = 0;
} dayLog;
SemaphoreHandle_t logMutex = NULL;

// Starts a poll: resets the log on a new local day. False until the clock is set.
bool logBegin() {
  time_t now = time(nullptr);
  if (now < 1700000000) return false;
  struct tm lt; localtime_r(&now, &lt);
  if (lt.tm_yday != dayLog.yday) {
    memset(dayLog.seenBits, 0, sizeof(dayLog.seenBits));
    memset(dayLog.hours, 0, sizeof(dayLog.hours));
    dayLog.total = 0; dayLog.nNotable = dayLog.moreNotable = 0;
    dayLog.lowest = dayLog.fastest = dayLog.closest = LogRecord();
    dayLog.since = dayLog.yday < 0 ? now : now - (lt.tm_hour * 3600 + lt.tm_min * 60 + lt.tm_sec);
    dayLog.yday = lt.tm_yday;
  }
  return true;
}

bool logSeenNew(uint32_t h) {
  uint16_t a = (uint16_t)((h * 2654435761u) >> 17), b = (uint16_t)(((h ^ 0x5bd1e995u) * 0x9E3779B1u) >> 17);
  bool seen = (dayLog.seenBits[a >> 3] & (1 << (a & 7))) && (dayLog.seenBits[b >> 3] & (1 << (b & 7)));
  dayLog.seenBits[a >> 3] |= 1 << (a & 7); dayLog.seenBits[b >> 3] |= 1 << (b & 7);
  return !seen;
}

void logRecord(LogRecord& r, const char* cs, const char* type, int alt, float kt, double km, time_t now) {
  copyAscii(r.cs, cs, sizeof(r.cs)); copyAscii(r.type, type, sizeof(r.type));
  r.alt = alt; r.kt = kt; r.km = km; r.at = now;
}

// One aircraft from the current poll (call with logMutex held)
void logAircraft(const char* hexStr, const char* cs, const char* type, const char* category,
                 uint8_t dbFlags, int alt, float kt, double km, const WatchList& wl, bool emerg) {
  if (!hexStr[0]) return;
  uint32_t hex = strtoul(hexStr[0] == '~' ? hexStr + 1 : hexStr, nullptr, 16) | (hexStr[0] == '~' ? 0x1000000u : 0);
  time_t now = time(nullptr);
  if (logSeenNew(hex)) {
    struct tm lt; localtime_r(&now, &lt);
    dayLog.total++;
    dayLog.hours[lt.tm_hour]++;
  }
  if (alt > 0 && (!dayLog.lowest.at || alt < dayLog.lowest.alt)) logRecord(dayLog.lowest, cs, type, alt, kt, km, now);
  if (!dayLog.fastest.at || kt > dayLog.fastest.kt) logRecord(dayLog.fastest, cs, type, alt, kt, km, now);
  if (!dayLog.closest.at || km < dayLog.closest.km) logRecord(dayLog.closest, cs, type, alt, kt, km, now);

  // Notable: watch list, military, government/rare, helicopters (regardless of LED settings)
  LedConfig all; all.military = all.interesting = all.heli = true; all.lowAltFt = 0;
  AlertKind k = emerg ? ALERT_EMERG : classifyAlert(all, wl, cs, type, category, dbFlags, alt);
  if (k == ALERT_NONE) return;
  for (int i = 0; i < dayLog.nNotable; i++) {
    LogNotable& n = dayLog.notable[i];
    if (n.hex == hex) { if (km < n.km) { n.km = km; n.alt = alt; } return; }
  }
  if (dayLog.nNotable >= LOG_NOTABLE) { dayLog.moreNotable++; return; }
  LogNotable& n = dayLog.notable[dayLog.nNotable++];
  n.hex = hex; copyAscii(n.cs, cs[0] ? cs : "?", sizeof(n.cs)); copyAscii(n.type, type, sizeof(n.type));
  n.kind = k; n.first = now; n.km = km; n.alt = alt;
}

// ---------------------------------------------------------------------------
// Phone notifications: candidates are collected while parsing, sent after the poll
// ---------------------------------------------------------------------------
struct NotifyItem { char hex[8]; char cs[10]; char type[6]; char reg[10]; AlertKind kind; double km, brg; int alt; float kt; };
NotifyItem notifyCand[4]; int notifyCandN = 0;   // fetch task only

void notifyConsider(const NotifyConfig& nc, const WatchList& wl, const WatchList& noWatch, const char* hex,
                    const char* cs, const char* type, const char* reg, const char* category, uint8_t dbFlags,
                    int alt, float kt, double km, double brg) {
  if (!hex[0] || km > nc.range * KM_PER_UNIT) return;
  LedConfig t;
  t.military = nc.cats & NT_MIL; t.interesting = nc.cats & NT_INTR; t.heli = nc.cats & NT_HELI;
  t.lowAltFt = (nc.cats & NT_LOW) ? nc.lowAltFt : 0;
  AlertKind k = classifyAlert(t, (nc.cats & NT_WATCH) ? wl : noWatch, cs, type, category, dbFlags, alt);
  if (k == ALERT_NONE) return;
  int slot = notifyCandN < 4 ? notifyCandN++ : -1;
  if (slot < 0) {                        // full: replace the farthest if this one is nearer
    double far = km;
    for (int i = 0; i < 4; i++) if (notifyCand[i].km > far) { far = notifyCand[i].km; slot = i; }
    if (slot < 0) return;
  }
  NotifyItem& n = notifyCand[slot];
  copyAscii(n.hex, hex, sizeof(n.hex)); copyAscii(n.cs, cs[0] ? cs : "?", sizeof(n.cs));
  copyAscii(n.type, type, sizeof(n.type)); copyAscii(n.reg, reg, sizeof(n.reg));
  n.kind = k; n.km = km; n.brg = brg; n.alt = alt; n.kt = kt;
}

// ---------------------------------------------------------------------------
// Type collection: every ICAO aircraft type seen, kept in LittleFS (/types.csv).
// RAM holds only the packed type codes (4 bytes each) for the "is it new?" check.
// ---------------------------------------------------------------------------
const int COL_MAX = 700;
uint32_t colTypes[COL_MAX]; int colCount = 0;   // sorted; fetch task only (after setup)
bool fsOk = false;
volatile bool collectionResetRequested = false;
uint32_t colStart = 0;                          // when the collection started (NVS "col_start")
struct NewType { char type[6], hex[8], cs[10], reg[10]; };
NewType newTypes[4]; int newTypeN = 0;          // found during a poll, saved after it
char newTypeBanner[48] = ""; uint32_t newTypeBannerUntil = 0;

uint32_t packType(const char* t) {
  uint32_t v = 0;
  for (int i = 0; i < 4 && t[i]; i++) v |= (uint32_t)(uint8_t)t[i] << (24 - 8 * i);
  return v;
}

// Inserts into the sorted list; false if it was already there (or the list is full)
bool colInsert(uint32_t v) {
  int lo = 0, hi = colCount;
  while (lo < hi) { int m = (lo + hi) / 2; if (colTypes[m] < v) lo = m + 1; else hi = m; }
  if (lo < colCount && colTypes[lo] == v) return false;
  if (colCount >= COL_MAX) return false;
  memmove(colTypes + lo + 1, colTypes + lo, (colCount - lo) * sizeof(uint32_t));
  colTypes[lo] = v; colCount++;
  return true;
}

void collectionLoad() {
  colCount = 0;
  File f = LittleFS.open("/types.csv", "r");
  if (!f) return;
  char line[160];
  while (f.available()) {
    int n = f.readBytesUntil('\n', line, sizeof(line) - 1); line[n] = '\0';
    char* c = strchr(line, ','); if (!c) continue;
    *c = '\0';
    colInsert(packType(line));
  }
  f.close();
  Serial.printf("Type collection: %d types\n", colCount);
}

// Called while parsing a poll
void collectionConsider(const char* type, const char* hex, const char* cs, const char* reg) {
  if (!fsOk || strlen(type) < 2 || time(nullptr) < 1700000000) return;
  if (newTypeN >= 4) return;                    // the rest waits for the next poll
  uint32_t v = packType(type);
  if (!colInsert(v)) return;
  NewType& n = newTypes[newTypeN++];
  copyAscii(n.type, type, sizeof(n.type)); copyAscii(n.hex, hex, sizeof(n.hex));
  copyAscii(n.cs, cs, sizeof(n.cs)); copyAscii(n.reg, reg, sizeof(n.reg));
}

// Fetches aircraft data into caller-provided local buffers (thread-safe — no globals touched)
bool fetchAircraftTo(Aircraft* top5Out, uint8_t& top5CntOut,
                     Aircraft& nearOut, Blip* blipsOut, uint8_t& blipCntOut,
                     AlertInfo& alertOut, Emergency& emOut, Aircraft& selOut) {
  // Snapshot config under mutex (Core 0 reads, Core 1 may write via web UI)
  double hLat, hLon; float sRadNm; LedConfig lc;
  if (configMutex) xSemaphoreTake(configMutex, portMAX_DELAY);
  hLat = homeLat; hLon = homeLon; sRadNm = searchRadiusNm; lc = ledCfg;
  if (configMutex) xSemaphoreGive(configMutex);

  WatchList wl; wl.parse(lc.watch);
  double alertKm = lc.alertRange * KM_PER_UNIT;
  alertOut = AlertInfo();
  emOut = Emergency();
  selOut.valid = false;
  char selHex[8];
  if (dataMutex) xSemaphoreTake(dataMutex, portMAX_DELAY);
  strcpy(selHex, (selectedUntil && (int32_t)(selectedUntil - millis()) > 0) ? selectedHex : "");
  if (dataMutex) xSemaphoreGive(dataMutex);

  // Query area: the configured search radius, widened to the radar range when that is larger
  float rRange;
  if (configMutex) xSemaphoreTake(configMutex, portMAX_DELAY);
  rRange = radarRange;
  if (configMutex) xSemaphoreGive(configMutex);
  double radarKm = rRange * KM_PER_UNIT, searchKm = sRadNm * 1.852;
  float queryNm = max(sRadNm, (float)(radarKm / 1.852));
  if (queryNm > MAX_SEARCH_NM) queryNm = MAX_SEARCH_NM;

  // Only the fields we use; the aircraft are parsed one at a time from the stream,
  // so even a few hundred aircraft need just one small document at a time
  JsonDocument filter;
  for (const char* fld : {"hex", "flight", "r", "t", "lat", "lon", "alt_baro", "alt_geom", "gs",
                          "track", "baro_rate", "geom_rate", "category", "dbFlags", "squawk", "emergency"})
    filter[fld] = true;

  uint16_t count = 0, shown = 0;
  bool onlyHeli = heliOnly;
  NotifyConfig nc;
  if (configMutex) xSemaphoreTake(configMutex, portMAX_DELAY);
  nc = notifyCfg;
  if (configMutex) xSemaphoreGive(configMutex);
  WatchList noWatch;
  bool logging = logMutex && logBegin();

  auto onAircraft = [&](JsonObject s) {
    if (!s["lat"].is<double>() || !s["lon"].is<double>()) return;

    // Skip aircraft on the ground (alt_baro == "ground") — only track those in flight
    if (s["alt_baro"].is<const char*>()) return;

    double lat = s["lat"].as<double>(), lon = s["lon"].as<double>();
    double d = haversineKm(hLat, hLon, lat, lon);
    double brg = bearingDeg(hLat, hLon, lat, lon);
    int altFt = s["alt_baro"].is<int>() ? s["alt_baro"].as<int>() : (s["alt_geom"] | 0);
    float gsKt = s["gs"] | 0.0f;
    float track = s["track"] | 0.0f;
    char cs[10];
    copyAscii(cs, s["flight"] | "", sizeof(cs));
    if (cs[0] == '\0') copyAscii(cs, s["r"] | "", sizeof(cs));  // fall back to registration
    char type[6], category[3];
    copyAscii(type, s["t"] | "", sizeof(type));
    copyAscii(category, s["category"] | "", sizeof(category));
    uint8_t dbFlags = s["dbFlags"] | 0;
    count++;
    bool inSearch = d <= searchKm;     // log, collection and emergencies use the configured area

    // Emergency: squawk 7500/7600/7700 or a readsb emergency state
    const char* sq = s["squawk"] | "";
    const char* em = s["emergency"] | "none";
    const char* what = nullptr;
    if (!strcmp(sq, "7700")) what = "EMERGENCY";
    else if (!strcmp(sq, "7600")) what = "RADIO FAILURE";
    else if (!strcmp(sq, "7500")) what = "HIJACK";
    else if (!strcmp(em, "general")) what = "EMERGENCY";
    else if (!strcmp(em, "lifeguard")) what = "MEDICAL";
    else if (!strcmp(em, "minfuel")) what = "MIN FUEL";
    else if (!strcmp(em, "nordo")) what = "RADIO FAILURE";
    else if (!strcmp(em, "unlawful")) what = "HIJACK";
    else if (!strcmp(em, "downed")) what = "DOWNED";
    if (what && inSearch && (!emOut.on || d < emOut.km)) {
      emOut.on = true; emOut.km = d; emOut.brg = brg; emOut.alt = altFt;
      copyAscii(emOut.cs, cs[0] ? cs : "?", sizeof(emOut.cs)); copyAscii(emOut.hex, s["hex"] | "", sizeof(emOut.hex));
      copyAscii(emOut.type, type, sizeof(emOut.type)); copyAscii(emOut.squawk, sq, sizeof(emOut.squawk));
      copyAscii(emOut.what, what, sizeof(emOut.what));
    }

    if (inSearch) {
      if (logging) {
        xSemaphoreTake(logMutex, portMAX_DELAY);
        logAircraft(s["hex"] | "", cs, type, category, dbFlags, altFt, gsKt, d, wl, what != nullptr);
        xSemaphoreGive(logMutex);
      }
      collectionConsider(type, s["hex"] | "", cs, s["r"] | "");
    }
    if (nc.on && nc.topic[0])
      notifyConsider(nc, wl, noWatch, s["hex"] | "", cs, type, s["r"] | "", category, dbFlags, altFt, gsKt, d, brg);

    // LED alerts: keep the highest-priority (then nearest) special aircraft in range
    if (lc.alerts && d <= alertKm) {
      AlertKind k = classifyAlert(lc, wl, cs, type, category, dbFlags, altFt);
      if (k != ALERT_NONE && (k > alertOut.kind || (k == alertOut.kind && d < alertOut.distKm))) {
        alertOut.kind = k; alertOut.distKm = d;
        strcpy(alertOut.callsign, cs[0] ? cs : "?");
        strcpy(alertOut.type, type);
      }
    }

    // Helicopters only: everything above (alerts, log, collection, emergencies) still sees
    // all aircraft; the radar, traffic list and nearest card show helicopters only
    if (onlyHeli && strcmp(category, "A7") != 0) return;
    shown++;

    // Radar: up to MAX_BLIPS aircraft inside the radar range, spread over 4 distance rings
    // (10 per ring, nearest first) so a wide range doesn't show only the centre
    if (d <= radarKm) {
      uint8_t ring = (uint8_t)min(3.0, d / radarKm * 4);
      int inRing = 0, far = -1; double farD = -1;
      for (int i = 0; i < blipCntOut; i++) if (blipsOut[i].ring == ring) {
        inRing++;
        if (blipsOut[i].distKm > farD) { farD = blipsOut[i].distKm; far = i; }
      }
      int slot = -1;
      if (inRing < MAX_BLIPS / 4 && blipCntOut < MAX_BLIPS) slot = blipCntOut++;
      else if (far >= 0 && d < farD) slot = far;
      if (slot >= 0) {
        Blip& b = blipsOut[slot];
        b.lat = lat; b.lon = lon; b.track = track; b.speedKt = gsKt;
        b.altFt = altFt; b.distKm = d; b.ring = ring;
        strcpy(b.callsign, cs);
        copyAscii(b.hex, s["hex"] | "", sizeof(b.hex));
        b.heli = strcmp(category, "A7") == 0;
      }
    }

    Aircraft a;
    a.distanceKm = d; a.lat = lat; a.lon = lon; a.bearingDeg = brg;
    a.onGround = false; a.altFt = altFt; a.gsKt = gsKt; a.trackDeg = track;
    a.vrateFpm = s["baro_rate"].is<int>() ? s["baro_rate"].as<int>() : (s["geom_rate"] | 0);
    a.hasRoute = false; a.dep[0] = a.arr[0] = a.airline[0] = '\0';
    a.valid = true;
    strcpy(a.callsign, cs[0] ? cs : "(no id)");
    strcpy(a.type, type);
    copyAscii(a.hex, s["hex"] | "", sizeof(a.hex));
    a.flightIata[0] = a.depCity[0] = a.arrCity[0] = a.model[0] = '\0';
    a.hasCoords = false;
    strcpy(a.category, category);
    a.dbFlags = dbFlags;
    copyAscii(a.reg, s["r"] | "", sizeof(a.reg));
    if (selHex[0] && strcmp(a.hex, selHex) == 0) selOut = a;   // the tapped aircraft, wherever it is

    int insertIdx = -1;
    for (int i = 0; i < MAX_TOP5; i++) {
      if (i >= top5CntOut || a.distanceKm < top5Out[i].distanceKm) {
        insertIdx = i; break;
      }
    }
    if (insertIdx != -1) {
      for (int i = MAX_TOP5 - 1; i > insertIdx; i--) { top5Out[i] = top5Out[i-1]; }
      top5Out[insertIdx] = a;
      if (top5CntOut < MAX_TOP5) top5CntOut++;
    }
  };

  // Try the last source that worked first, then the others
  bool ok = false;
  int start = activeSource;
  for (int k = 0; k < NUM_SOURCES && !ok; k++) {
    int i = (start + k) % NUM_SOURCES;
    String url = sourceUrl(i, hLat, hLon, (int)ceilf(queryNm));
    char err[72] = "";
    count = 0; shown = 0; blipCntOut = 0; top5CntOut = 0; notifyCandN = 0;     // a failed source may have sent some
    alertOut = AlertInfo(); emOut = Emergency(); selOut.valid = false;
    if (streamAircraft(url, filter, onAircraft, err, sizeof(err)) == HTTP_CODE_OK) {
      ok = true;
      activeSource = i;
      lastOkMs = millis();
    } else {
      snprintf(lastFetchErr, sizeof(lastFetchErr), "%s: %s", SOURCES[i].name, err);
      Serial.printf("Aircraft source %s failed: %s\n", SOURCES[i].name, err);
    }
  }
  if (!ok) { stats.requestsFail++; return false; }

  stats.inView = onlyHeli ? shown : count; if (stats.inView > stats.maxInView) stats.maxInView = stats.inView;
  stats.requestsOk++;

  if (top5CntOut > 0) {
    nearOut = top5Out[0];
  } else {
    nearOut.valid = false;
  }

  return true;
}

bool fetchWeatherTo(Weather& w) {
  double hLat, hLon;
  if (configMutex) xSemaphoreTake(configMutex, portMAX_DELAY);
  hLat = homeLat; hLon = homeLon;
  if (configMutex) xSemaphoreGive(configMutex);

  String url = "https://api.open-meteo.com/v1/forecast?latitude=" + String(hLat, 4) +
               "&longitude=" + String(hLon, 4) +
               "&current=temperature_2m,relative_humidity_2m,wind_speed_10m,wind_direction_10m,weather_code,pressure_msl"
               "&daily=sunrise,sunset&forecast_days=1&timezone=auto&timeformat=unixtime";
#if UNITS_IMPERIAL
  url += "&temperature_unit=fahrenheit&wind_speed_unit=mph";
#endif

  JsonDocument doc;
  if (httpGetJson(url, doc, nullptr) != HTTP_CODE_OK) return false;
  JsonObject c = doc["current"]; if (c.isNull()) return false;

  w.temp = c["temperature_2m"] | 0.0f;
  w.humidity = c["relative_humidity_2m"] | 0;
  w.wind = c["wind_speed_10m"] | 0.0f;
  w.windDir = c["wind_direction_10m"] | 0;
  w.code = c["weather_code"] | 0;
  w.pressure = c["pressure_msl"] | 0.0f;
  w.sunrise = doc["daily"]["sunrise"][0] | 0;
  w.sunset = doc["daily"]["sunset"][0] | 0;
  w.valid = true;
  return true;
}

// ---------------------------------------------------------------------------
// Update from GitHub: latest release tag, then <env>-firmware.bin from that release
// ---------------------------------------------------------------------------
void drawOtaScreen(const char* line1, const char* line2, int pct);

void checkLatestRelease() {
  ghState = GH_CHECKING;
  JsonDocument filter; filter["tag_name"] = true;
  JsonDocument d;
  char err[48] = "";
  int code = httpGetJson("https://api.github.com/repos/" GITHUB_REPO "/releases/latest", d, &filter, err, sizeof(err));
  ghCheckedMs = millis() | 1;
  if (code == HTTP_CODE_OK && d["tag_name"].is<const char*>()) {
    copyAscii(ghLatest, d["tag_name"].as<const char*>(), sizeof(ghLatest));
    ghMsg[0] = '\0';
    Serial.printf("Latest release: %s (running %s)\n", ghLatest, FW_VERSION);
  } else if (code == HTTP_CODE_NOT_FOUND) {
    snprintf(ghMsg, sizeof(ghMsg), "no releases yet");
  } else {
    snprintf(ghMsg, sizeof(ghMsg), "check failed: %s", err);
  }
  ghState = GH_IDLE;
}

void runGithubUpdate() {
  if (!ghLatest[0]) checkLatestRelease();
  if (!ghLatest[0]) { ghState = GH_FAILED; return; }
  String url = String("https://github.com/" GITHUB_REPO "/releases/download/") + ghLatest +
               "/" BUILD_ENV "-firmware.bin";
  Serial.printf("Updating from %s\n", url.c_str());
  ghState = GH_DOWNLOADING; ghPct = 0;
  otaActive = true;                      // loop() stops drawing; wait for a frame to finish
  vTaskDelay(pdMS_TO_TICKS(300));
  drawOtaScreen("Downloading update...", ghLatest, 0);

  WiFiClientSecure client; client.setInsecure();
  httpUpdate.setFollowRedirects(HTTPC_FORCE_FOLLOW_REDIRECTS);   // GitHub redirects to its CDN
  httpUpdate.rebootOnUpdate(false);
  httpUpdate.onProgress([](int done, int total) {
    ghPct = total > 0 ? (int)(done * 100LL / total) : 0;
    drawOtaScreen("Downloading update...", ghLatest, ghPct);
  });
  t_httpUpdate_return r = httpUpdate.update(client, url);
  if (r == HTTP_UPDATE_OK) {
    drawOtaScreen("Update complete", "Restarting...", 100);
    vTaskDelay(pdMS_TO_TICKS(1000));
    ESP.restart();
  }
  snprintf(ghMsg, sizeof(ghMsg), "update failed: %s", httpUpdate.getLastErrorString().c_str());
  Serial.println(ghMsg);
  ghState = GH_FAILED;
  drawOtaScreen("Update failed", ghMsg, -1);
  vTaskDelay(pdMS_TO_TICKS(3000));
  otaActive = false; lastScreen = 255;
}

String withCommas(int v);
bool nightNow();

// METAR from aviationweather.gov: the configured station, or the nearest one to home
bool fetchMetarTo(Metar& m, char* err, size_t errLen) {
  char icao[5]; double hLat, hLon;
  if (configMutex) xSemaphoreTake(configMutex, portMAX_DELAY);
  strcpy(icao, scr.metarIcao); hLat = homeLat; hLon = homeLon;
  if (configMutex) xSemaphoreGive(configMutex);

  JsonDocument filter;
  for (const char* f : {"icaoId", "name", "obsTime", "temp", "dewp", "wdir", "wspd", "wgst", "visib",
                        "altim", "rawOb", "fltCat", "lat", "lon"})
    filter[0][f] = true;
  filter[0]["clouds"][0]["cover"] = true; filter[0]["clouds"][0]["base"] = true;

  JsonDocument d;
  for (double box : {0.5, 1.5}) {
    char url[160];
    if (icao[0]) snprintf(url, sizeof(url), "https://aviationweather.gov/api/data/metar?ids=%s&format=json", icao);
    else snprintf(url, sizeof(url), "https://aviationweather.gov/api/data/metar?bbox=%.2f,%.2f,%.2f,%.2f&format=json",
                  hLat - box, hLon - box, hLat + box, hLon + box);
    int code = httpGetJson(url, d, &filter, err, errLen);
    if (code == 204) { d.clear(); if (icao[0]) break; continue; }   // no reports: widen the box
    if (code != HTTP_CODE_OK) return false;
    if (d.as<JsonArray>().size() > 0 || icao[0]) break;
  }
  JsonArray a = d.as<JsonArray>();
  if (a.size() == 0) { snprintf(err, errLen, icao[0] ? "no report for %s" : "no station nearby", icao); return false; }

  JsonObject best; double bestKm = 1e9;
  for (JsonObject o : a) {
    double k = haversineKm(hLat, hLon, o["lat"] | 0.0, o["lon"] | 0.0);
    if (k < bestKm) { bestKm = k; best = o; }
  }
  m = Metar();
  copyAscii(m.id, best["icaoId"] | "", sizeof(m.id));
  copyAscii(m.name, best["name"] | "", sizeof(m.name));
  copyAscii(m.cat, best["fltCat"] | "", sizeof(m.cat));
  m.obs = best["obsTime"] | 0;
  m.temp = best["temp"] | 0.0f; m.dewp = best["dewp"] | 0.0f; m.altim = best["altim"] | 0.0f;
  m.wdir = best["wdir"].is<int>() ? best["wdir"].as<int>() : -1;     // -1 = variable
  m.wspd = best["wspd"] | 0; m.wgst = best["wgst"] | 0;
  if (best["visib"].is<const char*>()) copyAscii(m.visib, best["visib"], sizeof(m.visib));
  else { char vb[24]; snprintf(vb, sizeof(vb), "%g", best["visib"] | 0.0); copyAscii(m.visib, vb, sizeof(m.visib)); }
  // Ceiling: lowest broken or overcast layer
  m.ceiling[0] = '\0';
  for (JsonObject c : best["clouds"].as<JsonArray>()) {
    const char* cv = c["cover"] | "";
    if (!strcmp(cv, "BKN") || !strcmp(cv, "OVC") || !strcmp(cv, "OVX")) {
      snprintf(m.ceiling, sizeof(m.ceiling), "%s %s", cv, withCommas(c["base"] | 0).c_str());
      break;
    }
  }
  if (!m.ceiling[0]) snprintf(m.ceiling, sizeof(m.ceiling), "None");
  copyAscii(m.raw, best["rawOb"] | "", sizeof(m.raw));
  m.valid = true;
  return true;
}

// Saves the types found in the last poll (with the model name from adsbdb) and announces them
void collectionSaveNew(const struct NotifyConfig& nc);

// POST one message to <server>/<topic>. Returns the HTTP status (200 = sent).
int ntfySend(const NotifyConfig& nc, const char* title, const String& body, const char* tags,
             const String& click, int priority) {
  String url = String(nc.server) + "/" + nc.topic;
  WiFiClientSecure sc; sc.setInsecure();
  WiFiClient pc;
  WiFiClient& cl = url.startsWith("https://") ? (WiFiClient&)sc : pc;
  HTTPClient h;
  h.setTimeout(10000);
  h.setUserAgent("CYD28PlaneSpotter/1.2 (+https://github.com/argonfatl/CYD28PlaneSpotter)");
  if (!h.begin(cl, url)) { snprintf(notifyStatus, sizeof(notifyStatus), "Error: bad server address"); return -1; }
  h.addHeader("Title", title);
  h.addHeader("Tags", tags);
  h.addHeader("Priority", String(priority));
  if (click.length()) h.addHeader("Click", click);
  int code = h.POST(body);
  h.end();
  if (code != HTTP_CODE_OK) {
    if (code < 0) snprintf(notifyStatus, sizeof(notifyStatus), "Error: %s", h.errorToString(code).c_str());
    else snprintf(notifyStatus, sizeof(notifyStatus), "Error: HTTP %d", code);
    Serial.printf("ntfy: %s\n", notifyStatus);
  }
  return code;
}

// After each poll: send candidates not notified within the cooldown (at most 3 per poll)
void processNotifications(const Emergency& em) {
  NotifyConfig nc;
  if (configMutex) xSemaphoreTake(configMutex, portMAX_DELAY);
  nc = notifyCfg;
  if (configMutex) xSemaphoreGive(configMutex);

  if (notifyTestRequested) {
    notifyTestRequested = false;
    if (!nc.topic[0]) snprintf(notifyStatus, sizeof(notifyStatus), "Error: no topic set");
    else if (ntfySend(nc, "CYD Plane Spotter test", String("Notifications work. Location: ") + homeName,
                      "airplane,white_check_mark", "", 3) == HTTP_CODE_OK)
      snprintf(notifyStatus, sizeof(notifyStatus), "Test sent");
  }
  if (!nc.on || !nc.topic[0]) return;

  // Emergencies: any distance, even at night, once an hour per aircraft
  static char emHex[8] = ""; static uint32_t emMs = 0;
  if ((nc.cats & NT_EMERG) && em.on && (strcmp(emHex, em.hex) != 0 || millis() - emMs > 3600000UL)) {
    char title[64], body[128];
    if (!strcmp(em.squawk, "7500") || !strcmp(em.squawk, "7600") || !strcmp(em.squawk, "7700")) snprintf(title, sizeof(title), "Squawk %s %s: %s", em.squawk, em.what, em.cs);
    else snprintf(title, sizeof(title), "%s: %s", em.what, em.cs);
    if (em.type[0]) snprintf(title + strlen(title), sizeof(title) - strlen(title), " (%s)", em.type);
    snprintf(body, sizeof(body), "%.1f %s %s, %s ft", em.km / KM_PER_UNIT, DIST_UNIT, compass(em.brg), withCommas(em.alt).c_str());
    if (ntfySend(nc, title, body, "rotating_light,airplane", String("https://globe.airplanes.live/?icao=") + em.hex, 5) == HTTP_CODE_OK) {
      strcpy(emHex, em.hex); emMs = millis();
      snprintf(notifyStatus, sizeof(notifyStatus), "Sent emergency %s", em.cs);
    }
  }

  if (notifyCandN == 0) return;
  if (nc.quietNight && nightNow()) return;

  struct Sent { char hex[8]; uint32_t ms; };
  static Sent sent[32] = {};
  uint32_t cool = (uint32_t)nc.coolH * 3600000UL, now = millis();
  int done = 0;
  for (int i = 0; i < notifyCandN && done < 3; i++) {
    NotifyItem& n = notifyCand[i];
    int idx = -1, oldest = 0;
    for (int j = 0; j < 32; j++) {
      if (sent[j].hex[0] && strcmp(sent[j].hex, n.hex) == 0) { idx = j; break; }
      if (now - sent[j].ms > now - sent[oldest].ms || !sent[j].hex[0]) oldest = j;
    }
    if (idx >= 0 && now - sent[idx].ms < cool) continue;

    static const char* what[] = {"", "Low flying", "Helicopter", "Rare aircraft", "Military", "Watch list", "Emergency"};
    static const char* tags[] = {"", "airplane,arrow_down", "helicopter", "airplane,star", "airplane,warning", "airplane,eyes", "rotating_light"};
    char title[64];
    if (n.type[0]) snprintf(title, sizeof(title), "%s nearby: %s (%s)", what[n.kind], n.cs, n.type);
    else snprintf(title, sizeof(title), "%s nearby: %s", what[n.kind], n.cs);
    char body[128];
    snprintf(body, sizeof(body), "%.1f %s %s, %s ft, %d kt%s%s", n.km / KM_PER_UNIT, DIST_UNIT, compass(n.brg),
             withCommas(n.alt).c_str(), (int)lroundf(n.kt), n.reg[0] ? ", reg " : "", n.reg);
    String click = String("https://globe.airplanes.live/?icao=") + n.hex;
    if (ntfySend(nc, title, body, tags[n.kind], click, n.kind == ALERT_WATCH ? 4 : 3) == HTTP_CODE_OK) {
      Sent& e = sent[idx >= 0 ? idx : oldest];
      strcpy(e.hex, n.hex); e.ms = now;
      time_t t = time(nullptr); struct tm lt; localtime_r(&t, &lt);
      snprintf(notifyStatus, sizeof(notifyStatus), "Sent %s at %02d:%02d", n.cs, lt.tm_hour, lt.tm_min);
      Serial.printf("ntfy: %s - %s\n", title, body);
    }
    done++;
  }
}

void collectionSaveNew(const NotifyConfig& nc) {
  if (newTypeN && !colStart && time(nullptr) > 1700000000) { colStart = time(nullptr); prefs.putUInt("col_start", colStart); }
  for (int i = 0; i < newTypeN; i++) {
    NewType& n = newTypes[i];
    Aircraft a = {}; copyAscii(a.hex, n.hex, sizeof(a.hex));
    getAircraftModel(a);                      // "Airbus A320-251N" (cached, may be empty)
    char model[32]; copyAscii(model, a.model, sizeof(model));
    for (char* c = model; *c; c++) if (*c == ',') *c = ' ';
    File f = LittleFS.open("/types.csv", "a");
    if (f) { f.printf("%s,%lu,%s,%s,%s,%s\n", n.type, (unsigned long)time(nullptr), n.hex, n.cs, n.reg, model); f.close(); }
    Serial.printf("New type for the collection: %s %s (%s)\n", n.type, model, n.cs);
    snprintf(newTypeBanner, sizeof(newTypeBanner), "NEW TYPE %s  %s", n.type, model[0] ? model : n.cs);
    newTypeBannerUntil = millis() + 30000;

    // The first day fills the collection with everything common, so stay quiet then
    bool learning = !colStart || (uint32_t)time(nullptr) - colStart < 86400;
    if (nc.on && nc.topic[0] && (nc.cats & NT_NEWTYPE) && !learning && !(nc.quietNight && nightNow())) {
      char title[64], body[96];
      snprintf(title, sizeof(title), "New for your collection: %s", n.type);
      snprintf(body, sizeof(body), "%s%s%s, %d types so far", model[0] ? model : "Unknown model", n.cs[0] ? " - " : "", n.cs, colCount);
      ntfySend(nc, title, body, "airplane,tada", String("https://globe.airplanes.live/?icao=") + n.hex, 3);
    }
  }
  newTypeN = 0;
}

// ===================================================================
// Background Data Fetcher — runs on Core 0 so Core 1 stays smooth
// ===================================================================
void dataFetcherTask(void* param) {
  // Local work buffers — all blocking HTTP happens here
  Aircraft locTop5[MAX_TOP5];
  Blip locBlips[MAX_BLIPS];
  uint8_t locTop5Count = 0;
  uint8_t locBlipCount = 0;
  Aircraft locNearest;
  AlertInfo locAlert;
  Emergency locEm;
  Aircraft locSel;
  locNearest.valid = false;
  uint32_t lastMetarMs = 0; bool metarDone = false;

  uint32_t lastWeatherMs = 0;
  bool weatherDone = false;

  for (;;) {
    if (otaActive || portalActive) { vTaskDelay(pdMS_TO_TICKS(500)); continue; }
    refreshRequested = false;
    if (WiFi.status() == WL_CONNECTED) {
      // Update from GitHub (web page), and the release check (on request, at boot, twice a day)
      if (ghUpdateRequested) { ghUpdateRequested = false; runGithubUpdate(); }
      if (ghCheckRequested || !ghCheckedMs || millis() - ghCheckedMs >= GH_CHECK_INTERVAL_MS) {
        ghCheckRequested = false;
        checkLatestRelease();
      }

      // "Locate by IP" from the settings screen
      if (ipLocateRequested) {
        ipLocateRequested = false;
        setStatus("Locating...");
        if (locateByIp()) { setStatus("Location updated"); weatherDone = false; }
        else setStatus("IP lookup failed");
        refreshRequested = false;
      }

      // Weather (every WEATHER_INTERVAL_MS) — previously blocked the UI loop
      if (!weatherDone || millis() - lastWeatherMs >= WEATHER_INTERVAL_MS) {
        Weather w;
        if (fetchWeatherTo(w)) {
          xSemaphoreTake(dataMutex, portMAX_DELAY);
          weather = w;
          xSemaphoreGive(dataMutex);
        }
        lastWeatherMs = millis();
        weatherDone = true;
      }

      // METAR (every 10 min while a METAR screen is enabled, or right after a settings change)
      if (scr.metarShow != SHOW_OFF && (metarRequested || !metarDone || millis() - lastMetarMs >= 600000UL)) {
        metarRequested = false;
        Metar m; char err[48] = "";
        if (fetchMetarTo(m, err, sizeof(err))) {
          xSemaphoreTake(dataMutex, portMAX_DELAY); metar = m; xSemaphoreGive(dataMutex);
          metarErr[0] = '\0';
        } else {
          snprintf(metarErr, sizeof(metarErr), "%s", err[0] ? err : "request failed");
        }
        lastMetarMs = millis(); metarDone = true;
      }

      if (collectionResetRequested) {
        collectionResetRequested = false;
        if (fsOk) LittleFS.remove("/types.csv");
        colCount = 0; newTypeN = 0; colStart = 0; prefs.remove("col_start");
        Serial.println("Type collection cleared");
      }

      selectRequested = false;
      if (fetchAircraftTo(locTop5, locTop5Count, locNearest, locBlips, locBlipCount, locAlert, locEm, locSel)) {
        // Routes for the top 5 (cached, so usually no network call)
        for (int i = 0; i < locTop5Count; i++)
          locTop5[i].hasRoute = getRoute(locTop5[i]);
        if (locSel.valid) locSel.hasRoute = getRoute(locSel);
        // Check every route against the aircraft's position (and ask adsb.lol too)
        Aircraft* withRoute[MAX_TOP5 + 1]; int nr = 0;
        for (int i = 0; i < locTop5Count; i++) withRoute[nr++] = &locTop5[i];
        if (locSel.valid) withRoute[nr++] = &locSel;
        routesetLookup(withRoute, nr);
        for (int i = 0; i < nr; i++) chooseRoute(*withRoute[i]);
        if (locTop5Count > 0) { getAircraftModel(locTop5[0]); locNearest = locTop5[0]; }
        if (locSel.valid) getAircraftModel(locSel);

        // Atomically publish results to the globals used by render()
        xSemaphoreTake(dataMutex, portMAX_DELAY);
        memcpy(blips, locBlips, sizeof(locBlips));
        blipCount = locBlipCount;
        memcpy(top5, locTop5, sizeof(locTop5));
        top5Count = locTop5Count;
        nearest = locNearest;
        currentAlert = locAlert;
        if (locAlert.kind != ALERT_NONE) { lastAlert = locAlert; lastAlertMs = millis(); }
        emergency = locEm;
        if (locSel.valid) selAc = locSel;
        lastDataMs = millis();
        stats.lastUpdateMs = lastDataMs;
        // Trails: add each radar aircraft's position; forget aircraft gone for 2 minutes
        for (int i = 0; i < locBlipCount; i++) {
          const Blip& b = locBlips[i];
          if (!b.hex[0]) continue;
          int t = -1, freeT = -1;
          for (int j = 0; j < 40; j++) {
            if (trails[j].hex[0] && strcmp(trails[j].hex, b.hex) == 0) { t = j; break; }
            if (freeT < 0 && (!trails[j].hex[0] || millis() - trails[j].seenMs > 120000UL)) freeT = j;
          }
          if (t < 0) { if (freeT < 0) continue; t = freeT; strcpy(trails[t].hex, b.hex); trails[t].n = 0; }
          Trail& tr = trails[t];
          if (tr.n == TRAIL_PTS) { memmove(tr.lat, tr.lat + 1, sizeof(float) * (TRAIL_PTS - 1)); memmove(tr.lon, tr.lon + 1, sizeof(float) * (TRAIL_PTS - 1)); tr.n--; }
          tr.lat[tr.n] = b.lat; tr.lon[tr.n] = b.lon; tr.n++;
          tr.seenMs = millis();
        }
        xSemaphoreGive(dataMutex);

        processNotifications(locEm);
        NotifyConfig nc;
        if (configMutex) xSemaphoreTake(configMutex, portMAX_DELAY);
        nc = notifyCfg;
        if (configMutex) xSemaphoreGive(configMutex);
        collectionSaveNew(nc);
      } else if (notifyTestRequested) {
        notifyCandN = 0;
        processNotifications(Emergency());
      }
    }

    // Wait for the next poll interval; a location change or button press cuts it short
    for (int t = 0; t < UPDATE_INTERVAL_MS / 250; t++) {
      if (refreshRequested || ipLocateRequested || ghCheckRequested || ghUpdateRequested || notifyTestRequested ||
          selectRequested || metarRequested) {
        if (refreshRequested) weatherDone = false;  // new location -> new weather too
        break;
      }
      vTaskDelay(pdMS_TO_TICKS(250));
    }
  }
}

// ---------------------------------------------------------------------------
// UI & Drawing
// ---------------------------------------------------------------------------
// True until the deadline passes; safe across the 49-day millis() rollover
bool stillBefore(uint32_t deadline) { return deadline && (int32_t)(deadline - millis()) > 0; }

bool timeReady() { return time(nullptr) > 1700000000; }
void fmtClock(char* buf, size_t n, bool withSecs) {
  if (!timeReady()) { strncpy(buf, withSecs ? "--:--:--" : "--:--", n); return; }
  time_t t = time(nullptr); struct tm lt; localtime_r(&t, &lt);
  strftime(buf, n, withSecs ? "%H:%M:%S" : "%H:%M", &lt);
}

// ===========================================================================
// UI — design system shared by all screens; colors come from the active theme
// ===========================================================================
constexpr uint16_t rgb(uint8_t r, uint8_t g, uint8_t b) {
  return (uint16_t)(((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3));
}

struct Theme {
  const char* name;
  uint16_t bg, panel, edge, accent, track, soft, label, badge, blueb;
  uint16_t aircraft, sun, pillOk, text, warn;
  // The radar sprite is 8-bit (RGB332): these keep only their top bits,
  // so they are picked to survive that conversion
  uint16_t scope, grid8, ring8, dim8;
};

const Theme THEMES[] = {
  // name           bg                panel             edge              accent            track
  //                soft              label             badge             blueb
  //                aircraft          sun               pillOk            text              warn
  //                scope             grid8             ring8             dim8
  {"Navy",        0x08A4,           0x08E5,           0x19CA,           0x065F,           0x21C9,
                  0x8E1C,           0x9D98,           0x11CC,           0x131C,
                  0x2F0D,           0xFE60,           0x3F2B,           0xFFFF,           TFT_ORANGE,
                  TFT_BLACK,        0x2310,           0x0495,           0x024A},
  {"Midnight",    rgb(0, 0, 0),     rgb(10, 14, 20),  rgb(38, 48, 64),  rgb(0, 200, 255), rgb(30, 38, 48),
                  rgb(154, 176, 196), rgb(138, 152, 168), rgb(20, 36, 58), rgb(30, 90, 200),
                  rgb(46, 224, 107), rgb(255, 204, 0), rgb(60, 230, 90), 0xFFFF,         TFT_ORANGE,
                  TFT_BLACK,        rgb(32, 64, 96),  rgb(0, 144, 192), rgb(0, 64, 96)},
  {"Radar Green", rgb(0, 8, 0),     rgb(0, 20, 0),    rgb(10, 74, 20),  rgb(51, 255, 102), rgb(10, 42, 16),
                  rgb(127, 212, 143), rgb(95, 170, 111), rgb(10, 58, 24), rgb(20, 140, 50),
                  rgb(204, 255, 102), rgb(224, 255, 128), rgb(51, 255, 102), rgb(200, 255, 208), rgb(255, 200, 0),
                  rgb(0, 0, 0),     rgb(0, 64, 0),    rgb(0, 192, 64),  rgb(0, 96, 32)},
  {"Amber",       rgb(10, 6, 0),    rgb(22, 12, 0),   rgb(74, 44, 0),   rgb(255, 176, 0), rgb(46, 28, 0),
                  rgb(224, 160, 80), rgb(176, 128, 64), rgb(58, 34, 0),  rgb(160, 96, 0),
                  rgb(255, 224, 128), rgb(255, 208, 64), rgb(255, 176, 0), rgb(255, 232, 192), rgb(255, 96, 32),
                  rgb(0, 0, 0),     rgb(96, 64, 0),   rgb(192, 128, 0), rgb(96, 64, 0)},
  {"Night Red",   rgb(0, 0, 0),     rgb(12, 0, 0),    rgb(64, 0, 0),    rgb(224, 32, 0),  rgb(32, 0, 0),
                  rgb(160, 48, 32), rgb(128, 32, 24), rgb(48, 0, 0),    rgb(96, 0, 0),
                  rgb(255, 64, 32), rgb(192, 48, 0),  rgb(192, 32, 0),  rgb(255, 80, 64), rgb(255, 128, 0),
                  rgb(0, 0, 0),     rgb(64, 0, 0),    rgb(160, 0, 0),   rgb(96, 0, 0)},
};
const uint8_t NUM_THEMES = sizeof(THEMES) / sizeof(THEMES[0]);
const uint8_t THEME_NIGHT_RED = 4;
uint8_t themeIdx = 0;                 // chosen theme (NVS "theme")
bool nightRedTheme = false;           // use Night Red while night mode is active (NVS "night_red")
const Theme* th = &THEMES[0];         // active theme

#define C_BG     (th->bg)        // background
#define C_PANEL  (th->panel)     // card fill
#define C_EDGE   (th->edge)      // card border
#define C_CYAN   (th->accent)    // accent
#define C_TRACK  (th->track)     // bars / unlit icon parts
#define C_SOFT   (th->soft)      // secondary text
#define C_LABEL  (th->label)     // labels
#define C_BADGE  (th->badge)     // dark accent fill
#define C_BLUEB  (th->blueb)     // bright accent fill (nav arrows)
#define C_GREEN  (th->aircraft)  // aircraft / OK
#define C_SUN    (th->sun)       // weather sun
#define C_PILLOK (th->pillOk)    // "connected" pill
#define C_TEXT   (th->text)      // primary text
#define C_WARN   (th->warn)      // warnings, military, descent
#define C_SCOPE  (th->scope)     // radar scope background
#define C_GRID8  (th->grid8)     // radar rings
#define C_RING8  (th->ring8)     // radar outer ring / sweep
#define C_DIM8   (th->dim8)      // radar sweep tail

// Currently selected aircraft (tap a row on the traffic page); falls back to nearest

// ---- Small helpers ----
String fitPx(const char* s, int maxW) {
  String t(s);
  if (tft.textWidth(t) <= maxW) return t;
  while (t.length() > 1 && tft.textWidth(t + ".") > maxW) t.remove(t.length() - 1);
  return t + ".";
}

String withCommas(int v) {
  char b[16]; snprintf(b, sizeof(b), "%d", abs(v));
  String s(b), out;
  int n = s.length();
  for (int i = 0; i < n; i++) { out += s[i]; if ((n - i - 1) % 3 == 0 && i < n - 1) out += ','; }
  return v < 0 ? "-" + out : out;
}

// "FL350" at and above 18,000 ft (US transition altitude), otherwise feet
String altText(int ft, bool withUnit) {
  char b[16];
  if (ft >= 18000) snprintf(b, sizeof(b), "FL%03d", (ft + 50) / 100);
  else return withCommas(ft) + (withUnit ? " ft" : "");
  return String(b);
}

void card(int x, int y, int w, int h) {
  tft.fillRoundRect(x, y, w, h, 6, C_PANEL);
  tft.drawRoundRect(x, y, w, h, 6, C_EDGE);
}

void label(const char* s, int x, int y, uint16_t bg = C_PANEL) {
  tft.setTextFont(1); tft.setTextDatum(TL_DATUM);
  tft.setTextColor(C_LABEL, bg);
  tft.drawString(s, x, y);
}

void chevron(int x, int cy, uint16_t c) {
  for (int k = 0; k < 2; k++) {
    tft.drawLine(x + k, cy - 5, x + 4 + k, cy, c);
    tft.drawLine(x + 4 + k, cy, x + k, cy + 5, c);
  }
}

// Degree sign for fonts without one (GFX / built-in fonts are ASCII only)
void degree(int x, int y, int r, uint16_t c) { tft.drawCircle(x, y, r, c); if (r > 2) tft.drawCircle(x, y, r - 1, c); }

void pill(int x, int y, int w, int h, const char* text, uint16_t bg, uint16_t fg) {
  tft.fillRoundRect(x, y, w, h, h / 2, bg);
  tft.setTextFont(h >= 18 ? 2 : 1); tft.setTextColor(fg, bg); tft.setTextDatum(MC_DATUM);
  tft.drawString(text, x + w / 2, y + h / 2 + (h >= 18 ? 1 : 0));
  tft.setTextDatum(TL_DATUM);
}

// Airplane silhouette on any canvas (screen or sprite); nose along angleDeg,
// 0 = pointing right, +90 = pointing down
void drawPlaneGlyph(TFT_eSPI& g, int cx, int cy, float sc, float angleDeg, uint16_t color) {
  static const float tri[][6] = {
    {-0.9f,-0.11f,  0.85f,-0.11f, 0.85f, 0.11f}, {-0.9f,-0.11f, 0.85f, 0.11f,-0.9f, 0.11f},  // fuselage
    { 0.85f,-0.11f, 1.1f, 0.0f,   0.85f, 0.11f},                                           // nose
    { 0.3f, 0.0f,  -0.25f,-0.95f,-0.3f, 0.0f}, { 0.3f, 0.0f, -0.25f, 0.95f,-0.3f, 0.0f},   // wings
    {-0.65f,0.0f,  -1.0f,-0.45f, -0.95f,0.0f}, {-0.65f,0.0f, -1.0f, 0.45f,-0.95f,0.0f},   // tail
  };
  float a = angleDeg * PI / 180.0f, ca = cosf(a), sa = sinf(a);
  for (auto& t : tri) {
    int p[6];
    for (int k = 0; k < 3; k++) {
      float x = t[2*k], y = t[2*k+1];
      p[2*k]   = cx + (int)lroundf(sc * (x * ca - y * sa));
      p[2*k+1] = cy + (int)lroundf(sc * (x * sa + y * ca));
    }
    g.fillTriangle(p[0], p[1], p[2], p[3], p[4], p[5], color);
  }
}
void drawPlaneGlyph(int cx, int cy, float sc, float angleDeg, uint16_t color) { drawPlaneGlyph(tft, cx, cy, sc, angleDeg, color); }

// Helicopter seen from above: body, tapered tail boom, tail rotor and a crossed main rotor.
// Same size and angle convention as drawPlaneGlyph.
void drawHeliGlyph(TFT_eSPI& g, int cx, int cy, float sc, float angleDeg, uint16_t color) {
  float a = angleDeg * PI / 180.0f, ca = cosf(a), sa = sinf(a);
  auto X = [&](float x, float y) { return cx + (int)lroundf(sc * (x * ca - y * sa)); };
  auto Y = [&](float x, float y) { return cy + (int)lroundf(sc * (x * sa + y * ca)); };
  // Tail boom (tapered) and tail rotor
  g.fillTriangle(X(0.05f, -0.12f), Y(0.05f, -0.12f), X(0.05f, 0.12f), Y(0.05f, 0.12f), X(-1.0f, 0), Y(-1.0f, 0), color);
  g.drawLine(X(-0.95f, -0.3f), Y(-0.95f, -0.3f), X(-0.95f, 0.3f), Y(-0.95f, 0.3f), color);
  // Body
  int r = (int)lroundf(sc * 0.34f); if (r < 2) r = 2;
  g.fillCircle(X(0.25f, 0), Y(0.25f, 0), r, color);
  // Main rotor: two blades crossing over the body
  for (float d : {0.785f, -0.785f}) {
    float bx = cosf(d) * 1.15f, by = sinf(d) * 1.15f;
    g.drawLine(X(0.25f + bx, by), Y(0.25f + bx, by), X(0.25f - bx, -by), Y(0.25f - bx, -by), color);
    if (sc >= 9) g.drawLine(X(0.25f + bx, by) + 1, Y(0.25f + bx, by), X(0.25f - bx, -by) + 1, Y(0.25f - bx, -by), color);
  }
}

// Aircraft symbol: a helicopter for rotorcraft (ADS-B category A7), otherwise an airplane
void drawCraftGlyph(TFT_eSPI& g, bool heli, int cx, int cy, float sc, float angleDeg, uint16_t color) {
  if (heli) drawHeliGlyph(g, cx, cy, sc, angleDeg, color); else drawPlaneGlyph(g, cx, cy, sc, angleDeg, color);
}

void arrowUpDown(int cx, int cy, int s, bool up, uint16_t c) {
  if (up) { tft.fillTriangle(cx - s, cy + s / 2, cx + s, cy + s / 2, cx, cy - s, c); tft.fillRect(cx - s / 3, cy + s / 2, 2 * s / 3 + 1, s, c); }
  else    { tft.fillTriangle(cx - s, cy - s / 2, cx + s, cy - s / 2, cx, cy + s, c); tft.fillRect(cx - s / 3, cy - 3 * s / 2, 2 * s / 3 + 1, s, c); }
}

// ---- Icons (drawn from primitives, ~20 px) ----
// WiFi: base point (cx, by); lit = number of lit arcs 0..3
void iconWifi(int cx, int by, int lit, uint16_t on, uint16_t off, uint16_t bg) {
  tft.fillCircle(cx, by, 2, lit > 0 ? on : off);
  const int r[3] = {6, 10, 14};
  for (int i = 0; i < 3; i++)
    tft.drawArc(cx, by, r[i], r[i] - 2, 135, 225, i < lit ? on : off, bg, true);
}

void iconPin(int cx, int cy, uint16_t c, uint16_t bg) {
  tft.fillCircle(cx, cy - 4, 7, c);
  tft.fillTriangle(cx - 6, cy - 1, cx + 6, cy - 1, cx, cy + 10, c);
  tft.fillCircle(cx, cy - 4, 3, bg);
}

void iconCrosshair(int cx, int cy, uint16_t c) {
  tft.drawCircle(cx, cy, 8, c); tft.drawCircle(cx, cy, 7, c);
  tft.drawFastHLine(cx - 12, cy, 7, c); tft.drawFastHLine(cx + 6, cy, 7, c);
  tft.drawFastVLine(cx, cy - 12, 7, c); tft.drawFastVLine(cx, cy + 6, 7, c);
  tft.fillCircle(cx, cy, 2, c);
}

void iconSun(int cx, int cy, int r, uint16_t c) {
  tft.fillCircle(cx, cy, r, c);
  for (int i = 0; i < 8; i++) {
    float a = i * PI / 4;
    int x0 = cx + lroundf(cosf(a) * (r + 3)), y0 = cy + lroundf(sinf(a) * (r + 3));
    int x1 = cx + lroundf(cosf(a) * (r + 6)), y1 = cy + lroundf(sinf(a) * (r + 6));
    tft.drawLine(x0, y0, x1, y1, c); tft.drawLine(x0 + 1, y0, x1 + 1, y1, c);
  }
}

void iconBell(int cx, int cy, uint16_t c) {
  tft.fillCircle(cx, cy - 3, 7, c);
  tft.fillRect(cx - 7, cy - 3, 15, 7, c);
  tft.fillRoundRect(cx - 10, cy + 3, 21, 4, 2, c);
  tft.fillCircle(cx, cy + 9, 2, c);
  tft.fillRect(cx - 1, cy - 13, 3, 3, c);
}

void iconGear(int cx, int cy, uint16_t c, uint16_t bg) {
  for (int i = 0; i < 8; i++) {
    float a = i * PI / 4;
    tft.fillCircle(cx + (int)lroundf(cosf(a) * 8), cy + (int)lroundf(sinf(a) * 8), 2, c);
  }
  tft.fillCircle(cx, cy, 7, c);
  tft.fillCircle(cx, cy, 3, bg);
}

void iconClock(int cx, int cy, uint16_t c) {
  tft.drawCircle(cx, cy, 9, c); tft.drawCircle(cx, cy, 8, c);
  tft.drawLine(cx, cy, cx, cy - 5, c); tft.drawLine(cx, cy, cx + 4, cy + 2, c);
}

void iconChip(int cx, int cy, uint16_t c) {
  tft.drawRect(cx - 6, cy - 6, 13, 13, c); tft.drawRect(cx - 5, cy - 5, 11, 11, c);
  tft.fillRect(cx - 2, cy - 2, 5, 5, c);
  for (int k = -4; k <= 4; k += 4) {
    tft.drawFastHLine(cx - 9, cy + k, 3, c); tft.drawFastHLine(cx + 7, cy + k, 3, c);
    tft.drawFastVLine(cx + k, cy - 9, 3, c); tft.drawFastVLine(cx + k, cy + 7, 3, c);
  }
}

void iconCloud(int cx, int cy, uint16_t c) {
  tft.fillCircle(cx - 6, cy + 2, 5, c);
  tft.fillCircle(cx + 1, cy - 2, 7, c);
  tft.fillCircle(cx + 8, cy + 3, 4, c);
  tft.fillRect(cx - 6, cy + 2, 14, 6, c);
}

void iconRefresh(int cx, int cy, uint16_t c, uint16_t bg) {
  tft.drawArc(cx, cy, 9, 7, 60, 330, c, bg, true);
  tft.fillTriangle(cx - 9, cy - 9, cx - 2, cy - 9, cx - 6, cy - 3, c);
}

void iconCompass(int cx, int cy, float trackDeg, uint16_t c) {
  tft.drawCircle(cx, cy, 12, c); tft.drawCircle(cx, cy, 11, c);
  float a = (trackDeg - 90) * PI / 180.0f;
  int tx = cx + lroundf(cosf(a) * 8), ty = cy + lroundf(sinf(a) * 8);
  int lx = cx + lroundf(cosf(a + 2.5f) * 6), ly = cy + lroundf(sinf(a + 2.5f) * 6);
  int rx = cx + lroundf(cosf(a - 2.5f) * 6), ry = cy + lroundf(sinf(a - 2.5f) * 6);
  tft.fillTriangle(tx, ty, lx, ly, cx, cy, c);
  tft.fillTriangle(tx, ty, rx, ry, cx, cy, c);
}

// WMO weather code -> short description and icon
const char* wmoText(int code) {
  if (code == 0) return "Clear";
  if (code == 1) return "Mostly Clear";
  if (code == 2) return "Partly Cloudy";
  if (code == 3) return "Overcast";
  if (code == 45 || code == 48) return "Fog";
  if (code >= 51 && code <= 57) return "Drizzle";
  if ((code >= 61 && code <= 67) || (code >= 80 && code <= 82)) return "Rain";
  if ((code >= 71 && code <= 77) || code == 85 || code == 86) return "Snow";
  if (code >= 95) return "Thunderstorm";
  return "";
}

void weatherIcon(int cx, int cy, int code) {
  bool sun = code <= 2, cloud = code >= 1;
  if (sun) iconSun(cloud ? cx - 7 : cx, cloud ? cy - 6 : cy, cloud ? 6 : 9, C_SUN);
  if (cloud) {
    uint16_t cc = code >= 45 ? 0xBDF7 : C_TEXT;
    tft.fillCircle(cx - 7, cy + 5, 7, cc); tft.fillCircle(cx + 3, cy, 9, cc); tft.fillCircle(cx + 12, cy + 6, 6, cc);
    tft.fillRect(cx - 7, cy + 5, 19, 8, cc);
    if ((code >= 51 && code <= 67) || (code >= 80 && code <= 82))
      for (int k = -6; k <= 10; k += 8) tft.drawLine(cx + k, cy + 16, cx + k - 3, cy + 22, C_CYAN);
    if ((code >= 71 && code <= 77) || code == 85 || code == 86)
      for (int k = -6; k <= 10; k += 8) tft.fillCircle(cx + k, cy + 19, 2, C_TEXT);
    if (code >= 95) tft.fillTriangle(cx + 2, cy + 13, cx - 3, cy + 22, cx + 5, cy + 19, C_SUN);
  }
}

// ---- Header: title, clock, WiFi strength (tap the top-right corner for settings) ----
int wifiBars() {
  if (WiFi.status() != WL_CONNECTED) return 0;
  int r = WiFi.RSSI();
  return r > -60 ? 3 : r > -70 ? 2 : 1;
}

void drawHeader(const char* title, bool newPage) {
  static char lastClock[8] = "";
  static int lastBars = -1;
  if (newPage) {
    tft.fillRect(0, 0, 320, 26, C_BG);
    tft.setFreeFont(&FreeSansBold9pt7b);
    tft.setTextDatum(TL_DATUM); tft.setTextColor(C_CYAN, C_BG);
    tft.drawString(title, 8, 4);
    tft.drawFastHLine(0, 25, 320, C_EDGE);
    lastClock[0] = '\0'; lastBars = -1;
  }
  char t[8]; fmtClock(t, sizeof(t), false);
  if (strcmp(t, lastClock) != 0) {
    strcpy(lastClock, t);
    tft.setTextFont(2); tft.setTextDatum(TR_DATUM); tft.setTextColor(C_TEXT, C_BG);
    tft.setTextPadding(44);
    tft.drawString(t, 282, 5);
    tft.setTextPadding(0); tft.setTextDatum(TL_DATUM);
  }
  int bars = wifiBars();
  if (bars != lastBars) {
    lastBars = bars;
    tft.fillRect(287, 2, 32, 22, C_BG);
    iconWifi(303, 19, bars, screen == SCREEN_SETTINGS ? C_WARN : C_CYAN, C_TRACK, C_BG);
  }
}

// ===========================================================================
// 1. NEAREST AIRCRAFT
// ===========================================================================
// Selected aircraft if one was tapped recently and is still in the top 5, else nearest
const Aircraft* targetAircraft(bool* isSelected = nullptr) {
  if (isSelected) *isSelected = false;
  if (selectedHex[0] && stillBefore(selectedUntil)) {
    for (int i = 0; i < top5Count; i++)
      if (strcmp(top5[i].hex, selectedHex) == 0) {
        if (isSelected) *isSelected = i != 0;
        return i == 0 ? &nearest : &top5[i];
      }
    if (selAc.valid && strcmp(selAc.hex, selectedHex) == 0) {   // tapped on the radar, beyond the top 5
      if (isSelected) *isSelected = true;
      return &selAc;
    }
  }
  return nearest.valid ? &nearest : nullptr;
}

void screenTargetIntel(bool newPage) {
  bool sel = false;
  const Aircraft* ap = targetAircraft(&sel);
  static bool lastSel = false;
  if (sel != lastSel) { lastSel = sel; newPage = true; tft.fillScreen(C_BG); }
  drawHeader(sel ? (heliOnly ? "SELECTED HELICOPTER" : "SELECTED AIRCRAFT") : (heliOnly ? "NEAREST HELICOPTER" : "NEAREST AIRCRAFT"), newPage);

  // Redraw only when the data or the selection changes (no flicker)
  static uint32_t drawnData = 0;
  static const Aircraft* drawnAp = nullptr;
  if (!newPage && drawnData == lastDataMs && drawnAp == ap) return;
  drawnData = lastDataMs; drawnAp = ap;

  tft.setTextPadding(0); tft.setTextDatum(TL_DATUM);
  tft.fillRect(0, 26, 320, 214, C_BG);

  if (!ap) {
    card(20, 80, 280, 80);
    drawPlaneGlyph(160, 108, 14, -90, C_EDGE);
    tft.setFreeFont(&FreeSans9pt7b); tft.setTextColor(C_SOFT, C_PANEL); tft.setTextDatum(TC_DATUM);
    tft.drawString(heliOnly ? "No helicopters in range" : "No aircraft in range", 160, 128);
    tft.setTextDatum(TL_DATUM); tft.setTextFont(2);
    return;
  }
  const Aircraft& a = *ap;
  char buf[48];

  // --- Identity card ---
  card(4, 29, 312, 52);
  tft.setFreeFont(&FreeSansBold18pt7b); tft.setTextColor(C_TEXT, C_PANEL);
  tft.drawString(fitPx(a.callsign, 196), 12, 31);
  tft.setFreeFont(&FreeSans9pt7b); tft.setTextColor(C_SOFT, C_PANEL);
  tft.drawString(fitPx(a.airline[0] ? a.airline : (a.model[0] ? a.model : "Unknown operator"), 200), 12, 62);
  bool mil = a.dbFlags & 1, heli = strcmp(a.category, "A7") == 0;
  drawCraftGlyph(tft, heli, 240, 51, 15, -90, mil ? C_WARN : C_CYAN);
  tft.setTextFont(2); tft.setTextColor(C_SOFT, C_PANEL); tft.setTextDatum(TR_DATUM);
  tft.drawString(mil ? "MIL" : heli ? "HELI" : (a.type[0] ? a.type : ""), 306, 58);
  tft.setTextDatum(TL_DATUM);

  // --- Route card: the line doubles as a progress bar, the plane sits where it is ---
  card(4, 85, 312, 44);
  if (a.hasRoute) {
    tft.setFreeFont(&FreeSansBold12pt7b); tft.setTextColor(C_TEXT, C_PANEL);
    tft.drawString(a.dep, 12, 88);
    int wDep = tft.textWidth(a.dep);
    tft.setTextDatum(TR_DATUM); tft.drawString(a.arr, 308, 88);
    int wArr = tft.textWidth(a.arr);
    tft.setTextFont(2); tft.setTextColor(C_SOFT, C_PANEL);
    tft.drawString(fitPx(a.arrCity, 130), 308, 108);
    tft.setTextDatum(TL_DATUM);
    tft.drawString(fitPx(a.depCity, 130), 12, 108);

    int x0 = 12 + wDep + 10, x1 = 308 - wArr - 10, ly = 98;
    float frac = 0.5f;
    if (a.hasCoords) {
      double flown = haversineKm(a.depLat, a.depLon, a.lat, a.lon);
      double left  = haversineKm(a.lat, a.lon, a.arrLat, a.arrLon);
      if (flown + left > 1) frac = (float)(flown / (flown + left));
    }
    int px = x0 + (int)((x1 - x0) * frac);
    tft.drawFastHLine(x0, ly, x1 - x0, C_EDGE);
    tft.drawFastHLine(x0, ly, px - x0, C_CYAN);
    tft.fillCircle(x0, ly, 2, C_CYAN); tft.fillCircle(x1, ly, 2, C_EDGE);
    tft.fillRect(px - 9, ly - 8, 19, 17, C_PANEL);
    drawCraftGlyph(tft, strcmp(a.category, "A7") == 0, px, ly, 8, 0, C_CYAN);
  } else {
    tft.setFreeFont(&FreeSans9pt7b); tft.setTextColor(C_SOFT, C_PANEL); tft.setTextDatum(MC_DATUM);
    tft.drawString("Route unknown", 160, 105);
    tft.setTextDatum(TL_DATUM);
  }

  // --- Altitude / speed / heading ---
  const int R1 = 133, H1 = 44;
  card(4, R1, 100, H1); card(108, R1, 100, H1); card(212, R1, 104, H1);
  label(a.altFt >= 18000 ? "ALTITUDE" : "ALTITUDE FT", 12, R1 + 6);
  label("SPEED", 116, R1 + 6);
  label("HEADING", 220, R1 + 6);
  tft.setFreeFont(&FreeSansBold12pt7b); tft.setTextColor(C_TEXT, C_PANEL);
  tft.drawString(altText(a.altFt, false), 12, R1 + 18);
  snprintf(buf, sizeof(buf), "%d KT", (int)lround(a.gsKt));
  tft.drawString(buf, 116, R1 + 18);
  snprintf(buf, sizeof(buf), "%03d", ((int)lround(a.trackDeg)) % 360);
  tft.drawString(buf, 220, R1 + 18);
  degree(220 + tft.textWidth(buf) + 4, R1 + 21, 3, C_TEXT);
  iconCompass(296, R1 + 26, a.trackDeg, C_CYAN);

  // --- Distance / position / climb ---
  const int R2 = 181, H2 = 55;
  card(4, R2, 100, H2); card(108, R2, 134, H2); card(246, R2, 70, H2);
  label("DISTANCE", 12, R2 + 6);
  label("POSITION", 116, R2 + 6);
  label(a.vrateFpm >= 300 ? "CLIMB" : a.vrateFpm <= -300 ? "DESCENT" : "LEVEL", 254, R2 + 6);
  tft.setFreeFont(&FreeSansBold12pt7b); tft.setTextColor(C_TEXT, C_PANEL);
  snprintf(buf, sizeof(buf), "%.1f %s", toDisplayDist(a.distanceKm), DIST_UNIT);
  tft.drawString(fitPx(buf, 86), 12, R2 + 22);
  tft.setTextFont(2); tft.setTextColor(C_TEXT, C_PANEL);
  for (int k = 0; k < 2; k++) {
    double v = k == 0 ? a.lat : a.lon;
    snprintf(buf, sizeof(buf), "%.2f", fabs(v));
    int y = R2 + 18 + k * 18;
    tft.drawString(buf, 116, y);
    int w = tft.textWidth(buf);
    degree(116 + w + 3, y + 3, 2, C_TEXT);
    tft.drawString(k == 0 ? (v >= 0 ? "N" : "S") : (v >= 0 ? "E" : "W"), 116 + w + 8, y);
  }
  bool up = a.vrateFpm >= 300, down = a.vrateFpm <= -300;
  drawCraftGlyph(tft, strcmp(a.category, "A7") == 0, 274, R2 + 32, 11, up ? -35 : down ? 35 : 0, up ? C_GREEN : down ? C_WARN : C_SOFT);
  if (up || down) {
    arrowUpDown(304, R2 + 26, 4, up, up ? C_GREEN : C_WARN);
    tft.setTextFont(1); tft.setTextColor(C_SOFT, C_PANEL);
    snprintf(buf, sizeof(buf), "%s fpm", withCommas(abs(a.vrateFpm)).c_str());
    tft.drawString(buf, 254, R2 + 45);
  }
  tft.setTextFont(2);
}

// ===========================================================================
// 2. TRAFFIC NEAR YOU — tap a row to show that aircraft on page 1
// ===========================================================================
const int TR_Y = 29, TR_H = 38, TR_STEP = 42;

void screenTop5(bool newPage) {
  drawHeader(heliOnly ? "HELICOPTERS NEAR YOU" : "TRAFFIC NEAR YOU", newPage);

  const Aircraft* ap = targetAircraft();
  static uint32_t drawnData = 0;
  static const Aircraft* drawnAp = nullptr;
  if (!newPage && drawnData == lastDataMs && drawnAp == ap) return;
  drawnData = lastDataMs; drawnAp = ap;

  tft.setTextPadding(0); tft.setTextDatum(TL_DATUM);
  tft.fillRect(0, 26, 320, 214, C_BG);

  if (top5Count == 0) {
    card(10, 82, 300, 80);
    tft.setFreeFont(&FreeSans9pt7b); tft.setTextColor(C_TEXT, C_PANEL); tft.setTextDatum(MC_DATUM);
    tft.drawString(heliOnly ? "No helicopters in range" : "No aircraft in range", 160, 112);
    tft.setTextFont(1); tft.setTextColor(C_SOFT, C_PANEL);
    tft.drawString("Traffic will appear here when detected", 160, 138);
    tft.setTextDatum(TL_DATUM); tft.setTextFont(2);
    return;
  }

  char buf[24];
  for (int i = 0; i < top5Count && i < MAX_TOP5; i++) {
    const Aircraft& a = top5[i];
    int y = TR_Y + i * TR_STEP;
    bool isTarget = ap && strcmp(ap->hex, a.hex) == 0;
    card(4, y, 312, TR_H);
    if (isTarget) { tft.drawRoundRect(4, y, 312, TR_H, 6, C_CYAN); tft.drawRoundRect(5, y + 1, 310, TR_H - 2, 5, C_CYAN); }
    bool mil = a.dbFlags & 1;
    drawCraftGlyph(tft, strcmp(a.category, "A7") == 0, 22, y + 19, 9, -90, mil ? C_WARN : isTarget ? C_CYAN : C_GREEN);

    tft.setFreeFont(&FreeSansBold9pt7b); tft.setTextColor(C_TEXT, C_PANEL);
    tft.drawString(fitPx(a.callsign, 130), 40, y + 3);
    tft.drawString(altText(a.altFt, true), 184, y + 3);

    tft.setTextFont(2); tft.setTextColor(C_SOFT, C_PANEL);
    if (a.hasRoute) snprintf(buf, sizeof(buf), "%.4s > %.4s", a.dep, a.arr);
    else snprintf(buf, sizeof(buf), "%s", a.type[0] ? a.type : "-");
    tft.drawString(buf, 40, y + 20);
    snprintf(buf, sizeof(buf), "%.1f %s", toDisplayDist(a.distanceKm), DIST_UNIT);
    tft.drawString(buf, 184, y + 20);
    chevron(298, y + TR_H / 2, C_SOFT);
  }
  tft.setTextFont(2);
}

// ===========================================================================
// 3. RADAR PPI — range presets on the left, legend on the right
// ===========================================================================
#if UNITS_IMPERIAL
const int RANGE_PRESETS[4] = {100, 50, 25, 10};
#else
const int RANGE_PRESETS[4] = {200, 100, 50, 25};
#endif
const int RB_X = 4, RB_W = 38, RB_Y = 30, RB_H = 46, RB_STEP = 52;
const int SCOPE_X = 46, SCOPE_Y = 30, SCOPE = 186;   // sprite size (8-bit, ~34 KB)
int16_t blipSX[MAX_BLIPS], blipSY[MAX_BLIPS]; bool blipVis[MAX_BLIPS];   // last drawn, for taps

void drawRangeButtons(float current) {
  for (int k = 0; k < 4; k++) {
    int y = RB_Y + k * RB_STEP;
    bool on = lroundf(current) == RANGE_PRESETS[k];
    tft.fillRoundRect(RB_X, y, RB_W, RB_H, 6, on ? C_BADGE : C_PANEL);
    tft.drawRoundRect(RB_X, y, RB_W, RB_H, 6, on ? C_CYAN : C_EDGE);
    if (on) tft.drawRoundRect(RB_X + 1, y + 1, RB_W - 2, RB_H - 2, 5, C_CYAN);
    tft.setTextDatum(MC_DATUM);
    tft.setTextFont(2); tft.setTextColor(C_TEXT, on ? C_BADGE : C_PANEL);
    tft.drawString(String(RANGE_PRESETS[k]), RB_X + RB_W / 2, y + 17);
    tft.setTextFont(1); tft.setTextColor(C_SOFT, on ? C_BADGE : C_PANEL);
    tft.drawString(DIST_UNIT, RB_X + RB_W / 2, y + 33);
    tft.setTextDatum(TL_DATUM);
  }
}

void screenRadar(bool newPage) {
  drawHeader(heliOnly ? "RADAR - HELICOPTERS" : "RADAR PPI", newPage);

  double hLat, hLon; float rRange;
  if (configMutex) xSemaphoreTake(configMutex, portMAX_DELAY);
  hLat = homeLat; hLon = homeLon; rRange = radarRange;
  if (configMutex) xSemaphoreGive(configMutex);
  double rMaxKm = rRange * KM_PER_UNIT;
  const Aircraft* ap = targetAircraft();

  // Static parts and slowly changing values
  static float drawnRange = -1; static int drawnTracks = -1;
  if (newPage) {
    tft.fillRect(0, 26, 320, 214, C_BG);
    tft.fillRoundRect(SCOPE_X - 2, SCOPE_Y - 2, SCOPE + 4, SCOPE + 4, 8, C_SCOPE);   // scope bezel
    tft.drawRoundRect(SCOPE_X - 2, SCOPE_Y - 2, SCOPE + 4, SCOPE + 4, 8, C_EDGE);
    card(236, 30, 80, 156);
    label("RANGE", 243, 37); label("TRACKS", 243, 73);
    tft.drawFastHLine(242, 105, 68, C_EDGE);
    drawPlaneGlyph(250, 120, 6, -90, C_GREEN);
    tft.drawCircle(250, 140, 5, C_CYAN); tft.drawCircle(250, 140, 4, C_CYAN);
    for (int k = 0; k < 3; k++) tft.drawFastHLine(244 + k * 5, 160, 3, C_SOFT);
    tft.setTextFont(1); tft.setTextColor(C_SOFT, C_PANEL);
    tft.drawString("Aircraft", 262, 117); tft.drawString("Selected", 262, 137); tft.drawString("Range", 262, 157);
    card(236, 192, 80, 40);
    tft.setFreeFont(&FreeSansBold9pt7b); tft.setTextColor(C_SOFT, C_PANEL);
    tft.drawString("N", 262, 205);
    tft.fillTriangle(284, 206, 280, 213, 288, 213, C_SOFT); tft.fillRect(283, 213, 3, 9, C_SOFT);
    drawnRange = -1; drawnTracks = -1;
  }
  if (rRange != drawnRange) {
    drawnRange = rRange;
    drawRangeButtons(rRange);
    tft.fillRect(242, 47, 70, 18, C_PANEL);
    tft.setFreeFont(&FreeSansBold9pt7b); tft.setTextColor(C_TEXT, C_PANEL);
    tft.drawString(String((int)lroundf(rRange)) + " " + DIST_UNIT, 243, 47);
  }
  if (stats.inView != drawnTracks) {
    drawnTracks = stats.inView;
    tft.fillRect(242, 83, 70, 18, C_PANEL);
    tft.setFreeFont(&FreeSansBold9pt7b); tft.setTextColor(C_TEXT, C_PANEL);
    tft.drawString(String(stats.inView), 243, 83);
  }

  // Scope (sprite, every frame)
  const int cx = SCOPE / 2, cy = SCOPE / 2, R = 80;
  radarSpr.fillSprite(C_SCOPE);
  for (int a = 0; a < 360; a += 6) {                       // dashed inner rings
    float t = a * PI / 180.0f;
    for (int ring = 1; ring <= 2; ring++) {
      int rr = R * ring / 3;
      radarSpr.drawPixel(cx + lroundf(cosf(t) * rr), cy + lroundf(sinf(t) * rr), C_GRID8);
      radarSpr.drawPixel(cx + lroundf(cosf(t + 0.03f) * rr), cy + lroundf(sinf(t + 0.03f) * rr), C_GRID8);
    }
  }
  radarSpr.drawFastHLine(cx - R, cy, 2 * R, C_GRID8);
  radarSpr.drawFastVLine(cx, cy - R, 2 * R, C_GRID8);

  float sweepDeg = fmodf(millis() / 15.0f, 360.0f);
  for (int k = 30; k >= 0; k -= 2) {                       // sweep with a fading tail
    double s = deg2rad(sweepDeg - k);
    uint16_t c = k > 16 ? C_DIM8 : k > 4 ? C_RING8 : C_CYAN;
    radarSpr.drawLine(cx, cy, cx + (int)(sin(s) * R), cy - (int)(cos(s) * R), c);
  }
  radarSpr.drawCircle(cx, cy, R, C_RING8); radarSpr.drawCircle(cx, cy, R + 1, C_RING8);
  for (int a = 0; a < 360; a += 30) {                      // ticks
    float t = a * PI / 180.0f;
    radarSpr.drawLine(cx + lroundf(cosf(t) * (R - 4)), cy + lroundf(sinf(t) * (R - 4)),
                      cx + lroundf(cosf(t) * R), cy + lroundf(sinf(t) * R), C_RING8);
  }
  radarSpr.setTextFont(1); radarSpr.setTextColor(C_TEXT, C_SCOPE); radarSpr.setTextDatum(MC_DATUM);
  radarSpr.drawString("N", cx, cy - R - 6); radarSpr.drawString("S", cx, cy + R + 6);
  radarSpr.drawString("W", cx - R - 6, cy); radarSpr.drawString("E", cx + R + 6, cy);
  radarSpr.setTextDatum(TL_DATUM);
  radarSpr.fillCircle(cx, cy, 2, C_CYAN);

  float elapsed = (millis() - lastDataMs) / 1000.0f;
  auto toScope = [&](double la, double lo, int& x, int& y) {
    float fr = (float)(haversineKm(hLat, hLon, la, lo) / rMaxKm);
    if (fr > 1) return false;
    double b = deg2rad(bearingDeg(hLat, hLon, la, lo));
    x = cx + (int)(sin(b) * fr * R); y = cy - (int)(cos(b) * fr * R);
    return true;
  };
  // Trails: earlier positions, older ones dimmer
  for (uint8_t i = 0; i < blipCount; i++) {
    for (const Trail& tr : trails) {
      if (!tr.hex[0] || strcmp(tr.hex, blips[i].hex) != 0) continue;
      for (int k = 0; k + 1 < tr.n; k++) {
        int x, y;
        if (toScope(tr.lat[k], tr.lon[k], x, y)) radarSpr.fillRect(x, y, 2, 2, k < tr.n / 2 ? C_DIM8 : C_RING8);
      }
      break;
    }
  }
  for (uint8_t i = 0; i < blipCount; i++) blipVis[i] = false;
  for (uint8_t i = 0; i < blipCount; i++) {
    double la = blips[i].lat, lo = blips[i].lon;
    if (blips[i].speedKt > 0 && elapsed > 0)
      projectLatLon(blips[i].lat, blips[i].lon, blips[i].track, blips[i].speedKt * 0.514444f * elapsed, la, lo);
    double dist = haversineKm(hLat, hLon, la, lo);
    float fr = (float)(dist / rMaxKm);
    if (fr > 1) continue;
    int rr = (int)(fr * R);
    double brg = bearingDeg(hLat, hLon, la, lo);
    int bx = cx + (int)(sin(deg2rad(brg)) * rr);
    int by = cy - (int)(cos(deg2rad(brg)) * rr);
    blipVis[i] = true; blipSX[i] = SCOPE_X + bx; blipSY[i] = SCOPE_Y + by;
    bool isTarget = ap && strcmp(ap->hex, blips[i].hex) == 0;
    if (isTarget) {
      radarSpr.drawCircle(bx, by, 11, C_CYAN);
      drawCraftGlyph(radarSpr, blips[i].heli, bx, by, 8, blips[i].track - 90, C_CYAN);
      radarSpr.setTextColor(C_CYAN, C_SCOPE);
      int lx = bx + 13; if (lx + 50 > SCOPE) lx = bx - 63;
      radarSpr.drawString(blips[i].callsign, lx, by - 4);
    } else {
      drawCraftGlyph(radarSpr, blips[i].heli, bx, by, 6, blips[i].track - 90, C_GREEN);
    }
  }
  radarSpr.pushSprite(SCOPE_X, SCOPE_Y);
}

// ===========================================================================
// 4. WEATHER & STATUS
// ===========================================================================
void screenWeatherSystem(bool newPage) {
  drawHeader("WEATHER & STATUS", newPage);

  static uint32_t lastValues = 0;
  static uint32_t drawnWxKey = 0;
  if (newPage) {
    tft.setTextPadding(0); tft.setTextDatum(TL_DATUM);
    tft.fillRect(0, 26, 320, 214, C_BG);
    card(4, 29, 312, 90);
    label("LOCAL WEATHER", 12, 35);
    tft.drawFastVLine(178, 38, 74, C_EDGE);
    tft.setTextFont(2); tft.setTextColor(C_SOFT, C_PANEL);
    tft.drawString("Wind", 186, 40); tft.drawString("Humidity", 186, 64); tft.drawString("Baro", 186, 88);
    tft.drawFastHLine(186, 59, 122, C_EDGE); tft.drawFastHLine(186, 83, 122, C_EDGE);

    card(4, 123, 154, 48); card(162, 123, 154, 48);
    label("DEVICE UPTIME", 12, 129); label("WIFI", 170, 129);
    iconClock(24, 154, C_SOFT);
    card(4, 175, 104, 61); card(112, 175, 100, 61); card(216, 175, 100, 61);
    label("MEMORY (HEAP)", 12, 181); label("API STATUS", 120, 181); label("LAST UPDATE", 224, 181);
    iconChip(22, 204, C_SOFT);
    iconCloud(131, 211, C_SOFT);
    iconRefresh(235, 211, C_SOFT, C_PANEL);
    drawnWxKey = 0xFFFFFFFF; lastValues = 0;
  }

  // Weather block: redrawn when the values change
  uint32_t wxKey = weather.valid ? ((uint32_t)lroundf(weather.temp * 10) * 31u + weather.humidity * 7u +
                                    (uint32_t)lroundf(weather.wind) * 3u + weather.code + (uint32_t)weather.pressure) : 1;
  if (wxKey != drawnWxKey) {
    drawnWxKey = wxKey;
    tft.fillRect(10, 45, 166, 70, C_PANEL);
    tft.fillRect(240, 40, 72, 16, C_PANEL); tft.fillRect(240, 64, 72, 16, C_PANEL); tft.fillRect(240, 88, 72, 16, C_PANEL);
    char buf[32];
    if (!weather.valid) {
      tft.setTextFont(2); tft.setTextColor(C_SOFT, C_PANEL);
      tft.drawString("Weather unavailable", 14, 72);
    } else {
      weatherIcon(34, 76, weather.code);
      tft.setFreeFont(&FreeSansBold18pt7b); tft.setTextColor(C_TEXT, C_PANEL);
      snprintf(buf, sizeof(buf), "%d", (int)lroundf(weather.temp));
      tft.drawString(buf, 66, 50);
      int w = tft.textWidth(buf);
      degree(66 + w + 5, 55, 4, C_TEXT);
      tft.drawString(TEMP_UNIT, 66 + w + 12, 50);
      tft.setTextFont(2); tft.setTextColor(C_SOFT, C_PANEL);
      tft.drawString(fitPx(wmoText(weather.code), 108), 66, 92);

      tft.setTextColor(C_TEXT, C_PANEL); tft.setTextDatum(TR_DATUM);
      snprintf(buf, sizeof(buf), "%s %d %s", compass(weather.windDir), (int)lroundf(weather.wind), WIND_UNIT);
      tft.drawString(buf, 308, 40);
      snprintf(buf, sizeof(buf), "%d%%", weather.humidity);
      tft.drawString(buf, 308, 64);
#if UNITS_IMPERIAL
      snprintf(buf, sizeof(buf), "%.2f inHg", weather.pressure * 0.02953f);
#else
      snprintf(buf, sizeof(buf), "%d hPa", (int)lroundf(weather.pressure));
#endif
      tft.drawString(buf, 308, 88);
      tft.setTextDatum(TL_DATUM);
    }
  }

  // Status values: once per second
  if (!newPage && millis() - lastValues < 1000) return;
  lastValues = millis();
  char buf[40];

  uint32_t up = millis() / 1000;
  if (up >= 86400) snprintf(buf, sizeof(buf), "%lud %luh", (unsigned long)(up / 86400), (unsigned long)((up % 86400) / 3600));
  else if (up >= 3600) snprintf(buf, sizeof(buf), "%luh %02lum", (unsigned long)(up / 3600), (unsigned long)((up % 3600) / 60));
  else snprintf(buf, sizeof(buf), "%lum %02lus", (unsigned long)(up / 60), (unsigned long)(up % 60));
  tft.fillRect(40, 140, 112, 26, C_PANEL);
  tft.setFreeFont(&FreeSansBold12pt7b); tft.setTextColor(C_TEXT, C_PANEL);
  tft.drawString(buf, 42, 141);

  bool online = WiFi.status() == WL_CONNECTED;
  tft.fillRect(168, 140, 144, 28, C_PANEL);
  tft.fillRect(240, 126, 72, 14, C_PANEL);
  iconWifi(180, 163, wifiBars(), online ? C_CYAN : C_WARN, C_TRACK, C_PANEL);
  if (online) { snprintf(buf, sizeof(buf), "%d dBm", WiFi.RSSI()); }
  else snprintf(buf, sizeof(buf), "no link");
  tft.setFreeFont(&FreeSansBold9pt7b); tft.setTextColor(C_TEXT, C_PANEL);
  tft.drawString(buf, 200, 146);
  pill(244, 127, 66, 12, online ? "CONNECTED" : "OFFLINE", online ? C_PILLOK : TFT_RED, online ? TFT_BLACK : TFT_WHITE);

  // Memory: free / total heap, bar = used share; largest free block (TLS needs ~45 KB)
  uint32_t freeH = ESP.getFreeHeap(), totalH = ESP.getHeapSize(), blk = ESP.getMaxAllocHeap();
  tft.fillRect(34, 192, 72, 40, C_PANEL);
  tft.setTextFont(2); tft.setTextColor(C_TEXT, C_PANEL);
  snprintf(buf, sizeof(buf), "%u/%uK", (unsigned)(freeH / 1024), (unsigned)(totalH / 1024));
  tft.drawString(buf, 36, 194);
  tft.setTextFont(1); tft.setTextColor(blk < 45000 ? C_WARN : C_SOFT, C_PANEL);
  snprintf(buf, sizeof(buf), "blk %uK", (unsigned)(blk / 1024));
  tft.drawString(buf, 36, 212);
  int used = totalH ? (int)(88L * (totalH - freeH) / totalH) : 0;
  tft.fillRoundRect(12, 224, 88, 5, 2, C_TRACK);
  tft.fillRoundRect(12, 224, max(4, used), 5, 2, blk < 45000 ? C_WARN : C_GREEN);

  // API status: online if data arrived within a minute
  uint32_t ago = lastOkMs ? (millis() - lastOkMs) / 1000 : 0;
  const char* st; uint16_t sc, sf = TFT_BLACK;
  if (!online) { st = "Offline"; sc = TFT_RED; sf = TFT_WHITE; }
  else if (lastOkMs && ago < 60) { st = "Online"; sc = C_PILLOK; }
  else if (lastOkMs) { st = "Stale"; sc = C_WARN; }
  else { st = "Waiting"; sc = C_TRACK; sf = C_TEXT; }
  tft.fillRect(146, 198, 64, 36, C_PANEL);
  pill(148, 202, 58, 18, st, sc, sf);
  if (lastFetchErr[0] && !(lastOkMs && ago < 60)) {
    tft.setTextFont(1); tft.setTextColor(C_WARN, C_PANEL);
    tft.drawString(fitPx(lastFetchErr, 90), 118, 225);
  }

  tft.fillRect(248, 198, 64, 30, C_PANEL);
  tft.setFreeFont(&FreeSansBold9pt7b); tft.setTextColor(C_TEXT, C_PANEL);
  if (!lastOkMs) snprintf(buf, sizeof(buf), "--");
  else if (ago < 120) snprintf(buf, sizeof(buf), "%lus", (unsigned long)ago);
  else if (ago < 7200) snprintf(buf, sizeof(buf), "%lum", (unsigned long)(ago / 60));
  else snprintf(buf, sizeof(buf), "%luh", (unsigned long)(ago / 3600));
  tft.drawString(fitPx(buf, 64), 248, 203);
  tft.setTextFont(2);
}

// ===========================================================================
// 5. SETTINGS
// ===========================================================================
struct MenuBtn { int x, y, w, h; };
const int MENU_N = 9;
const MenuBtn MENU_BTNS[MENU_N] = {
  {4, 30, 154, 52},  {162, 30, 154, 52},
  {4, 86, 154, 52},  {162, 86, 154, 52},
  {4, 142, 154, 52}, {162, 142, 154, 52},
  {4, 198, 52, 38},  {186, 198, 130, 38}, {60, 198, 122, 38},
};
enum { MB_WIFI, MB_LOCATE, MB_CALIBRATE, MB_BRIGHT, MB_LED, MB_NAV, MB_BACK, MB_UPDATE, MB_THEME };

int menuButtonAt(int sx, int sy) {
  for (int i = 0; i < MENU_N; i++) {
    const MenuBtn& b = MENU_BTNS[i];
    if (sx >= b.x - 2 && sx < b.x + b.w + 2 && sy >= b.y - 2 && sy < b.y + b.h + 2) return i;
  }
  return -1;
}

void menuCard(int i, const char* title, const char* sub, uint16_t edge) {
  const MenuBtn& b = MENU_BTNS[i];
  tft.fillRoundRect(b.x, b.y, b.w, b.h, 6, C_PANEL);
  tft.drawRoundRect(b.x, b.y, b.w, b.h, 6, edge);
  if (edge != C_EDGE) tft.drawRoundRect(b.x + 1, b.y + 1, b.w - 2, b.h - 2, 5, edge);
  int ix = b.x + 19, iy = b.y + b.h / 2;
  switch (i) {
    case MB_WIFI:      iconWifi(ix, iy + 7, 3, C_CYAN, C_TRACK, C_PANEL); break;
    case MB_LOCATE:    iconPin(ix, iy - 1, C_CYAN, C_PANEL); break;
    case MB_CALIBRATE: iconCrosshair(ix, iy, C_CYAN); break;
    case MB_BRIGHT:    iconSun(ix, iy, 5, C_CYAN); break;
    case MB_LED:       iconBell(ix, iy, C_CYAN); break;
    case MB_NAV:       iconGear(ix, iy, C_CYAN, C_PANEL); break;
  }
  tft.setFreeFont(&FreeSansBold9pt7b); tft.setTextColor(C_TEXT, C_PANEL);
  tft.drawString(title, b.x + 38, b.y + 8);
  tft.setTextFont(2); tft.setTextColor(C_SOFT, C_PANEL);
  tft.drawString(fitPx(sub, b.w - 56), b.x + 38, b.y + 29);
  chevron(b.x + b.w - 14, iy, C_SOFT);
}

void screenSettings(bool newPage) {
  drawHeader("SETTINGS", newPage);

  char name[48]; bool ledAlerts;
  if (configMutex) xSemaphoreTake(configMutex, portMAX_DELAY);
  strcpy(name, homeName); ledAlerts = ledCfg.alerts;
  if (configMutex) xSemaphoreGive(configMutex);
  bool wifiArmed = stillBefore(wifiConfirmUntil), online = WiFi.status() == WL_CONNECTED;

  // Redraw when anything shown on the cards changes
  static uint32_t lastKey = 0xFFFFFFFF;
  static char lastStatus[48] = "";
  bool updArmed = stillBefore(updConfirmUntil), updAvail = ghAvailable();
  uint32_t key = (wifiArmed ? 1 : 0) | (ledAlerts ? 2 : 0) | (navButtons ? 4 : 0) | (online ? 8 : 0) |
                 ((uint32_t)themeIdx << 24) |
                 ((uint32_t)ghState << 4) | (updArmed ? 0x40 : 0) | (updAvail ? 0x80 : 0) |
                 ((uint32_t)backlightPct << 8) | ((uint32_t)(name[0] + name[1] * 3 + ghLatest[1] * 5) << 16);
  if (!newPage && key == lastKey && strcmp(lastStatus, statusMsg) == 0) return;
  lastKey = key; strcpy(lastStatus, statusMsg);

  tft.setTextPadding(0); tft.setTextDatum(TL_DATUM);
  if (newPage) tft.fillRect(0, 26, 320, 214, C_BG);
  char buf[40];
  String wifiSub = wifiArmed ? String("Tap again") : online ? WiFi.localIP().toString() : String("Not connected");
  menuCard(MB_WIFI, "WiFi", wifiSub.c_str(), wifiArmed ? C_WARN : C_EDGE);
  bool locating = strcmp(statusMsg, "Locating...") == 0;
  menuCard(MB_LOCATE, "Location", locating ? "Locating..." : name, C_EDGE);
  menuCard(MB_CALIBRATE, "Calibrate", strcmp(statusMsg, "Touch calibrated") == 0 ? "Saved" : "Touch screen", C_EDGE);
  snprintf(buf, sizeof(buf), "Screen %u%%", backlightPct);
  menuCard(MB_BRIGHT, "Brightness", buf, C_EDGE);
  menuCard(MB_LED, "Alerts", ledAlerts ? "LED alerts on" : "LED alerts off", C_EDGE);
  menuCard(MB_NAV, "Navigation", navButtons ? "Arrow buttons" : "Tap to flip", C_EDGE);

  const MenuBtn& bb = MENU_BTNS[MB_BACK];
  tft.fillRoundRect(bb.x, bb.y, bb.w, bb.h, 6, C_PANEL);
  tft.drawRoundRect(bb.x, bb.y, bb.w, bb.h, 6, C_EDGE);
  int ax = bb.x + bb.w / 2 - 2, ay = bb.y + bb.h / 2;
  tft.fillTriangle(ax - 8, ay, ax - 1, ay - 6, ax - 1, ay + 6, C_TEXT);
  tft.fillRect(ax - 2, ay - 1, 12, 3, C_TEXT);

  // Theme: each tap switches to the next color theme
  const MenuBtn& tb = MENU_BTNS[MB_THEME];
  tft.fillRoundRect(tb.x, tb.y, tb.w, tb.h, 6, C_PANEL);
  tft.drawRoundRect(tb.x, tb.y, tb.w, tb.h, 6, C_EDGE);
  int sy = tb.y + tb.h / 2;
  tft.fillCircle(tb.x + 13, sy, 5, C_CYAN);
  tft.fillCircle(tb.x + 21, sy, 5, C_GREEN);
  tft.drawCircle(tb.x + 21, sy, 6, C_PANEL);
  tft.setFreeFont(&FreeSansBold9pt7b); tft.setTextColor(C_TEXT, C_PANEL);
  tft.drawString(fitPx(THEMES[themeIdx].name, tb.w - 38), tb.x + 33, tb.y + 10);
  tft.setTextFont(2);

  // Update: one tap checks GitHub; when a newer release exists, tap twice to install it
  const MenuBtn& ub = MENU_BTNS[MB_UPDATE];
  uint16_t edge = updArmed ? C_WARN : updAvail ? C_GREEN : C_EDGE;
  tft.fillRoundRect(ub.x, ub.y, ub.w, ub.h, 6, C_PANEL);
  tft.drawRoundRect(ub.x, ub.y, ub.w, ub.h, 6, edge);
  if (edge != C_EDGE) tft.drawRoundRect(ub.x + 1, ub.y + 1, ub.w - 2, ub.h - 2, 5, edge);

  char ul[32];
  if (!online)                    snprintf(ul, sizeof(ul), "No WiFi");
  else if (updArmed)              snprintf(ul, sizeof(ul), "Tap again");
  else if (ghState == GH_CHECKING) snprintf(ul, sizeof(ul), "Checking...");
  else if (ghState == GH_DOWNLOADING) snprintf(ul, sizeof(ul), "Updating...");
  else if (ghState == GH_FAILED)  snprintf(ul, sizeof(ul), "Failed, retry");
  else if (updAvail)              snprintf(ul, sizeof(ul), "Update %s", ghLatest);
  else if (ghLatest[0])           snprintf(ul, sizeof(ul), "Up to date");
  else                            snprintf(ul, sizeof(ul), "Updates");
  tft.setFreeFont(&FreeSansBold9pt7b);
  tft.setTextColor(updArmed ? C_WARN : updAvail ? C_GREEN : C_TEXT, C_PANEL);
  tft.setTextDatum(TC_DATUM);
  tft.drawString(fitPx(ul, ub.w - 12), ub.x + ub.w / 2, ub.y + 10);
  tft.setTextDatum(TL_DATUM);
  tft.setTextFont(2);
}

// ===========================================================================
// 6. CLOCK (as a page, or when no aircraft are nearby)
// ===========================================================================
void clockString(char* buf, size_t n, const struct tm& lt, bool* pm = nullptr) {
  if (scr.clock24) { strftime(buf, n, "%H:%M", &lt); return; }
  int h = lt.tm_hour % 12; if (h == 0) h = 12;
  snprintf(buf, n, "%d:%02d", h, lt.tm_min);
  if (pm) *pm = lt.tm_hour >= 12;
}

// Bottom cards shared by both clock styles: weather and today's count
void clockInfoCards(int x, int y, int w, bool newPage) {
  static uint32_t lastMs = 0;
  if (!newPage && millis() - lastMs < 10000) return;
  lastMs = millis();
  int cw = (w - 4) / 2;
  card(x, y, cw, 82); card(x + cw + 4, y, cw, 82);
  label("WEATHER", x + 8, y + 6); label("TODAY", x + cw + 12, y + 6);
  char buf[40];
  if (weather.valid) {
    weatherIcon(x + 26, y + 46, weather.code);
    tft.setFreeFont(&FreeSansBold12pt7b); tft.setTextColor(C_TEXT, C_PANEL);
    snprintf(buf, sizeof(buf), "%d", (int)lroundf(weather.temp));
    tft.drawString(buf, x + 54, y + 24);
    int tw = tft.textWidth(buf); degree(x + 54 + tw + 4, y + 28, 3, C_TEXT);
    tft.setTextFont(2); tft.setTextColor(C_SOFT, C_PANEL);
    tft.drawString(fitPx(wmoText(weather.code), cw - 58), x + 54, y + 52);
  }
  uint32_t total = 0; char note[40] = "";
  if (logMutex) xSemaphoreTake(logMutex, portMAX_DELAY);
  if (dayLog.yday >= 0) {
    total = dayLog.total;
    if (dayLog.nNotable) {
      const LogNotable& n = dayLog.notable[dayLog.nNotable - 1];
      snprintf(note, sizeof(note), "%s %s", n.cs, n.type);
    }
  }
  if (logMutex) xSemaphoreGive(logMutex);
  int tx = x + cw + 12;
  tft.setFreeFont(&FreeSansBold12pt7b); tft.setTextColor(C_TEXT, C_PANEL);
  tft.drawString(withCommas(total), tx, y + 20);
  tft.setTextFont(2); tft.setTextColor(C_SOFT, C_PANEL);
  tft.drawString("aircraft", tx, y + 44);
  if (note[0]) { tft.setTextColor(C_CYAN, C_PANEL); tft.drawString(fitPx(note, cw - 16), tx, y + 60); }
}

void screenClock(bool newPage) {
  time_t now = time(nullptr); struct tm lt; localtime_r(&now, &lt);
  bool have = timeReady();
  static int lastYday = -1;
  if (have && lt.tm_yday != lastYday) { lastYday = lt.tm_yday; if (!newPage) { tft.fillScreen(C_BG); newPage = true; } }
  char title[32] = "CLOCK";
  if (have) { strftime(title, sizeof(title), "%A, %b %d", &lt); for (char* c = title; *c; c++) *c = toupper(*c); }
  drawHeader(title, newPage);

  static int lastSec = -1;
  if (newPage) { tft.fillRect(0, 26, 320, 214, C_BG); lastSec = -1; }
  if (!have) {
    if (newPage) { tft.setTextFont(2); tft.setTextColor(C_SOFT, C_BG); tft.drawString("Waiting for the time...", 90, 110); }
    return;
  }

  if (scr.clockStyle == 2) {                     // ---- analog
    const int cx = 100, cy = 131, R = 96;
    static float lh = -1, lm = -1, ls = -1;
    if (newPage) {
      tft.fillCircle(cx, cy, R, C_PANEL); tft.drawCircle(cx, cy, R, C_EDGE); tft.drawCircle(cx, cy, R - 1, C_EDGE);
      for (int k = 0; k < 60; k++) {
        float a = k * 6 * PI / 180;
        int r1 = k % 5 ? R - 6 : R - 13;
        tft.drawLine(cx + sinf(a) * r1, cy - cosf(a) * r1, cx + sinf(a) * (R - 3), cy - cosf(a) * (R - 3), k % 5 ? C_TRACK : C_SOFT);
      }
      lh = lm = ls = -1;
    }
    if (lt.tm_sec != lastSec) {
      lastSec = lt.tm_sec;
      auto hand = [&](float ang, int len, int w, uint16_t c) {
        float a = ang * PI / 180;
        int x2 = cx + sinf(a) * len, y2 = cy - cosf(a) * len;
        if (w > 1) tft.drawWideLine(cx, cy, x2, y2, w, c, C_PANEL); else tft.drawLine(cx, cy, x2, y2, c);
      };
      float h = (lt.tm_hour % 12) * 30 + lt.tm_min * 0.5f, m = lt.tm_min * 6 + lt.tm_sec * 0.1f, sd = lt.tm_sec * 6;
      if (ls >= 0) { hand(ls, R - 18, 1, C_PANEL); hand(lm, R - 22, 4, C_PANEL); hand(lh, R - 46, 6, C_PANEL); }
      hand(h, R - 46, 6, C_TEXT); hand(m, R - 22, 4, C_TEXT); hand(sd, R - 18, 1, C_CYAN);
      tft.fillCircle(cx, cy, 5, C_CYAN);
      lh = h; lm = m; ls = sd;

      char t[8]; bool pm = false; clockString(t, sizeof(t), lt, &pm);
      tft.setFreeFont(&FreeSansBold18pt7b); tft.setTextColor(C_TEXT, C_BG); tft.setTextDatum(TC_DATUM);
      tft.setTextPadding(112); tft.drawString(t, 262, 40); tft.setTextPadding(0);
      tft.setTextFont(2); tft.setTextColor(C_SOFT, C_BG);
      char d[16]; strftime(d, sizeof(d), "%b %d", &lt);
      tft.setTextPadding(112); tft.drawString(scr.clock24 ? String(d) : String(pm ? "PM  " : "AM  ") + d, 262, 76); tft.setTextPadding(0);
      tft.setTextDatum(TL_DATUM);
    }
    clockInfoCards(204, 104, 112, newPage);
    return;
  }

  // ---- digital (optionally with seconds)
  static char lastT[8] = "";
  if (newPage) lastT[0] = '\0';
  char t[8]; bool pm = false; clockString(t, sizeof(t), lt, &pm);
  if (strcmp(t, lastT) != 0) {
    strcpy(lastT, t);
    tft.setTextDatum(TC_DATUM); tft.setTextColor(C_TEXT, C_BG); tft.setTextPadding(262);
    tft.drawString(t, 140, 44, 8);
    tft.setTextPadding(0);
    if (!scr.clock24) {
      tft.setFreeFont(&FreeSansBold9pt7b); tft.setTextColor(C_CYAN, C_BG);
      tft.drawString(pm ? "PM" : "AM", 296, 50);
    }
    tft.setTextDatum(TL_DATUM);
  }
  if (scr.clockStyle == 1 && lt.tm_sec != lastSec) {
    lastSec = lt.tm_sec;
    char sbuf[4]; snprintf(sbuf, sizeof(sbuf), "%02d", lt.tm_sec);
    tft.setFreeFont(&FreeSansBold18pt7b); tft.setTextColor(C_CYAN, C_BG); tft.setTextDatum(TC_DATUM);
    tft.setTextPadding(48); tft.drawString(sbuf, 294, 86); tft.setTextPadding(0);
    tft.setTextDatum(TL_DATUM);
  }
  clockInfoCards(4, 150, 312, newPage);
}

// ===========================================================================
// 7. METAR (nearest or chosen station)
// ===========================================================================
void screenMetar(bool newPage) {
  static char lastId[5] = ""; static uint32_t lastObs = 0; static int lastAgeMin = -1;
  if (strcmp(lastId, metar.id) != 0) { strcpy(lastId, metar.id); if (!newPage) { tft.fillScreen(C_BG); newPage = true; } }
  char title[24]; snprintf(title, sizeof(title), metar.valid ? "METAR %s" : "METAR", metar.id);
  drawHeader(title, newPage);
  if (newPage) { tft.fillRect(0, 26, 320, 214, C_BG); lastObs = 0; lastAgeMin = -1; }

  if (!metar.valid) {
    static char lastErr[48] = "";
    if (newPage || strcmp(lastErr, metarErr) != 0) {
      strcpy(lastErr, metarErr);
      tft.fillRect(0, 100, 320, 40, C_BG);
      tft.setTextFont(2); tft.setTextDatum(TC_DATUM); tft.setTextColor(C_SOFT, C_BG);
      tft.drawString(metarErr[0] ? metarErr : "Loading weather report...", 160, 104);
      tft.setTextDatum(TL_DATUM);
    }
    return;
  }

  int ageMin = timeReady() && metar.obs ? (int)((time(nullptr) - (time_t)metar.obs) / 60) : -1;
  if (metar.obs != lastObs) {
    lastObs = metar.obs; lastAgeMin = -1;
    tft.fillRect(0, 26, 320, 214, C_BG);
    card(4, 29, 312, 46);
    tft.setTextFont(2); tft.setTextColor(C_TEXT, C_PANEL);
    tft.drawString(fitPx(metar.name, 230), 12, 34);
    // Flight category pill
    uint16_t cc = C_PILLOK, cf = TFT_BLACK;
    if (!strcmp(metar.cat, "MVFR")) { cc = C_BLUEB; cf = TFT_WHITE; }
    else if (!strcmp(metar.cat, "IFR")) { cc = TFT_RED; cf = TFT_WHITE; }
    else if (!strcmp(metar.cat, "LIFR")) { cc = TFT_MAGENTA; cf = TFT_WHITE; }
    if (metar.cat[0]) pill(250, 41, 58, 22, metar.cat, cc, cf);

    char v[24];
    const int TW = 101, TH = 58;
    int tx[3] = {4, 109, 214};
    for (int r = 0; r < 2; r++) for (int c = 0; c < 3; c++) card(tx[c], 79 + r * (TH + 4), c == 2 ? 102 : TW, TH);
    label("WIND KT", tx[0] + 8, 85); label("VISIBILITY", tx[1] + 8, 85); label("CEILING FT", tx[2] + 8, 85);
    label("TEMP / DEW", tx[0] + 8, 147); label("ALTIMETER", tx[1] + 8, 147); label("OBSERVED", tx[2] + 8, 147);

    tft.setFreeFont(&FreeSansBold12pt7b); tft.setTextColor(C_TEXT, C_PANEL);
    if (metar.wspd == 0) snprintf(v, sizeof(v), "Calm");
    else if (metar.wdir < 0) snprintf(v, sizeof(v), metar.wgst ? "VRB %dG%d" : "VRB %d", metar.wspd, metar.wgst);
    else snprintf(v, sizeof(v), metar.wgst ? "%03d/%dG%d" : "%03d/%d", metar.wdir, metar.wspd, metar.wgst);
    tft.drawString(fitPx(v, TW - 12), tx[0] + 8, 102);
    snprintf(v, sizeof(v), "%s SM", metar.visib);
    tft.drawString(fitPx(v, TW - 12), tx[1] + 8, 102);
    tft.setFreeFont(&FreeSansBold9pt7b);
    tft.drawString(fitPx(metar.ceiling, 90), tx[2] + 8, 106);

    tft.setFreeFont(&FreeSansBold12pt7b);
#if UNITS_IMPERIAL
    snprintf(v, sizeof(v), "%d/%d", (int)lroundf(metar.temp * 9 / 5 + 32), (int)lroundf(metar.dewp * 9 / 5 + 32));
#else
    snprintf(v, sizeof(v), "%d/%d", (int)lroundf(metar.temp), (int)lroundf(metar.dewp));
#endif
    tft.drawString(v, tx[0] + 8, 164);
    tft.setTextFont(2); tft.setTextColor(C_SOFT, C_PANEL);
    tft.drawString(UNITS_IMPERIAL ? "deg F" : "deg C", tx[0] + 8 + 2, 188);
    tft.setFreeFont(&FreeSansBold12pt7b); tft.setTextColor(C_TEXT, C_PANEL);
#if UNITS_IMPERIAL
    snprintf(v, sizeof(v), "%.2f", metar.altim * 0.0295300f);
    tft.drawString(v, tx[1] + 8, 164);
    tft.setTextFont(2); tft.setTextColor(C_SOFT, C_PANEL); tft.drawString("inHg", tx[1] + 10, 188);
#else
    snprintf(v, sizeof(v), "%d", (int)lroundf(metar.altim));
    tft.drawString(v, tx[1] + 8, 164);
    tft.setTextFont(2); tft.setTextColor(C_SOFT, C_PANEL); tft.drawString("hPa", tx[1] + 10, 188);
#endif

    // Raw report, wrapped by words into 3 lines
    card(4, 203, 312, 35);
    tft.setTextFont(1); tft.setTextColor(C_SOFT, C_PANEL);
    const char* p = metar.raw; int line = 0;
    while (*p && line < 3) {
      int len = strlen(p), cut = len > 50 ? 50 : len;
      if (len > 50) { while (cut > 0 && p[cut] != ' ') cut--; if (cut == 0) cut = 50; }
      char buf[52]; memcpy(buf, p, cut); buf[cut] = '\0';
      tft.drawString(buf, 10, 207 + line * 10);
      p += cut; while (*p == ' ') p++; line++;
    }
  }
  if (ageMin != lastAgeMin) {
    lastAgeMin = ageMin;
    char v[24];
    tft.fillRect(222, 162, 88, 40, C_PANEL);
    tft.setFreeFont(&FreeSansBold12pt7b); tft.setTextColor(ageMin > 90 ? C_WARN : C_TEXT, C_PANEL);
    if (ageMin < 0) snprintf(v, sizeof(v), "--");
    else if (ageMin < 120) snprintf(v, sizeof(v), "%d min", ageMin);
    else snprintf(v, sizeof(v), "%d h", ageMin / 60);
    tft.drawString(v, 222, 164);
    tft.setTextFont(2); tft.setTextColor(C_SOFT, C_PANEL);
    tft.drawString("ago", 224, 188);
    tft.fillRect(12, 52, 230, 18, C_PANEL);
    if (metar.obs) {
      time_t o = metar.obs; struct tm lt; localtime_r(&o, &lt);
      char t[12]; clockString(t, sizeof(t), lt);
      tft.drawString(String("Observed ") + t + "  " + metar.id, 12, 53);
    }
  }
}

// ---- Banner over the header: emergency squawk (red, blinking) or a new collection type
void updateBanner() {
  static int shown = 0; static uint32_t lastDraw = 0; static uint8_t lastScr = 255;
  Emergency em;
  if (dataMutex) xSemaphoreTake(dataMutex, portMAX_DELAY);
  em = emergency;
  if (dataMutex) xSemaphoreGive(dataMutex);
  int want = (emergencyBanner && em.on) ? 1 : stillBefore(newTypeBannerUntil) ? 2 : 0;
  if (screen == SCREEN_SETTINGS) want = 0;
  if (!want) { if (shown) { shown = 0; lastScreen = 255; } return; }
  if (want == shown && screen == lastScr && millis() - lastDraw < 250) return;
  shown = want; lastScr = screen; lastDraw = millis();
  char t[64]; uint16_t bg, fg;
  if (want == 1) {
    bool on = (millis() / 500) % 2;
    bg = on ? TFT_RED : 0x8000; fg = TFT_WHITE;
    snprintf(t, sizeof(t), "%s %s  %s  %.1f %s %s", em.squawk, em.what, em.cs, em.km / KM_PER_UNIT, DIST_UNIT, compass(em.brg));
  } else {
    bg = C_CYAN; fg = C_BG;
    snprintf(t, sizeof(t), "%s", newTypeBanner);
  }
  tft.fillRect(0, 0, 320, 25, bg);
  tft.setFreeFont(&FreeSansBold9pt7b); tft.setTextColor(fg, bg); tft.setTextDatum(MC_DATUM);
  tft.drawString(fitPx(t, 312), 160, 12);
  tft.setTextDatum(TL_DATUM); tft.setTextFont(2);
}

void render() {
  bool newPage = false;
  if (screen != lastScreen) {
    tft.fillScreen(C_BG);
    lastScreen = screen;
    newPage = true;
  }

  // Take mutex for the duration of rendering so we see a consistent snapshot
  if (dataMutex) xSemaphoreTake(dataMutex, portMAX_DELAY);
  switch (screen) {
    case S_NEAREST: screenTargetIntel(newPage); break;
    case S_TRAFFIC: screenTop5(newPage);        break;
    case S_RADAR:   screenRadar(newPage);       break;
    case S_WEATHER: screenWeatherSystem(newPage); break;
    case S_CLOCK:   screenClock(newPage);       break;
    case S_METAR:   screenMetar(newPage);       break;
    case SCREEN_SETTINGS: screenSettings(newPage); break;
  }
  if (dataMutex) xSemaphoreGive(dataMutex);
}

// ---------------------------------------------------------------------------
// Back RGB LED
// ---------------------------------------------------------------------------
struct RGB { uint8_t r, g, b; };

RGB alertColor(AlertKind k) {
  switch (k) {
    case ALERT_EMERG:       return {255, 0, 0};      // red
    case ALERT_WATCH:       return {255, 255, 255};  // white
    case ALERT_MILITARY:    return {255, 0, 255};    // magenta
    case ALERT_INTERESTING: return {255, 80, 0};     // orange
    case ALERT_HELI:        return {0, 60, 255};     // blue
    case ALERT_LOWALT:      return {0, 255, 200};    // cyan
    default:                return {0, 0, 0};
  }
}

void ledInit() {
  ledcSetup(LED_CH_R, 5000, 8); ledcAttachPin(LED_PIN_R, LED_CH_R);
  ledcSetup(LED_CH_G, 5000, 8); ledcAttachPin(LED_PIN_G, LED_CH_G);
  ledcSetup(LED_CH_B, 5000, 8); ledcAttachPin(LED_PIN_B, LED_CH_B);
  ledcWrite(LED_CH_R, 255); ledcWrite(LED_CH_G, 255); ledcWrite(LED_CH_B, 255);  // off (active LOW)
}

// level: extra dimming in percent on top of the configured brightness
void ledSet(RGB c, uint8_t brightness, uint8_t level = 100) {
  static RGB last = {1, 1, 1};
  uint32_t k = (uint32_t)brightness * level;           // 0..10000
  RGB v = { (uint8_t)(c.r * k / 10000), (uint8_t)(c.g * k / 10000), (uint8_t)(c.b * k / 10000) };
  if (v.r == last.r && v.g == last.g && v.b == last.b) return;
  last = v;
  ledcWrite(LED_CH_R, 255 - v.r);
  ledcWrite(LED_CH_G, 255 - v.g);
  ledcWrite(LED_CH_B, 255 - v.b);
}

// Remembers which aircraft already triggered a blink, so each one blinks once
bool alreadyAlerted(const char* callsign) {
  struct Seen { char cs[10]; uint32_t ms; };
  static Seen seen[16] = {};
  const uint32_t FORGET_MS = 15UL * 60 * 1000;
  int freeIdx = 0; uint32_t oldest = 0xFFFFFFFF;
  for (int i = 0; i < 16; i++) {
    if (seen[i].cs[0] && strcmp(seen[i].cs, callsign) == 0 && millis() - seen[i].ms < FORGET_MS) {
      seen[i].ms = millis();
      return true;
    }
    if (seen[i].ms < oldest) { oldest = seen[i].ms; freeIdx = i; }
  }
  strcpy(seen[freeIdx].cs, callsign);
  seen[freeIdx].ms = millis();
  return false;
}

// Called every loop on Core 1; non-blocking
void ledUpdate() {
  static uint32_t blinkUntil = 0, testUntil = 0;
  static RGB blinkColor = {0, 0, 0};
  uint32_t now = millis();

  LedConfig lc;
  if (configMutex) xSemaphoreTake(configMutex, portMAX_DELAY);
  lc = ledCfg;
  if (configMutex) xSemaphoreGive(configMutex);

  AlertInfo al; bool haveNearest; double nearestKm;
  if (dataMutex) xSemaphoreTake(dataMutex, portMAX_DELAY);
  al = currentAlert; haveNearest = nearest.valid; nearestKm = nearest.distanceKm;
  if (dataMutex) xSemaphoreGive(dataMutex);

  // "Test LED" from the web page: cycle through the alert colors for 3 s
  if (ledTestRequested) { ledTestRequested = false; testUntil = now + 3000; }
  if ((int32_t)(testUntil - now) > 0) {
    static const RGB cycle[] = {{255,0,0},{0,255,0},{0,60,255},{255,0,255},{255,255,255}};
    ledSet(cycle[(now / 300) % 5], lc.brightness);
    return;
  }

  // Night mode: LED off
  if (nightCfg.ledOff && nightNow()) { ledSet({0, 0, 0}, lc.brightness); return; }

  // Emergency squawk in the area: fast red blink while it lasts
  bool emOn;
  if (dataMutex) xSemaphoreTake(dataMutex, portMAX_DELAY);
  emOn = emergency.on;
  if (dataMutex) xSemaphoreGive(dataMutex);
  if (lc.alerts && emOn) { ledSet(((now / 200) % 2) ? RGB{255, 0, 0} : RGB{0, 0, 0}, lc.brightness); return; }

  // New special aircraft in range -> blink 8 times in its color
  if (lc.alerts && al.kind != ALERT_NONE && !alreadyAlerted(al.callsign)) {
    blinkUntil = now + 8 * 2 * 150;
    blinkColor = alertColor(al.kind);
    Serial.printf("LED alert: %s %s %s %.1f km\n", alertName(al.kind), al.callsign, al.type, al.distKm);
  }
  if ((int32_t)(blinkUntil - now) > 0) {
    bool on = ((blinkUntil - now) / 150) % 2;
    ledSet(on ? blinkColor : RGB{0, 0, 0}, lc.brightness);
    return;
  }

  // While it stays in range: steady glow at half brightness
  if (lc.alerts && al.kind != ALERT_NONE) {
    ledSet(alertColor(al.kind), lc.brightness, 50);
    return;
  }

  // Proximity colors: red inside the alert range, yellow up to 3x, green beyond
  if (lc.proximity && haveNearest) {
    double r = lc.alertRange * KM_PER_UNIT;
    if (nearestKm <= r)          ledSet({255, 0, 0}, lc.brightness);
    else if (nearestKm <= 3 * r) ledSet({255, 120, 0}, lc.brightness);
    else                         ledSet({0, 255, 0}, lc.brightness, 40);
    return;
  }

  ledSet({0, 0, 0}, 0);
}

// ---------------------------------------------------------------------------
// WiFi setup portal (WiFiManager)
// ---------------------------------------------------------------------------
// Draw a QR code as black modules on a white square with a 2-module quiet zone,
// as large as fits in maxPx, with its right edge at rightX.
// Returns the size drawn in pixels (0 on failure).
int drawQrCode(const char* text, int rightX, int y, int maxPx) {
  const int MAXVER = 6;   // up to ~100 characters at ECC level M
  uint8_t qr[cydqr_BUFFER_LEN_FOR_VERSION(MAXVER)];
  uint8_t tmp[cydqr_BUFFER_LEN_FOR_VERSION(MAXVER)];
  if (!cydqr_encodeText(text, tmp, qr, cydqr_Ecc_MEDIUM, cydqr_VERSION_MIN, MAXVER,
                        cydqr_Mask_AUTO, true)) return 0;

  const int border = 2;
  int n = cydqr_getSize(qr);
  int scale = maxPx / (n + 2 * border);
  if (scale < 1) return 0;
  int px = (n + 2 * border) * scale;
  int x = rightX - px;

  tft.fillRect(x, y, px, px, TFT_WHITE);
  for (int my = 0; my < n; my++)
    for (int mx = 0; mx < n; mx++)
      if (cydqr_getModule(qr, mx, my))
        tft.fillRect(x + (mx + border) * scale, y + (my + border) * scale, scale, scale, TFT_BLACK);
  return px;
}

// WiFi QR payload: WIFI:T:nopass;S:<ssid>;;  with \ ; , : " escaped in the SSID
String wifiQrPayload(const char* ssid) {
  String s = "WIFI:T:nopass;S:";
  for (const char* p = ssid; *p; p++) {
    if (strchr("\\;,:\"", *p)) s += '\\';
    s += *p;
  }
  return s + ";;";
}

void drawPortalScreen(WiFiManager*) {
  tft.fillScreen(TFT_BLACK);
  tft.setTextDatum(TL_DATUM);
  tft.setTextPadding(0);
  tft.setTextFont(4); tft.setTextColor(TFT_GREEN, TFT_BLACK);
  tft.drawString("WIFI SETUP", 10, 8);

  // Instructions on the left (each line <= 120 px in font 2, so they clear the QR)
  tft.setTextFont(2);
  int y = 42;
  auto step = [&](const char* num) {
    tft.setTextColor(TFT_GREEN, TFT_BLACK); tft.drawString(num, 6, y);
  };
  auto line = [&](const char* s, uint16_t color) {
    tft.setTextColor(color, TFT_BLACK); tft.drawString(s, 22, y); y += 17;
  };
  step("1."); line("Scan with your", TFT_WHITE);
  line("phone camera to", TFT_WHITE);
  line("join the network", TFT_WHITE);
  line(SETUP_AP_NAME, TFT_YELLOW);
  y += 6;
  step("2."); line("Setup page opens", TFT_WHITE);
  line("(or 192.168.4.1)", TFT_LIGHTGREY);
  y += 6;
  step("3."); line("Choose network,", TFT_WHITE);
  line("enter password", TFT_WHITE);
  line("and your city", TFT_WHITE);

  tft.setTextColor(TFT_DARKGREY, TFT_BLACK);
  tft.drawString("Closes after 5 min", 6, 222);

  // QR on the right: the phone camera offers to join the setup network,
  // then the setup page opens by itself (captive portal)
  drawQrCode(wifiQrPayload(SETUP_AP_NAME).c_str(), 314, 40, 176);
}

// onDemand = started from the settings screen (otherwise: boot without working WiFi)
bool runWifiPortal(bool onDemand) {
  WiFiManager wm;
  WiFiManagerParameter locParam("location", "Your city, or lat,lon (optional)", "", 63);
  wm.addParameter(&locParam);
  wm.setTitle("CYD Plane Spotter");
  wm.setConfigPortalTimeout(300);
  wm.setConnectTimeout(20);
  wm.setAPCallback(drawPortalScreen);

  bool ok;
  if (onDemand) {
    portalActive = true;           // no HTTPS requests while the WiFi mode changes
    delay(300);
    server.stop();                 // the portal needs port 80
    drawPortalScreen(nullptr);
    ok = wm.startConfigPortal(SETUP_AP_NAME);
  } else {
    ok = wm.autoConnect(SETUP_AP_NAME);
  }

  if (WiFi.status() != WL_CONNECTED) {   // portal timed out: go back to saved network
    WiFi.mode(WIFI_STA);
    WiFi.begin();
  }
  if (ok && WiFi.status() == WL_CONNECTED) {
    if (applyLocationInput(locParam.getValue())) setStatus("Location updated");
    else setStatus("WiFi saved");
  }
  if (onDemand) { server.begin(); portalActive = false; refreshRequested = true; }

  tft.fillScreen(TFT_BLACK);
  lastScreen = 255;   // full redraw of the current page
  return ok;
}

void connectWiFiAtBoot() {
  WiFi.mode(WIFI_STA);
  tft.fillScreen(TFT_BLACK);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setTextFont(4);
  tft.drawString("Connecting WiFi...", 20, 100);

  // Credentials from config.env, if any, are tried first
  if (strlen(WIFI_SSID) > 0 && strcmp(WIFI_SSID, "yourWifiId") != 0) {
    WiFi.begin(WIFI_SSID, WIFI_PASS);
    for (int i = 0; i < 30 && WiFi.status() != WL_CONNECTED; i++) delay(500);
  }
  // Otherwise saved credentials, and if those fail, the setup portal
  if (WiFi.status() != WL_CONNECTED && !runWifiPortal(false)) {
    tft.fillScreen(TFT_BLACK);
    tft.setTextColor(TFT_RED, TFT_BLACK);
    tft.drawString("Setup timed out", 20, 90);
    tft.drawString("Restarting...", 20, 120);
    delay(3000);
    ESP.restart();
  }
  WiFi.setAutoReconnect(true);
  tft.fillScreen(TFT_BLACK);
  lastScreen = 255;
}

// ---------------------------------------------------------------------------
// Touch
// ---------------------------------------------------------------------------
void applyBacklight(uint8_t pct) {
  if (pct == backlightApplied) return;
  backlightApplied = pct;
  ledcWrite(BL_CH, (uint32_t)pct * 255 / 100);
}

void setBacklight(uint8_t pct) {
  backlightPct = pct;
  applyBacklight(pct);
}

// Is it night now, by the configured schedule or by today's sunset/sunrise?
bool nightNow() {
  time_t now = time(nullptr);
  if (nightCfg.mode == 0 || now < 1700000000) return false;
  if (nightCfg.mode == 2) {
    uint32_t rise = weather.sunrise, set = weather.sunset;
    if (!rise || !set) return false;
    return (uint32_t)now < rise || (uint32_t)now >= set;
  }
  struct tm lt; localtime_r(&now, &lt);
  int m = lt.tm_hour * 60 + lt.tm_min;
  if (nightCfg.fromMin == nightCfg.toMin) return false;
  return nightCfg.fromMin < nightCfg.toMin ? (m >= nightCfg.fromMin && m < nightCfg.toMin)
                                           : (m >= nightCfg.fromMin || m < nightCfg.toMin);
}

// Screen is dimmed for the night right now (not woken by a tap)
bool nightDimmed() { return nightCfg.level < backlightPct && nightNow() && !stillBefore(nightWakeUntil); }

// Picks the active theme (Night Red at night if enabled); a change redraws the page
void themeUpdate() {
  const Theme* want = &THEMES[themeIdx];
  if (nightRedTheme && nightNow()) want = &THEMES[THEME_NIGHT_RED];
  if (want != th) { th = want; lastScreen = 255; }
}

void nightUpdate() {
  applyBacklight(nightDimmed() ? nightCfg.level : backlightPct);
}

// Raw touch reading: median of 5 samples, rejects light touches and SPI noise
bool readTouchRaw(int& rx, int& ry) {
  if (!ts.touched()) return false;
  int xs[5], ys[5], n = 0;
  for (int i = 0; i < 5; i++) {
    TS_Point p = ts.getPoint();
    if (p.z > 400 && p.x > 100 && p.x < 4000 && p.y > 100 && p.y < 4000) { xs[n] = p.x; ys[n] = p.y; n++; }
  }
  if (n < 3) return false;
  std::sort(xs, xs + n); std::sort(ys, ys + n);
  rx = xs[n / 2]; ry = ys[n / 2];
  return true;
}

void mapTouch(int rx, int ry, int& sx, int& sy) {
  sx = constrain((int)lroundf(touchCal[0] * rx + touchCal[1] * ry + touchCal[2]), 0, 319);
  sy = constrain((int)lroundf(touchCal[3] * rx + touchCal[4] * ry + touchCal[5]), 0, 239);
}

// Wait for a fresh press (after release) and return its raw median; false on timeout
bool waitPress(int& rx, int& ry, uint32_t timeoutMs) {
  uint32_t t0 = millis();
  while (ts.touched()) { if (millis() - t0 > timeoutMs) return false; delay(10); }   // release first
  while (millis() - t0 < timeoutMs) {
    if (readTouchRaw(rx, ry)) {
      // Average for ~250 ms while held, for a steadier point
      long sx = 0, sy = 0; int n = 0; uint32_t t1 = millis();
      int x, y;
      while (millis() - t1 < 250) { if (readTouchRaw(x, y)) { sx += x; sy += y; n++; } delay(10); }
      if (n > 0) { rx = sx / n; ry = sy / n; }
      while (ts.touched()) delay(10);
      return true;
    }
    delay(10);
  }
  return false;
}

void drawCross(int x, int y, uint16_t c) {
  tft.drawFastHLine(x - 12, y, 25, c); tft.drawFastVLine(x, y - 12, 25, c);
  tft.drawCircle(x, y, 6, c);
}

// 3-point touch calibration with a test screen; saved to NVS on SAVE
void runTouchCalibration() {
  const int tx[3] = {30, 290, 160}, ty[3] = {30, 120, 212};
  int rx[3], ry[3];
  for (;;) {
    for (int i = 0; i < 3; i++) {
      tft.fillScreen(TFT_BLACK);
      tft.setTextDatum(MC_DATUM); tft.setTextPadding(0); tft.setTextFont(2);
      tft.setTextColor(TFT_WHITE, TFT_BLACK);
      tft.drawString("TOUCH CALIBRATION", 160, 70);
      tft.setTextColor(TFT_LIGHTGREY, TFT_BLACK);
      char m[40]; snprintf(m, sizeof(m), "Tap the center of the cross (%d/3)", i + 1);
      tft.drawString(m, 160, 92);
      tft.drawString("Use a stylus or fingernail", 160, 110);
      tft.setTextDatum(TL_DATUM);
      drawCross(tx[i], ty[i], TFT_YELLOW);
      if (!waitPress(rx[i], ry[i], 30000)) {
        Serial.println("Calibration timed out, keeping the old one");
        tft.fillScreen(TFT_BLACK); lastScreen = 255; return;
      }
      drawCross(tx[i], ty[i], TFT_GREEN);
      Serial.printf("Cal point %d: screen(%d,%d) raw(%d,%d)\n", i + 1, tx[i], ty[i], rx[i], ry[i]);
      delay(300);
    }

    // Solve sx = a*rx + b*ry + c and sy = d*rx + e*ry + f through the 3 points (Cramer's rule)
    double det = (double)rx[0] * (ry[1] - ry[2]) - (double)ry[0] * (rx[1] - rx[2]) + ((double)rx[1] * ry[2] - (double)rx[2] * ry[1]);
    float cal[6];
    bool ok = fabs(det) > 1000;
    if (ok) {
      for (int k = 0; k < 2; k++) {
        const int* t = k == 0 ? tx : ty;
        cal[3*k+0] = (float)((t[0] * (ry[1] - ry[2]) - ry[0] * (t[1] - t[2]) + ((double)t[1] * ry[2] - (double)t[2] * ry[1])) / det);
        cal[3*k+1] = (float)((rx[0] * (t[1] - t[2]) - t[0] * (rx[1] - rx[2]) + ((double)rx[1] * t[2] - (double)rx[2] * t[1])) / det);
        cal[3*k+2] = (float)((rx[0] * ((double)ry[1] * t[2] - (double)ry[2] * t[1]) - ry[0] * ((double)rx[1] * t[2] - (double)rx[2] * t[1])
                              + t[0] * ((double)rx[1] * ry[2] - (double)rx[2] * ry[1])) / det);
      }
    }
    if (!ok) {
      tft.fillScreen(TFT_BLACK); tft.setTextDatum(MC_DATUM); tft.setTextColor(TFT_RED, TFT_BLACK);
      tft.drawString("Points too close - let's try again", 160, 120); tft.setTextDatum(TL_DATUM);
      delay(1500); continue;
    }

    // Test screen: dots where you tap; SAVE or REDO
    float old[6]; memcpy(old, touchCal, sizeof(old));
    memcpy(touchCal, cal, sizeof(cal));
    tft.fillScreen(TFT_BLACK);
    tft.setTextDatum(MC_DATUM); tft.setTextColor(TFT_WHITE, TFT_BLACK); tft.setTextFont(2);
    tft.drawString("Tap anywhere to test - dots show where", 160, 20);
    tft.drawString("Then press SAVE (or REDO)", 160, 38);
    tft.fillRoundRect(10, 190, 140, 42, 6, C_BADGE);  tft.setTextColor(TFT_WHITE, C_BADGE);  tft.drawString("REDO", 80, 211);
    tft.fillRoundRect(170, 190, 140, 42, 6, 0x0400);  tft.setTextColor(TFT_WHITE, 0x0400);  tft.drawString("SAVE", 240, 211);
    tft.setTextDatum(TL_DATUM);
    int result = 0;   // 1 save, 2 redo, 0 timeout
    uint32_t t0 = millis();
    while (!result && millis() - t0 < 60000) {
      int x, y, sx, sy;
      if (!waitPress(x, y, 1000)) continue;
      mapTouch(x, y, sx, sy);
      t0 = millis();
      if (sy >= 186 && sx < 155) result = 2;
      else if (sy >= 186 && sx >= 165) result = 1;
      else tft.fillCircle(sx, sy, 3, TFT_GREEN);
    }
    if (result == 1) {
      prefs.putBytes("tcal", touchCal, sizeof(touchCal));
      Serial.printf("Touch calibration saved: %.5f %.5f %.2f / %.5f %.5f %.2f\n",
                    touchCal[0], touchCal[1], touchCal[2], touchCal[3], touchCal[4], touchCal[5]);
      setStatus("Touch calibrated");
      break;
    }
    if (result == 0) { memcpy(touchCal, old, sizeof(old)); break; }   // timed out: keep old
    memcpy(touchCal, old, sizeof(old));                               // redo
  }
  tft.fillScreen(TFT_BLACK);
  lastScreen = 255;
}

void openSettings() {
  if (screen != SCREEN_SETTINGS) screenBeforeSettings = screen;
  screen = SCREEN_SETTINGS;
  statusMsg[0] = '\0';
  wifiConfirmUntil = 0;
  updConfirmUntil = 0;
}

void handleMenuButton(int b) {
  switch (b) {
    case MB_WIFI:
      // WiFi setup interrupts tracking, so it needs a confirming second tap
      if (stillBefore(wifiConfirmUntil)) { wifiConfirmUntil = 0; portalRequested = true; }
      else wifiConfirmUntil = millis() + 3000;
      break;
    case MB_LOCATE:
      if (WiFi.status() == WL_CONNECTED) { setStatus("Locating..."); ipLocateRequested = true; }
      else setStatus("No WiFi connection");
      break;
    case MB_CALIBRATE:
      calibrateRequested = true;
      break;
    case MB_BRIGHT: {
      static const uint8_t levels[] = {100, 60, 30, 10};
      int k = 0; while (k < 4 && levels[k] != backlightPct) k++;
      setBacklight(levels[(k + 1) % 4]);
      prefs.putUChar("bl", backlightPct);
      break;
    }
    case MB_LED:
      if (configMutex) xSemaphoreTake(configMutex, portMAX_DELAY);
      ledCfg.alerts = !ledCfg.alerts;
      prefs.putBool("led_alert", ledCfg.alerts);
      if (configMutex) xSemaphoreGive(configMutex);
      break;
    case MB_NAV:
      navButtons = !navButtons;
      prefs.putBool("navbtn", navButtons);
      break;
    case MB_THEME:
      themeIdx = (themeIdx + 1) % NUM_THEMES;
      prefs.putUChar("theme", themeIdx);
      break;
    case MB_UPDATE:
      if (WiFi.status() != WL_CONNECTED) break;
      if (ghState == GH_CHECKING || ghState == GH_DOWNLOADING) break;
      if (ghAvailable()) {
        // Installing restarts the device, so it needs a confirming second tap
        if (stillBefore(updConfirmUntil)) {
          updConfirmUntil = 0;
          ghMsg[0] = '\0'; ghPct = 0; ghState = GH_DOWNLOADING; ghUpdateRequested = true;
        } else updConfirmUntil = millis() + 3000;
      } else {
        ghState = GH_CHECKING; ghCheckRequested = true;   // "check now"
      }
      break;
    case MB_BACK:
      screen = screenBeforeSettings;
      break;
  }
}

// ---- Pages: the four main ones, plus clock / METAR when shown "as a page" ----
bool pageEnabled(int i) {
  if (i == S_CLOCK) return scr.clockShow == SHOW_PAGE;
  if (i == S_METAR) return scr.metarShow == SHOW_PAGE;
  return i >= 0 && i < NUM_SCREENS;
}
uint8_t nextPage(uint8_t from, int dir) {
  int i = from >= NUM_SCREENS ? 0 : from;
  for (int k = 0; k < NUM_SCREENS; k++) {
    i = (i + dir + NUM_SCREENS) % NUM_SCREENS;
    if (pageEnabled(i)) return i;
  }
  return S_NEAREST;
}

// Select an aircraft (by ICAO address) and show its card
void selectAircraft(const char* hex, const Blip* b) {
  if (dataMutex) xSemaphoreTake(dataMutex, portMAX_DELAY);
  copyAscii(selectedHex, hex, sizeof(selectedHex));
  selectedUntil = millis() + SELECT_MS;
  if (b) {                           // show what the radar knows right away; details follow
    Aircraft a = {};
    a.valid = true; copyAscii(a.hex, hex, sizeof(a.hex));
    copyAscii(a.callsign, b->callsign[0] ? b->callsign : "(no id)", sizeof(a.callsign));
    a.lat = b->lat; a.lon = b->lon; a.altFt = b->altFt; a.gsKt = b->speedKt; a.trackDeg = b->track;
    a.distanceKm = b->distKm; a.bearingDeg = bearingDeg(homeLat, homeLon, b->lat, b->lon);
    if (b->heli) strcpy(a.category, "A7");
    selAc = a;
  }
  if (dataMutex) xSemaphoreGive(dataMutex);
  selectRequested = true;
  screen = S_NEAREST;
}

// ---- On-screen navigation arrows ----
const int NAV_Y = 95, NAV_H = 50, NAV_W = 36;
bool navVisible() { return navButtons && screen != SCREEN_SETTINGS && stillBefore(navShownUntil); }

void drawNavArrow(int x, bool right) {
  tft.fillRoundRect(x, NAV_Y, NAV_W, NAV_H, 8, C_BLUEB);
  tft.drawRoundRect(x, NAV_Y, NAV_W, NAV_H, 8, TFT_LIGHTGREY);
  int cx = x + NAV_W / 2, cy = NAV_Y + NAV_H / 2;
  if (right) tft.fillTriangle(cx - 6, cy - 11, cx - 6, cy + 11, cx + 8, cy, TFT_WHITE);
  else       tft.fillTriangle(cx + 6, cy - 11, cx + 6, cy + 11, cx - 8, cy, TFT_WHITE);
}

// Drawn after each frame while visible; when they time out, the page is redrawn clean
void updateNavOverlay() {
  static bool wasVisible = false;
  bool vis = navVisible();
  if (vis) { drawNavArrow(2, false); drawNavArrow(320 - NAV_W - 2, true); }
  else if (wasVisible) lastScreen = 255;
  wasVisible = vis;
}

// Acts once per press (on touch-down), using the calibrated mapping
void handleTap(int sx, int sy);

void checkTouch() {
  static bool wasDown = false;
  int rx, ry;
  bool down = readTouchRaw(rx, ry);
  if (!down) { if (!ts.touched()) wasDown = false; return; }
  if (wasDown) return;                       // still the same press
  wasDown = true;
  if (millis() - lastTouchMs < 150) return;  // bounce
  lastTouchMs = millis();

  // Night mode: a tap on the dimmed screen only wakes it
  if (nightNow()) {
    bool dimmed = nightDimmed();
    nightWakeUntil = millis() + NIGHT_WAKE_MS;
    if (dimmed) return;
  }

  int sx, sy;
  mapTouch(rx, ry, sx, sy);
  Serial.printf("Touch: raw(%d,%d) -> screen(%d,%d)\n", rx, ry, sx, sy);
  handleTap(sx, sy);
}

// A tap at screen coordinates: from the touch panel, or from the web page's screen mirror
void handleTap(int sx, int sy) {

  // Top-right corner (clock / WiFi icon): settings
  if (sy < 30 && sx >= 250) {
    if (screen == SCREEN_SETTINGS) screen = screenBeforeSettings; else openSettings();
    return;
  }

  if (screen == SCREEN_SETTINGS) {
    int b = menuButtonAt(sx, sy);
    if (b >= 0) handleMenuButton(b);
    return;                                  // taps elsewhere on the menu do nothing
  }

  // Visible navigation arrows come first: they are drawn over the page
  bool arrowsShown = navVisible();
  if (arrowsShown && sy >= NAV_Y - 15 && sy < NAV_Y + NAV_H + 15) {
    if (sx < NAV_W + 20)        { navShownUntil = millis() + NAV_TIMEOUT_MS; screen = nextPage(screen, -1); return; }
    if (sx >= 320 - NAV_W - 20) { navShownUntil = millis() + NAV_TIMEOUT_MS; screen = nextPage(screen, 1); return; }
  }
  if (navButtons) navShownUntil = millis() + NAV_TIMEOUT_MS;

  // Radar screen: range preset buttons on the left
  if (screen == S_RADAR && sx < RB_X + RB_W + 6 && sy >= RB_Y - 3) {
    int k = constrain((sy - RB_Y + 3) / RB_STEP, 0, 3);
    if (configMutex) xSemaphoreTake(configMutex, portMAX_DELAY);
    radarRange = RANGE_PRESETS[k];
    prefs.putFloat("range_u", radarRange);
    if (configMutex) xSemaphoreGive(configMutex);
    return;
  }

  // Traffic screen: tap a row to show that aircraft on the first page
  // (the header strip and empty space below the rows still flip to the next page)
  if (screen == S_TRAFFIC && sy >= TR_Y) {
    int row = (sy - TR_Y) / TR_STEP;
    if (row < top5Count && sy - TR_Y - row * TR_STEP <= TR_H) {
      char hex[8]; strcpy(hex, top5[row].hex);
      selectAircraft(hex, nullptr);
      return;
    }
  }

  // Radar: tap an aircraft to open its card (the nearest blip within 18 px)
  if (screen == S_RADAR && sx >= SCOPE_X && sx < SCOPE_X + SCOPE && sy >= SCOPE_Y && sy < SCOPE_Y + SCOPE) {
    int best = -1, bestD = 18 * 18;
    for (int i = 0; i < blipCount; i++) {
      if (!blipVis[i] || !blips[i].hex[0]) continue;
      int dx = blipSX[i] - sx, dy = blipSY[i] - sy, dd = dx * dx + dy * dy;
      if (dd < bestD) { bestD = dd; best = i; }
    }
    if (best >= 0) { Blip b = blips[best]; selectAircraft(b.hex, &b); return; }
  }

  // Navigation arrows mode: other taps only show the arrows; only the arrows switch pages
  if (navButtons) return;

  // Anywhere else: next screen
  screen = nextPage(screen, 1);
}

// ---------------------------------------------------------------------------
// Firmware update over WiFi (browser upload and PlatformIO/ArduinoOTA)
// ---------------------------------------------------------------------------
void drawOtaScreen(const char* line1, const char* line2, int pct) {
  static int lastPct = -2;
  if (pct >= 0 && pct == lastPct) return;   // avoid redrawing on every chunk
  if (pct < 0 || lastPct < 0) {
    tft.fillScreen(TFT_BLACK);
    tft.setTextDatum(TL_DATUM); tft.setTextPadding(0);
    tft.setTextFont(4); tft.setTextColor(TFT_GREEN, TFT_BLACK);
    tft.drawString("FIRMWARE UPDATE", 10, 20);
    tft.drawRect(10, 120, 300, 24, TFT_DARKGREY);
  }
  lastPct = pct;
  tft.setTextFont(2); tft.setTextPadding(300);
  tft.setTextColor(TFT_WHITE, TFT_BLACK); tft.drawString(line1, 10, 70);
  tft.setTextColor(TFT_LIGHTGREY, TFT_BLACK); tft.drawString(line2, 10, 90);
  tft.setTextPadding(0);
  if (pct >= 0) tft.fillRect(12, 122, 296 * pct / 100, 20, TFT_GREEN);
  tft.setTextColor(TFT_DARKGREY, TFT_BLACK);
  tft.drawString("Do not unplug the board", 10, 200, 2);
}

// Browser upload state. Accepts firmware.bin or the merged image from CI:
// a merged image starts with 0xFF padding and has the app at offset 0x10000.
struct OtaUpload { size_t seen = 0, skip = 0, total = 0; bool err = false; String msg; } otaUp;

void otaFail(const char* why) {
  otaUp.err = true; otaUp.msg = why;
  Update.abort();
  Serial.printf("OTA failed: %s\n", why);
}

void handleUpdateChunk() {
  HTTPUpload& up = server.upload();
  if (up.status == UPLOAD_FILE_START) {
    otaActive = true;
    otaUp = OtaUpload();
    otaUp.total = server.hasHeader("Content-Length") ? server.header("Content-Length").toInt() : 0;
    Serial.printf("OTA upload: %s\n", up.filename.c_str());
    drawOtaScreen("Receiving firmware...", up.filename.c_str(), 0);
    if (!Update.begin(UPDATE_SIZE_UNKNOWN, U_FLASH)) otaFail(Update.errorString());
  } else if (up.status == UPLOAD_FILE_WRITE && !otaUp.err) {
    if (otaUp.seen == 0 && up.currentSize > 0 && up.buf[0] == 0xFF) {
      otaUp.skip = 0x10000;   // merged image: skip bootloader/partition table
      Serial.println("OTA: merged image, using the app part at 0x10000");
    }
    size_t start = 0;
    if (otaUp.seen < otaUp.skip) start = min((size_t)up.currentSize, otaUp.skip - otaUp.seen);
    otaUp.seen += up.currentSize;
    size_t n = up.currentSize - start;
    if (n > 0 && Update.write(up.buf + start, n) != n) { otaFail(Update.errorString()); return; }
    char l2[40]; snprintf(l2, sizeof(l2), "%u KB received", (unsigned)(otaUp.seen / 1024));
    int pct = otaUp.total > 0 ? (int)min((size_t)99, otaUp.seen * 100 / otaUp.total) : 0;
    drawOtaScreen("Receiving firmware...", l2, pct);
  } else if (up.status == UPLOAD_FILE_END && !otaUp.err) {
    if (otaUp.seen <= otaUp.skip) otaFail("file too small");
    else if (!Update.end(true)) otaFail(Update.errorString());
    else drawOtaScreen("Update complete", "Restarting...", 100);
  } else if (up.status == UPLOAD_FILE_ABORTED) {
    otaFail("upload aborted");
  }
}

void setupArduinoOta() {
  ArduinoOTA.setHostname(DEVICE_HOSTNAME);
#ifdef OTA_PASSWORD
  ArduinoOTA.setPassword(OTA_PASSWORD);
#endif
  ArduinoOTA.onStart([]() { otaActive = true; drawOtaScreen("Receiving firmware...", "from PlatformIO", 0); });
  ArduinoOTA.onProgress([](unsigned int done, unsigned int total) {
    drawOtaScreen("Receiving firmware...", "from PlatformIO", total ? (int)(done * 100ULL / total) : 0);
  });
  ArduinoOTA.onEnd([]() { drawOtaScreen("Update complete", "Restarting...", 100); });
  ArduinoOTA.onError([](ota_error_t e) {
    char b[32]; snprintf(b, sizeof(b), "Error %u", (unsigned)e);
    drawOtaScreen("Update failed", b, -1);
    delay(2000);
    otaActive = false; lastScreen = 255;
  });
  ArduinoOTA.begin();                      // also starts mDNS as <hostname>.local
  MDNS.addService("http", "tcp", 80);
  Serial.printf("OTA ready: http://%s.local/ (firmware %s, env %s)\n", DEVICE_HOSTNAME, FW_VERSION, BUILD_ENV);
}

// ---------------------------------------------------------------------------
// Previous firmware: the other OTA slot still holds the version before the last update
// ---------------------------------------------------------------------------
const esp_partition_t* previousFirmware() {
  const esp_partition_t* run = esp_ota_get_running_partition();
  const esp_partition_t* p = esp_ota_get_next_update_partition(nullptr);
  esp_app_desc_t d;
  if (!p || p == run || esp_ota_get_partition_description(p, &d) != ESP_OK) return nullptr;
  return p;
}

// Makes the previous firmware boot next (the image is verified first). Caller restarts.
bool switchToPreviousFirmware() {
  const esp_partition_t* p = previousFirmware();
  if (!p) return false;
  esp_err_t e = esp_ota_set_boot_partition(p);
  Serial.printf("Switch to previous firmware (%s): %s\n", p->label, esp_err_to_name(e));
  return e == ESP_OK;
}

// ---------------------------------------------------------------------------
// Web Server — Config Page
// ---------------------------------------------------------------------------
void initWebServer() {
  // GET / — serve config page
  server.on("/", []() {
    double hLat, hLon; float sRad, rRange; String name; LedConfig lc;
    if (configMutex) xSemaphoreTake(configMutex, portMAX_DELAY);
    hLat = homeLat; hLon = homeLon; sRad = searchRadiusNm; rRange = radarRange; name = homeName;
    lc = ledCfg;
    if (configMutex) xSemaphoreGive(configMutex);
    auto chk = [](bool b) { return b ? String(" checked") : String(""); };
    NotifyConfig nc; ScreensConfig sc;
    if (configMutex) xSemaphoreTake(configMutex, portMAX_DELAY);
    nc = notifyCfg; sc = scr;
    if (configMutex) xSemaphoreGive(configMutex);
    auto showOpts = [](uint8_t v) {
      const char* n[] = {"Off", "As a page", "Only when no aircraft are nearby"};
      String o;
      for (int i = 0; i < 3; i++) o += String("<option value=\"") + i + "\"" + (v == i ? " selected" : "") + ">" + n[i] + "</option>";
      return o;
    };
    auto hhmm = [](uint16_t m) { char b[8]; snprintf(b, sizeof(b), "%02u:%02u", (m / 60) % 24, m % 60); return String(b); };
    auto sel = [](bool b) { return b ? String(" selected") : String(""); };
    String themeOptions;
    for (int i = 0; i < NUM_THEMES; i++)
      themeOptions += String("<option value=\"") + i + "\"" + sel(i == themeIdx) + ">" + THEMES[i].name + "</option>";
    String rollback = previousFirmware()
        ? String("<button type=\"button\" class=\"opt\" style=\"text-align:center\" onclick=\"rollback()\">Switch back to ") +
          (prevFwVersion.length() ? prevFwVersion : String("the previous firmware")) + "</button>"
        : String("");
    name.replace("&", "&amp;"); name.replace("\"", "&quot;"); name.replace("<", "&lt;");
    String ssid = WiFi.SSID();
    ssid.replace("&", "&amp;"); ssid.replace("<", "&lt;");

    String html = R"rawliteral(
<!DOCTYPE html><html><head>
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>CYD Plane Spotter</title>
<style>
  body{font-family:Arial,sans-serif;background:#0a1520;color:#e0e0e0;margin:0;padding:20px}
  h1{color:#00c8ff;text-align:center;font-size:1.4em}
  h2{color:#00c8ff;font-size:1em;margin:0 0 8px}
  .card{background:#0e1c2b;border-radius:12px;padding:20px;margin:12px 0;max-width:420px;margin-left:auto;margin-right:auto}
  label{display:block;margin:10px 0 4px;color:#aaa;font-size:0.85em}
  input{width:100%;padding:10px;border:1px solid #1f3b57;border-radius:6px;background:#08111b;color:#fff;font-size:1em;box-sizing:border-box}
  input:focus{border-color:#00c8ff;outline:none}
  button{width:100%;padding:12px;margin-top:16px;background:#00c8ff;color:#000;border:none;border-radius:8px;font-size:1.1em;font-weight:bold;cursor:pointer}
  button:hover{background:#00a8d8}
  .row{display:flex;gap:8px}.row input{flex:1}.row button{width:auto;margin-top:0;padding:10px 16px}
  .opt{background:#08111b;color:#e0e0e0;border:1px solid #1f3b57;font-weight:normal;font-size:0.95em;margin-top:6px;text-align:left}
  .opt:hover{background:#16283a}
  .danger{background:#3a1a1a;color:#ff8080}.danger:hover{background:#552222}
  .info{text-align:center;color:#666;font-size:0.8em;margin-top:12px}
  .msg{color:#00c8ff;text-align:center;display:none;position:fixed;left:50%;bottom:20px;transform:translateX(-50%);background:#0e1c2b;border:1px solid #00c8ff;padding:10px 18px;border-radius:8px}
  #res{font-size:0.9em;color:#aaa;margin-top:6px}
  .chk{display:flex;align-items:center;gap:10px;margin:10px 0;color:#e0e0e0;font-size:0.95em}
  .chk input{width:auto;margin:0}
  .sub{margin-left:26px}
  .hint{color:#777;font-size:0.8em;margin-top:4px}
  .dot{display:inline-block;width:10px;height:10px;border-radius:50%;margin-left:4px}
</style></head><body>
<h1>&#9992; CYD Plane Spotter</h1>
<div class="card">
  <h2>Find location</h2>
  <div class="row"><input id="q" placeholder="City, e.g. Lombard"><button type="button" onclick="find()">Search</button></div>
  <div id="res"></div>
</div>
<div class="card">
  <form id="cfg" onsubmit="save(event)">
    <label>Location name</label>
    <input name="name" id="name" maxlength="47" value=")rawliteral" + name + R"rawliteral(">
    <label>Home Latitude (&deg;)</label>
    <input type="number" step="any" name="lat" id="lat" value=")rawliteral" + String(hLat, 6) + R"rawliteral(" required>
    <label>Home Longitude (&deg;)</label>
    <input type="number" step="any" name="lon" id="lon" value=")rawliteral" + String(hLon, 6) + R"rawliteral(" required>
    <label>Search Radius (nautical miles, 1&ndash;250)</label>
    <input type="number" step="1" min="1" max="250" name="radius" id="radius" value=")rawliteral" + String((int)sRad) + R"rawliteral(" required>
    <label>Radar Range ()rawliteral" + String(DIST_UNIT) + ", " + String((int)RANGE_MIN) + "&ndash;" + String((int)RANGE_MAX) + R"rawliteral()</label>
    <input type="number" step="1" name="range" id="range" value=")rawliteral" + String((int)rRange) + R"rawliteral(" required>
    <button type="submit">Save Settings</button>
  </form>
  <div class="msg" id="msg"></div>
</div>
<div class="card">
  <h2>Back LED</h2>
  <label class="chk"><input type="checkbox" id="lp")rawliteral" + chk(lc.proximity) + R"rawliteral(> Color by distance to nearest aircraft</label>
  <div class="hint sub">green = far, yellow = within 3&times; alert range, red = within alert range</div>
  <label class="chk"><input type="checkbox" id="la")rawliteral" + chk(lc.alerts) + R"rawliteral(> Blink when a special aircraft comes close</label>
  <div class="sub">
    <label class="chk"><input type="checkbox" id="th")rawliteral" + chk(lc.heli) + R"rawliteral(> Helicopters <span class="dot" style="background:#0040ff"></span></label>
    <label class="chk"><input type="checkbox" id="tm")rawliteral" + chk(lc.military) + R"rawliteral(> Military <span class="dot" style="background:#ff00ff"></span></label>
    <label class="chk"><input type="checkbox" id="ti")rawliteral" + chk(lc.interesting) + R"rawliteral(> Government / rare aircraft <span class="dot" style="background:#ff5000"></span></label>
    <label>Low altitude below (ft, 0 = off) <span class="dot" style="background:#00ffc8"></span></label>
    <input type="number" step="100" min="0" max="20000" id="tl" value=")rawliteral" + String(lc.lowAltFt) + R"rawliteral(">
    <label>Watch list <span class="dot" style="background:#fff"></span></label>
    <input id="tw" maxlength="95" placeholder="e.g. B52, C17, A388, N911" value=")rawliteral" + String(lc.watch) + R"rawliteral(">
    <div class="hint">Aircraft type codes, or the start of a callsign / registration</div>
  </div>
  <label>Alert range ()rawliteral" + String(DIST_UNIT) + R"rawliteral()</label>
  <input type="number" step="0.5" min="0.5" max="50" id="lr" value=")rawliteral" + String(lc.alertRange, 1) + R"rawliteral(">
  <label>Brightness</label>
  <input type="range" min="5" max="100" id="lb" value=")rawliteral" + String(lc.brightness) + R"rawliteral(" style="padding:0">
  <button type="button" onclick="saveLed()">Save LED Settings</button>
  <button type="button" class="opt" style="text-align:center" onclick="post('/led-test','')">Test LED</button>
</div>
<div class="card">
  <h2>Today</h2>
  <div id="td" style="font-size:0.9em;color:#aaa">Loading...</div>
  <div id="tdh" style="display:flex;align-items:flex-end;gap:2px;height:60px;margin-top:10px"></div>
  <div style="display:flex;justify-content:space-between;color:#777;font-size:0.7em"><span>0</span><span>6</span><span>12</span><span>18</span><span>24</span></div>
  <div id="tdr" style="font-size:0.9em;margin-top:10px"></div>
  <div id="tdn" style="font-size:0.9em;margin-top:10px"></div>
</div>
<div class="card">
  <h2>Phone notifications</h2>
  <div class="hint">Install the <b>ntfy</b> app (iPhone / Android) and subscribe to the topic below. No account needed; anyone who knows the topic name can read it, so keep it random.</div>
  <label class="chk"><input type="checkbox" id="non")rawliteral" + chk(nc.on) + R"rawliteral(> Send notifications</label>
  <label>Topic</label>
  <div class="row"><input id="ntp" maxlength="64" value=")rawliteral" + String(nc.topic) + R"rawliteral(" placeholder="e.g. planes-7k3x9q"><button type="button" onclick="rndTopic()">Random</button></div>
  <div class="hint" id="nlink"></div>
  <label>Server</label>
  <input id="nsv" maxlength="63" value=")rawliteral" + String(nc.server) + R"rawliteral(">
  <label>Within ()rawliteral" + String(DIST_UNIT) + R"rawliteral()</label>
  <input type="number" step="0.5" min="0.5" max="100" id="nrg" value=")rawliteral" + String(nc.range, 1) + R"rawliteral(">
  <div class="sub">
    <label class="chk"><input type="checkbox" id="nc1")rawliteral" + chk(nc.cats & NT_WATCH) + R"rawliteral(> Watch list (from the Back LED card)</label>
    <label class="chk"><input type="checkbox" id="nc2")rawliteral" + chk(nc.cats & NT_MIL) + R"rawliteral(> Military</label>
    <label class="chk"><input type="checkbox" id="nc4")rawliteral" + chk(nc.cats & NT_INTR) + R"rawliteral(> Government / rare aircraft</label>
    <label class="chk"><input type="checkbox" id="nc8")rawliteral" + chk(nc.cats & NT_HELI) + R"rawliteral(> Helicopters</label>
    <label class="chk"><input type="checkbox" id="nc32")rawliteral" + chk(nc.cats & NT_EMERG) + R"rawliteral(> Emergency squawks (any distance, also at night)</label>
    <label class="chk"><input type="checkbox" id="nc64")rawliteral" + chk(nc.cats & NT_NEWTYPE) + R"rawliteral(> New type for my collection (after its first day)</label>
    <label class="chk"><input type="checkbox" id="nc16")rawliteral" + chk(nc.cats & NT_LOW) + R"rawliteral(> Lower than <input type="number" step="100" min="100" max="20000" id="nlo" value=")rawliteral" + String(nc.lowAltFt) + R"rawliteral(" style="width:90px;padding:4px;margin:0 4px"> ft</label>
  </div>
  <label>Same aircraft again after</label>
  <select id="ncl" style="width:100%;padding:10px;border:1px solid #1f3b57;border-radius:6px;background:#08111b;color:#fff;font-size:1em">
    <option value="1")rawliteral" + sel(nc.coolH == 1) + R"rawliteral(>1 hour</option>
    <option value="3")rawliteral" + sel(nc.coolH == 3) + R"rawliteral(>3 hours</option>
    <option value="6")rawliteral" + sel(nc.coolH == 6) + R"rawliteral(>6 hours</option>
    <option value="12")rawliteral" + sel(nc.coolH == 12) + R"rawliteral(>12 hours</option>
    <option value="24")rawliteral" + sel(nc.coolH == 24) + R"rawliteral(>24 hours</option>
  </select>
  <label class="chk"><input type="checkbox" id="nqt")rawliteral" + chk(nc.quietNight) + R"rawliteral(> Quiet during night mode</label>
  <div class="hint" id="nst">)rawliteral" + String(notifyStatus) + R"rawliteral(</div>
  <button type="button" onclick="saveNotify()">Save notifications</button>
  <button type="button" class="opt" style="text-align:center" onclick="testNotify()">Send test notification</button>
</div>
<div class="card">
  <h2>Screen mirror</h2>
  <label class="chk"><input type="checkbox" id="mon")rawliteral" + chk(screenMirror) + R"rawliteral( onchange="post('/mirror','on='+(this.checked?1:0),function(){location.reload()})"> Show the device screen here (tap the picture to tap the screen)</label>
  <div id="mbox" style="display:)rawliteral" + String(screenMirror ? "block" : "none") + R"rawliteral(">
    <img id="mimg" alt="Device screen" style="width:100%;aspect-ratio:4/3;border-radius:8px;border:1px solid #1f3b57;background:#000;cursor:pointer;image-rendering:pixelated" onclick="mtap(event)">
    <div class="row" style="margin-top:8px;align-items:center">
      <select id="mint" style="flex:1;padding:8px;border:1px solid #1f3b57;border-radius:6px;background:#08111b;color:#fff" onchange="msched()">
        <option value="0">Refresh by hand</option><option value="3">Every 3 s</option><option value="5" selected>Every 5 s</option><option value="10">Every 10 s</option></select>
      <button type="button" onclick="mload()">Refresh</button>
    </div>
    <label class="chk"><input type="checkbox" id="mrb" onchange="mload()"> Colors look swapped (red / blue)</label>
    <div class="hint">Each picture freezes the device screen for about a second while it is read.</div>
  </div>
</div>
<div class="card">
  <h2>Clock</h2>
  <label>Show the clock screen</label>
  <select id="scs" style="width:100%;padding:10px;border:1px solid #1f3b57;border-radius:6px;background:#08111b;color:#fff;font-size:1em">)rawliteral" + showOpts(sc.clockShow) + R"rawliteral(</select>
  <div class="row" style="margin-top:4px"><div style="flex:1"><label>Style</label><select id="sst" style="width:100%;padding:10px;border:1px solid #1f3b57;border-radius:6px;background:#08111b;color:#fff;font-size:1em">
    <option value="0")rawliteral" + sel(sc.clockStyle == 0) + R"rawliteral(>Digital</option>
    <option value="1")rawliteral" + sel(sc.clockStyle == 1) + R"rawliteral(>Digital with seconds</option>
    <option value="2")rawliteral" + sel(sc.clockStyle == 2) + R"rawliteral(>Analog</option></select></div>
  <div style="flex:1"><label>Format</label><select id="s24" style="width:100%;padding:10px;border:1px solid #1f3b57;border-radius:6px;background:#08111b;color:#fff;font-size:1em">
    <option value="0")rawliteral" + sel(!sc.clock24) + R"rawliteral(>12-hour (3:45 PM)</option>
    <option value="1")rawliteral" + sel(sc.clock24) + R"rawliteral(>24-hour (15:45)</option></select></div></div>
  <label class="chk"><input type="checkbox" id="scn")rawliteral" + chk(sc.clockAtNight) + R"rawliteral(> At night, while the screen is dimmed, show only the clock</label>
  <div class="hint">Uses the Night mode schedule below. A tap wakes the screen with the normal pages for 30 seconds.</div>
  <button type="button" onclick="saveClock()">Save clock</button>
</div>
<div class="card">
  <h2>Screens</h2>
  <label class="chk"><input type="checkbox" id="sro")rawliteral" + chk(sc.rotate) + R"rawliteral(> Flip pages automatically every <input type="number" min="5" max="600" id="srs" value=")rawliteral" + String(sc.rotateSec) + R"rawliteral(" style="width:70px;padding:4px;margin:0 4px"> s</label>
  <div class="hint">Pauses for a minute after you touch the screen.</div>
  <label>METAR screen (airport weather report)</label>
  <select id="sms" style="width:100%;padding:10px;border:1px solid #1f3b57;border-radius:6px;background:#08111b;color:#fff;font-size:1em">)rawliteral" + showOpts(sc.metarShow) + R"rawliteral(</select>
  <label>Airport (ICAO code, empty = nearest)</label>
  <input id="smi" maxlength="4" placeholder="e.g. KORD" value=")rawliteral" + String(sc.metarIcao) + R"rawliteral(" style="text-transform:uppercase">
  <label>"No aircraft nearby" means none within ()rawliteral" + String(DIST_UNIT) + R"rawliteral()</label>
  <input type="number" step="1" min="1" max="100" id="sir" value=")rawliteral" + String((int)sc.idleRange) + R"rawliteral(">
  <label class="chk"><input type="checkbox" id="sho")rawliteral" + chk(heliOnly) + R"rawliteral(> Helicopters only (radar, traffic and nearest show only helicopters)</label>
  <div class="hint sub">Alerts, notifications, emergencies, today's log and the collection still see all aircraft.</div>
  <label class="chk"><input type="checkbox" id="seb")rawliteral" + chk(emergencyBanner) + R"rawliteral(> Red banner for emergency squawks (7500 / 7600 / 7700)</label>
  <button type="button" onclick="saveScreens()">Save screens</button>
</div>
<div class="card">
  <h2>Collection</h2>
  <div id="col" style="font-size:0.9em;color:#aaa">Loading...</div>
  <input id="colq" placeholder="Filter, e.g. Boeing or B7" style="margin-top:8px" oninput="colShow()">
  <div id="coll" style="font-size:0.9em;margin-top:6px;max-height:420px;overflow:auto"></div>
  <div class="hint" style="margin-top:8px"><a style="color:#00c8ff" href="/types.csv" download="collection.csv">Download CSV</a></div>
  <button type="button" class="opt danger" style="text-align:center" onclick="colReset()">Start the collection over</button>
</div>
<div class="card">
  <h2>Theme</h2>
  <select id="thm" style="width:100%;padding:10px;border:1px solid #1f3b57;border-radius:6px;background:#08111b;color:#fff;font-size:1em">)rawliteral" + themeOptions + R"rawliteral(</select>
  <label class="chk"><input type="checkbox" id="thr")rawliteral" + chk(nightRedTheme) + R"rawliteral(> Switch to Night Red during night mode</label>
  <div class="hint">Also on the device: Settings, the button with two colored dots.</div>
  <button type="button" onclick="saveTheme()">Save theme</button>
</div>
<div class="card">
  <h2>Night mode</h2>
  <label>Dim the screen</label>
  <select id="nm" style="width:100%;padding:10px;border:1px solid #1f3b57;border-radius:6px;background:#08111b;color:#fff;font-size:1em">
    <option value="0")rawliteral" + sel(nightCfg.mode == 0) + R"rawliteral(>Off</option>
    <option value="1")rawliteral" + sel(nightCfg.mode == 1) + R"rawliteral(>On a schedule</option>
    <option value="2")rawliteral" + sel(nightCfg.mode == 2) + R"rawliteral(>From sunset to sunrise</option>
  </select>
  <div class="row" style="margin-top:4px"><div style="flex:1"><label>From</label><input type="time" id="nf" value=")rawliteral" + hhmm(nightCfg.fromMin) + R"rawliteral("></div>
  <div style="flex:1"><label>To</label><input type="time" id="nt" value=")rawliteral" + hhmm(nightCfg.toMin) + R"rawliteral("></div></div>
  <label>Night brightness: <span id="nlv">)rawliteral" + String(nightCfg.level) + R"rawliteral(</span>% (0 = screen off)</label>
  <input type="range" min="0" max="100" step="5" id="nl" value=")rawliteral" + String(nightCfg.level) + R"rawliteral(" style="padding:0" oninput="$('nlv').textContent=this.value">
  <label class="chk"><input type="checkbox" id="nled")rawliteral" + chk(nightCfg.ledOff) + R"rawliteral(> Back LED off at night</label>
  <div class="hint">A tap wakes the screen for 30 seconds.</div>
  <button type="button" onclick="saveNight()">Save night mode</button>
</div>
<div class="card">
  <h2>Firmware</h2>
  <div style="font-size:0.9em;color:#aaa">Version <b>)rawliteral" + String(FW_VERSION) + R"rawliteral(</b>, build <b>)rawliteral" + String(BUILD_ENV) + R"rawliteral(</b>. Upload <code>firmware.bin</code> or <code>*-merged.bin</code> from the <b>)rawliteral" + String(BUILD_ENV) + R"rawliteral(</b> build on GitHub.</div>
  <input type="file" id="fw" accept=".bin" style="margin-top:12px">
  <progress id="fwp" max="100" value="0" style="width:100%;margin-top:10px;display:none"></progress>
  <div id="gh" style="font-size:0.9em;color:#aaa;margin-top:12px">Checking GitHub...</div>
  <button type="button" id="ghi" style="display:none" onclick="ghInstall()">Install from GitHub</button>
  <button type="button" class="opt" style="text-align:center" onclick="ghCheck()">Check for updates</button>
  )rawliteral" + rollback + R"rawliteral(
  <div class="hint" style="margin-top:14px">Or upload a file:</div>
  <div id="fws" class="hint"></div>
  <button type="button" onclick="upload()">Update firmware</button>
  <button type="button" class="opt" style="text-align:center" onclick="post('/calibrate','',function(){flash('Calibration started on the device screen')})">Calibrate touch screen</button>
</div>
<div class="card">
  <h2>WiFi</h2>
  <div style="font-size:0.9em;color:#aaa">Connected to <b>)rawliteral" + ssid + R"rawliteral(</b>. Forgetting it restarts the device into setup mode: join the <b>)rawliteral" + String(SETUP_AP_NAME) + R"rawliteral(</b> network from your phone to pick a new one.</div>
  <button type="button" class="danger" onclick="forget()">Forget WiFi &amp; restart</button>
</div>
<div class="info">Route source adsb.lol: )rawliteral" + String(routesetStatus) + R"rawliteral(</div>
<div class="info">http://)rawliteral" + String(DEVICE_HOSTNAME) + R"rawliteral(.local &middot; IP )rawliteral" + WiFi.localIP().toString() + R"rawliteral(</div>
<script>
function $(i){return document.getElementById(i)}
function flash(t){var m=$('msg');m.textContent=t;m.style.display='block';setTimeout(function(){m.style.display='none'},3000)}
function find(){
  var q=$('q').value.trim(),r=$('res');if(q.length<2)return;
  r.textContent='Searching...';
  fetch('/geocode?q='+encodeURIComponent(q)).then(function(x){if(!x.ok)throw 0;return x.json()}).then(function(a){
    r.innerHTML='';
    if(!a.length){r.textContent='Nothing found';return}
    a.forEach(function(p){
      var b=document.createElement('button');b.type='button';b.className='opt';b.textContent=p.n;
      b.onclick=function(){$('lat').value=p.lat;$('lon').value=p.lon;$('name').value=p.n;r.textContent='Selected - press Save Settings';};
      r.appendChild(b);
    });
  }).catch(function(){r.textContent='Search failed'});
}
$('q').addEventListener('keydown',function(e){if(e.key==='Enter'){e.preventDefault();find()}});
function save(e){
  e.preventDefault();
  var x=new XMLHttpRequest();
  x.open('POST','/save',true);
  x.setRequestHeader('Content-Type','application/x-www-form-urlencoded');
  x.onload=function(){flash(x.status==200?'✓ Settings saved!':x.responseText)};
  x.send(['name','lat','lon','radius','range'].map(function(k){return k+'='+encodeURIComponent($(k).value)}).join('&'));
}
function post(u,body,cb){
  var x=new XMLHttpRequest();x.open('POST',u,true);
  x.setRequestHeader('Content-Type','application/x-www-form-urlencoded');
  x.onload=function(){if(cb)cb(x)};x.send(body);
}
function saveLed(){
  var b=function(i){return $(i).checked?'1':'0'};
  var body='lp='+b('lp')+'&la='+b('la')+'&th='+b('th')+'&tm='+b('tm')+'&ti='+b('ti')+
    '&tl='+encodeURIComponent($('tl').value)+'&tw='+encodeURIComponent($('tw').value)+
    '&lr='+encodeURIComponent($('lr').value)+'&lb='+encodeURIComponent($('lb').value);
  post('/led',body,function(x){flash(x.status==200?'✓ LED settings saved!':x.responseText)});
}
function upload(){
  var f=$('fw').files[0];if(!f){flash('Choose a .bin file first');return}
  if(!confirm('Install '+f.name+'? The device restarts when done.'))return;
  var fd=new FormData();fd.append('firmware',f,f.name);
  var x=new XMLHttpRequest(),p=$('fwp'),st=$('fws');
  p.style.display='block';p.value=0;st.textContent='Uploading...';
  x.upload.onprogress=function(e){if(e.lengthComputable){p.value=e.loaded*100/e.total;st.textContent='Uploading '+Math.round(e.loaded/1024)+' KB'}};
  x.onload=function(){
    if(x.status==200){p.value=100;st.textContent='Done. Restarting, this page reloads in 15 s...';setTimeout(function(){location.reload()},15000)}
    else{st.textContent='Failed: '+x.responseText}
  };
  x.onerror=function(){st.textContent='Upload failed (connection lost)'};
  x.open('POST','/update',true);x.send(fd);
}
var ghBusy=false;
function ghShow(s){
  var t=$('gh'),b=$('ghi');
  if(s.state=='checking'){t.textContent='Checking GitHub...';b.style.display='none';return}
  if(s.state=='downloading'){t.textContent='Installing '+s.latest+': '+s.pct+'% - the device restarts when done';b.style.display='none';return}
  var m=s.latest?'Latest release: '+s.latest+(s.available?'':' (installed)'):'Latest release: unknown';
  if(s.msg)m+=' - '+s.msg;
  t.textContent=m;
  b.style.display=s.available?'block':'none';b.textContent='Install '+s.latest+' from GitHub';
}
function ghPoll(){
  fetch('/update-status').then(function(x){return x.json()}).then(function(s){
    ghShow(s);
    if(s.state=='checking'||s.state=='downloading'){ghBusy=s.state=='downloading';setTimeout(ghPoll,1000)}
  }).catch(function(){
    if(ghBusy){$('gh').textContent='Restarting... this page reloads in 20 s';setTimeout(function(){location.reload()},20000)}
  });
}
function ghCheck(){post('/update-check','',function(){setTimeout(ghPoll,300)})}
function ghInstall(){
  if(!confirm('Download and install the latest release from GitHub? The device restarts when done.'))return;
  post('/update-github','',function(x){if(x.status!=200){flash(x.responseText);return}ghBusy=true;setTimeout(ghPoll,1000)});
}
ghPoll();
function nLink(){
  var t=$('ntp').value.trim(),s=$('nsv').value.trim().replace(/\/$/,''),h=s.replace(/^https?:\/\//,'');
  $('nlink').innerHTML=t?'On your phone: <a style="color:#00c8ff" href="ntfy://'+h+'/'+encodeURIComponent(t)+'">open in the ntfy app</a> or <a style="color:#00c8ff" href="'+s+'/'+encodeURIComponent(t)+'" target="_blank">in the browser</a>':'';
}
$('ntp').addEventListener('input',nLink);$('nsv').addEventListener('input',nLink);nLink();
function rndTopic(){var c='abcdefghijkmnpqrstuvwxyz23456789',t='planes-';for(var i=0;i<10;i++)t+=c[Math.floor(Math.random()*c.length)];$('ntp').value=t;nLink()}
function nBody(){
  var c=0;[1,2,4,8,16,32,64].forEach(function(b){if($('nc'+b).checked)c|=b});
  return 'on='+($('non').checked?1:0)+'&topic='+encodeURIComponent($('ntp').value.trim())+'&server='+encodeURIComponent($('nsv').value.trim())+
    '&range='+encodeURIComponent($('nrg').value)+'&cats='+c+'&low='+encodeURIComponent($('nlo').value)+'&cool='+$('ncl').value+'&quiet='+($('nqt').checked?1:0);
}
function saveNotify(){post('/notify',nBody(),function(x){flash(x.status==200?'✓ Notifications saved!':x.responseText)})}
function testNotify(){
  post('/notify',nBody(),function(x){
    if(x.status!=200){flash(x.responseText);return}
    $('nst').textContent='Sending...';
    post('/notify-test','',function(){setTimeout(function(){fetch('/notify-status').then(function(r){return r.text()}).then(function(t){$('nst').textContent=t})},4000)});
  });
}
var mTimer=null,mBusy=false;
function mload(){
  if(!$('mon').checked||mBusy)return;
  mBusy=true;var i=new Image();
  i.onload=function(){$('mimg').src=i.src;mBusy=false};i.onerror=function(){mBusy=false};
  i.src='/screen.bmp?rb='+($('mrb').checked?1:0)+'&t='+Date.now();
}
function msched(){
  if(mTimer)clearInterval(mTimer);mTimer=null;
  var s=+$('mint').value;
  try{localStorage.setItem('mint',s);localStorage.setItem('mrb',$('mrb').checked?1:0)}catch(e){}
  if(s>0)mTimer=setInterval(function(){if(!document.hidden)mload()},s*1000);
}
function mtap(e){
  var r=e.target.getBoundingClientRect();
  var x=Math.round((e.clientX-r.left)*320/r.width),y=Math.round((e.clientY-r.top)*240/r.height);
  post('/tap','x='+x+'&y='+y,function(){setTimeout(mload,500)});
}
if($('mon').checked){
  try{var v=localStorage.getItem('mint');if(v!==null)$('mint').value=v;$('mrb').checked=localStorage.getItem('mrb')=='1'}catch(e){}
  $('mrb').addEventListener('change',msched);
  mload();msched();
}
function saveClock(){
  post('/clock','cs='+$('scs').value+'&st='+$('sst').value+'&c24='+$('s24').value+'&cn='+($('scn').checked?1:0),
    function(x){flash(x.status==200?'✓ Clock saved!':x.responseText)});
}
function saveScreens(){
  var b=function(i){return $(i).checked?1:0};
  post('/screens','ro='+b('sro')+'&rs='+$('srs').value+
    '&ms='+$('sms').value+'&mi='+encodeURIComponent($('smi').value.trim().toUpperCase())+'&ir='+$('sir').value+'&eb='+b('seb')+'&ho='+b('sho'),
    function(x){flash(x.status==200?'✓ Screens saved!':x.responseText)});
}
var colRows=[];
function colLoad(){
  fetch('/types.csv').then(function(x){return x.ok?x.text():''}).then(function(t){
    colRows=t.split('\n').filter(function(l){return l.indexOf(',')>0}).map(function(l){var f=l.split(',');
      return {t:f[0],at:+f[1],hex:f[2]||'',cs:f[3]||'',reg:f[4]||'',m:f[5]||''}}).reverse();
    var first=colRows.length?colRows[colRows.length-1].at:0;
    $('col').innerHTML=colRows.length?'<b style="color:#fff;font-size:1.4em">'+colRows.length+'</b> aircraft types since '+new Date(first*1000).toLocaleDateString():'No types yet - they are added as aircraft fly by.';
    colShow();
  }).catch(function(){$('col').textContent='Could not load'});
}
function colShow(){
  var q=$('colq').value.trim().toLowerCase(),h='',n=0;
  colRows.forEach(function(r){
    if(q&&(r.t+' '+r.m+' '+r.cs+' '+r.reg).toLowerCase().indexOf(q)<0)return;
    if(++n>300)return;
    var L=function(u,t){return '<a style="color:#00c8ff" target="_blank" href="'+u+'">'+t+'</a>'};
    var links=[L('https://en.wikipedia.org/wiki/Special:Search?search='+encodeURIComponent(r.m||(r.t+' aircraft')),'About the type')];
    if(r.reg)links.push(L('https://www.planespotters.net/search?q='+encodeURIComponent(r.reg),'Photos of '+esc(r.reg)));
    if(r.hex)links.push(L('https://globe.airplanes.live/?icao='+r.hex,'Map'));
    h+='<div style="padding:8px 0;border-bottom:1px solid #1f3b57"><b style="color:#fff">'+esc(r.t)+'</b> '+esc(r.m)+
      ' <span style="color:#777;float:right">'+new Date(r.at*1000).toLocaleString([], {month:'short',day:'numeric',hour:'2-digit',minute:'2-digit'})+'</span><br>'+
      '<span style="color:#aaa">'+esc(r.cs)+'</span> &middot; '+links.join(' &middot; ')+'</div>';
  });
  $('coll').innerHTML=h;
}
function colReset(){
  if(!confirm('Delete the whole collection and start over?'))return;
  post('/collection-reset','',function(){setTimeout(colLoad,1500);flash('Collection cleared')});
}
colLoad();
function saveTheme(){
  post('/theme','t='+$('thm').value+'&nr='+($('thr').checked?1:0),function(x){flash(x.status==200?'✓ Theme saved!':x.responseText)});
}
function saveNight(){
  var t=function(i){var v=$(i).value.split(':');return (+v[0])*60+(+v[1]||0)};
  post('/night','m='+$('nm').value+'&f='+t('nf')+'&t='+t('nt')+'&l='+$('nl').value+'&led='+($('nled').checked?1:0),
    function(x){flash(x.status==200?'✓ Night mode saved!':x.responseText)});
}
function rollback(){
  if(!confirm('Restart into the previous firmware? Settings are kept.'))return;
  post('/rollback','',function(x){if(x.status==200){flash('Restarting...');setTimeout(function(){location.reload()},15000)}else flash(x.responseText)});
}
function esc(t){return String(t).replace(/[&<>]/g,function(c){return{'&':'&amp;','<':'&lt;','>':'&gt;'}[c]})}
function today(){
  fetch('/today').then(function(x){return x.json()}).then(function(d){
    if(!d.ok){$('td').textContent='Waiting for the clock and the first data...';return}
    $('td').innerHTML='<b style="color:#fff;font-size:1.4em">'+d.total+'</b> aircraft since '+d.since;
    var mx=Math.max.apply(null,d.hours.concat([1])),h='';
    d.hours.forEach(function(v,i){h+='<div title="'+i+':00 - '+v+'" style="flex:1;background:'+(i==d.hour?'#00c8ff':'#1f6f9f')+';height:'+Math.max(2,v*100/mx)+'%"></div>'});
    $('tdh').innerHTML=h;
    var r=function(n,o,v){return o?'<div>'+n+': <b>'+esc(o.cs)+'</b> '+esc(o.t)+' &middot; '+v+' <span style="color:#777">'+o.at+'</span></div>':''};
    $('tdr').innerHTML=r('Lowest',d.low,d.low&&d.low.alt+' ft')+r('Fastest',d.fast,d.fast&&d.fast.kt+' kt')+
      r('Closest',d.close,d.close&&d.close.d+' '+d.unit);
    var n=d.notable.map(function(a){return '<div><span style="color:#777">'+a.at+'</span> <b>'+esc(a.cs)+'</b> '+esc(a.t)+
      ' <span class="dot" style="background:'+a.c+'"></span> '+esc(a.k)+' &middot; '+a.d+' '+d.unit+'</div>'}).join('');
    if(d.more)n+='<div class="hint">and '+d.more+' more</div>';
    $('tdn').innerHTML=n?'<div style="color:#aaa;margin-bottom:4px">Notable</div>'+n:'';
  }).catch(function(){$('td').textContent='Could not load'});
}
today();setInterval(today,60000);
function forget(){
  if(!confirm('Forget the saved WiFi network and restart into setup mode?'))return;
  var x=new XMLHttpRequest();x.open('POST','/wifi-reset',true);x.send();
  flash('Restarting into setup mode...');
}
</script></body></html>)rawliteral";

    server.send(200, "text/html", html);
  });

  // GET /geocode?q=city — proxy to the Open-Meteo geocoding API
  server.on("/geocode", []() {
    String q = server.arg("q"); q.trim();
    if (q.length() < 2) { server.send(400, "text/plain", "Query too short"); return; }
    GeoResult r[5];
    int n = geocodeCity(q, r, 5);
    if (n < 0) { server.send(502, "text/plain", "Geocoding failed"); return; }
    JsonDocument d;
    JsonArray a = d.to<JsonArray>();
    for (int i = 0; i < n; i++) {
      JsonObject o = a.add<JsonObject>();
      o["n"] = r[i].name; o["lat"] = r[i].lat; o["lon"] = r[i].lon;
    }
    String out; serializeJson(d, out);
    server.send(200, "application/json", out);
  });

  // POST /save — persist new config
  server.on("/save", []() {
    if (!server.hasArg("lat") || !server.hasArg("lon") || !server.hasArg("radius") || !server.hasArg("range")) {
      server.send(400, "text/plain", "Missing fields");
      return;
    }

    double newLat = server.arg("lat").toDouble();
    double newLon = server.arg("lon").toDouble();
    float  newRad = server.arg("radius").toFloat();
    float  newRng = server.arg("range").toFloat();
    String newName = server.arg("name"); newName.trim();

    if (newLat < -90 || newLat > 90 || newLon < -180 || newLon > 180 ||
        newRad < 1 || newRad > MAX_SEARCH_NM || newRng < RANGE_MIN || newRng > RANGE_MAX) {
      server.send(400, "text/plain", "Invalid values");
      return;
    }

    if (configMutex) xSemaphoreTake(configMutex, portMAX_DELAY);
    searchRadiusNm = newRad;
    radarRange = newRng;
    prefs.putFloat("radius_nm", newRad);
    prefs.putFloat("range_u", newRng);
    if (configMutex) xSemaphoreGive(configMutex);
    setLocation(newLat, newLon, newName.c_str());

    Serial.printf("Config saved: radius=%.0f nm range=%.0f %s\n", newRad, newRng, DIST_UNIT);
    server.send(200, "text/plain", "OK");
  });

  // POST /led — back LED settings
  server.on("/led", HTTP_POST, []() {
    LedConfig n;
    n.proximity   = server.arg("lp") == "1";
    n.alerts      = server.arg("la") == "1";
    n.heli        = server.arg("th") == "1";
    n.military    = server.arg("tm") == "1";
    n.interesting = server.arg("ti") == "1";
    n.lowAltFt    = server.arg("tl").toInt();
    n.alertRange  = server.arg("lr").toFloat();
    int bright    = server.arg("lb").toInt();
    if (n.lowAltFt < 0 || n.lowAltFt > 20000 || n.alertRange < 0.5f || n.alertRange > 50 ||
        bright < 5 || bright > 100) {
      server.send(400, "text/plain", "Invalid values");
      return;
    }
    n.brightness = (uint8_t)bright;
    // Watch list: keep only A-Z, 0-9, comma and space
    String w = server.arg("tw"); w.toUpperCase();
    int j = 0;
    for (unsigned i = 0; i < w.length() && j < (int)sizeof(n.watch) - 1; i++) {
      char c = w[i];
      if (isalnum((uint8_t)c) || c == ',' || c == ' ') n.watch[j++] = c;
    }
    n.watch[j] = '\0';

    if (configMutex) xSemaphoreTake(configMutex, portMAX_DELAY);
    ledCfg = n;
    prefs.putBool("led_prox", n.proximity);
    prefs.putBool("led_alert", n.alerts);
    prefs.putBool("led_heli", n.heli);
    prefs.putBool("led_mil", n.military);
    prefs.putBool("led_intr", n.interesting);
    prefs.putInt("led_lowalt", n.lowAltFt);
    prefs.putFloat("led_range", n.alertRange);
    prefs.putUChar("led_bright", n.brightness);
    prefs.putString("led_watch", n.watch);
    if (configMutex) xSemaphoreGive(configMutex);
    refreshRequested = true;   // re-evaluate alerts right away
    server.send(200, "text/plain", "OK");
  });

  // POST /led-test — cycle the LED through its colors for 3 s
  server.on("/led-test", HTTP_POST, []() {
    ledTestRequested = true;
    server.send(200, "text/plain", "OK");
  });

  // POST /update — firmware upload from the browser
  server.on("/update", HTTP_POST, []() {
    bool ok = !otaUp.err;
    server.sendHeader("Connection", "close");
    server.send(ok ? 200 : 500, "text/plain", ok ? "OK" : otaUp.msg);
    if (ok) { delay(1000); ESP.restart(); }
    drawOtaScreen("Update failed", otaUp.msg.c_str(), -1);
    delay(2500);
    otaActive = false; lastScreen = 255;   // back to normal operation
  }, handleUpdateChunk);
  const char* hdrs[] = {"Content-Length"};
  server.collectHeaders(hdrs, 1);

  // Update from GitHub: status, check now, install the latest release
  server.on("/update-status", HTTP_GET, []() {
    static const char* names[] = {"idle", "checking", "downloading", "failed"};
    JsonDocument d;
    d["cur"] = FW_VERSION; d["env"] = BUILD_ENV;
    d["latest"] = ghLatest; d["state"] = names[ghState]; d["pct"] = (int)ghPct; d["msg"] = ghMsg;
    d["available"] = ghAvailable();
    String out; serializeJson(d, out);
    server.send(200, "application/json", out);
  });
  server.on("/update-check", HTTP_POST, []() {
    if (ghState == GH_IDLE || ghState == GH_FAILED) { ghState = GH_CHECKING; ghCheckRequested = true; }
    server.send(200, "text/plain", "OK");
  });
  server.on("/update-github", HTTP_POST, []() {
    if (!ghLatest[0]) { server.send(409, "text/plain", "No release found yet - press Check for updates"); return; }
    if (ghState == GH_DOWNLOADING) { server.send(409, "text/plain", "Update already running"); return; }
    ghMsg[0] = '\0'; ghPct = 0; ghState = GH_DOWNLOADING; ghUpdateRequested = true;
    server.send(200, "text/plain", "OK");
  });

  // POST /notify — phone notification settings
  server.on("/notify", HTTP_POST, []() {
    NotifyConfig n;
    n.on = server.arg("on") == "1";
    String topic = server.arg("topic"), srv = server.arg("server");
    topic.trim(); srv.trim();
    while (srv.endsWith("/")) srv.remove(srv.length() - 1);
    for (unsigned i = 0; i < topic.length(); i++) {
      char c = topic[i];
      if (!isalnum((uint8_t)c) && c != '-' && c != '_') { server.send(400, "text/plain", "Topic: letters, digits, - and _ only"); return; }
    }
    if (topic.length() > 64) { server.send(400, "text/plain", "Topic is too long"); return; }
    if (n.on && topic.isEmpty()) { server.send(400, "text/plain", "Enter a topic first"); return; }
    if (!(srv.startsWith("https://") || srv.startsWith("http://")) || srv.length() > 63) {
      server.send(400, "text/plain", "Server must start with https://"); return;
    }
    n.range = server.arg("range").toFloat();
    n.cats = server.arg("cats").toInt() & 127;
    n.lowAltFt = server.arg("low").toInt();
    int cool = server.arg("cool").toInt();
    n.quietNight = server.arg("quiet") == "1";
    if (n.range < 0.5f || n.range > 100 || n.lowAltFt < 100 || n.lowAltFt > 20000 || cool < 1 || cool > 24) {
      server.send(400, "text/plain", "Invalid values"); return;
    }
    n.coolH = cool;
    copyAscii(n.topic, topic.c_str(), sizeof(n.topic));
    copyAscii(n.server, srv.c_str(), sizeof(n.server));
    if (configMutex) xSemaphoreTake(configMutex, portMAX_DELAY);
    notifyCfg = n;
    prefs.putBool("ntf_on", n.on);
    prefs.putString("ntf_topic", n.topic);
    prefs.putString("ntf_srv", n.server);
    prefs.putFloat("ntf_range", n.range);
    prefs.putUChar("ntf_cats", n.cats);
    prefs.putInt("ntf_low", n.lowAltFt);
    prefs.putUChar("ntf_cool", n.coolH);
    prefs.putBool("ntf_quiet", n.quietNight);
    if (configMutex) xSemaphoreGive(configMutex);
    server.send(200, "text/plain", "OK");
  });
  server.on("/notify-test", HTTP_POST, []() {
    snprintf(notifyStatus, sizeof(notifyStatus), "Sending...");
    notifyTestRequested = true;
    server.send(200, "text/plain", "OK");
  });
  server.on("/notify-status", HTTP_GET, []() { server.send(200, "text/plain", notifyStatus); });

  // POST /screens — page rotation, clock and METAR screens, emergency banner
  server.on("/screens", HTTP_POST, []() {
    ScreensConfig c;
    c.rotate = server.arg("ro") == "1";
    int rs = server.arg("rs").toInt(), ms = server.arg("ms").toInt();
    float ir = server.arg("ir").toFloat();
    String mi = server.arg("mi"); mi.trim(); mi.toUpperCase();
    bool icaoOk = mi.length() == 0 || mi.length() == 4;
    for (unsigned i = 0; i < mi.length(); i++) if (!isalnum((uint8_t)mi[i])) icaoOk = false;
    if (rs < 5 || rs > 600 || ms < 0 || ms > 2 || ir < 1 || ir > 100 || !icaoOk) {
      server.send(400, "text/plain", "Invalid values (airport: 4-letter ICAO code or empty)"); return;
    }
    c.rotateSec = rs;
    c.clockShow = scr.clockShow; c.clockStyle = scr.clockStyle; c.clock24 = scr.clock24; c.clockAtNight = scr.clockAtNight;
    c.metarShow = ms; copyAscii(c.metarIcao, mi.c_str(), sizeof(c.metarIcao)); c.idleRange = ir;
    bool metarChanged = strcmp(c.metarIcao, scr.metarIcao) != 0 || (c.metarShow != SHOW_OFF && scr.metarShow == SHOW_OFF);
    if (configMutex) xSemaphoreTake(configMutex, portMAX_DELAY);
    scr = c;
    if (configMutex) xSemaphoreGive(configMutex);
    emergencyBanner = server.arg("eb") == "1";
    bool ho = server.arg("ho") == "1";
    if (ho != heliOnly) { heliOnly = ho; refreshRequested = true; }   // refilter right away
    prefs.putBool("heli_only", heliOnly);
    prefs.putBool("rot_on", c.rotate); prefs.putUShort("rot_sec", c.rotateSec);
    prefs.putUChar("mtr_show", c.metarShow); prefs.putString("mtr_icao", c.metarIcao); prefs.putFloat("idle_rng", c.idleRange);
    prefs.putBool("emg_on", emergencyBanner);
    if (metarChanged) { if (dataMutex) xSemaphoreTake(dataMutex, portMAX_DELAY); metar = Metar(); if (dataMutex) xSemaphoreGive(dataMutex); metarErr[0] = '\0'; metarRequested = true; }
    lastScreen = 255;
    server.send(200, "text/plain", "OK");
  });

  // Screen mirror: on/off, the current screen as a BMP, and taps from the page
  server.on("/mirror", HTTP_POST, []() {
    screenMirror = server.arg("on") == "1";
    prefs.putBool("mirror", screenMirror);
    server.send(200, "text/plain", "OK");
  });
  server.on("/screen.bmp", HTTP_GET, []() {
    if (!screenMirror) { server.send(403, "text/plain", "Screen mirror is off"); return; }
    const int W = 320, H = 240, ROWS = 4;
    uint16_t* buf = (uint16_t*)malloc(W * ROWS * 2);
    if (!buf) { server.send(503, "text/plain", "Out of memory"); return; }
    bool rb = server.arg("rb") == "1";
    // 16-bit BMP (RGB565 bit fields), stored top-down
    uint8_t h[66] = {0};
    auto put32 = [&](int o, uint32_t v) { for (int i = 0; i < 4; i++) h[o + i] = (v >> (8 * i)) & 0xFF; };
    h[0] = 'B'; h[1] = 'M';
    put32(2, 66 + W * H * 2); put32(10, 66);
    put32(14, 40); put32(18, W); put32(22, (uint32_t)(-H));
    h[26] = 1; h[28] = 16; put32(30, 3); put32(34, W * H * 2); put32(38, 2835); put32(42, 2835);
    put32(54, rb ? 0x001F : 0xF800); put32(58, 0x07E0); put32(62, rb ? 0xF800 : 0x001F);
    server.setContentLength(66 + W * H * 2);
    server.sendHeader("Cache-Control", "no-store");
    server.send(200, "image/bmp", "");
    server.sendContent((const char*)h, sizeof(h));
    for (int y = 0; y < H; y += ROWS) {
      tft.readRect(0, y, W, ROWS, buf);
      for (int i = 0; i < W * ROWS; i++) buf[i] = (buf[i] >> 8) | (buf[i] << 8);   // to little-endian RGB565
      server.sendContent((const char*)buf, W * ROWS * 2);
    }
    free(buf);
  });
  server.on("/tap", HTTP_POST, []() {
    if (!screenMirror) { server.send(403, "text/plain", "Screen mirror is off"); return; }
    int x = server.arg("x").toInt(), y = server.arg("y").toInt();
    if (x < 0 || x > 319 || y < 0 || y > 239) { server.send(400, "text/plain", "Bad position"); return; }
    lastTouchMs = millis();
    if (nightNow()) nightWakeUntil = millis() + NIGHT_WAKE_MS;
    Serial.printf("Web tap: (%d,%d)\n", x, y);
    handleTap(x, y);
    server.send(200, "text/plain", "OK");
  });

  // POST /clock — clock screen: when to show it, style, 12/24 h, clock-only at night
  server.on("/clock", HTTP_POST, []() {
    int cs = server.arg("cs").toInt(), st = server.arg("st").toInt();
    if (cs < 0 || cs > 2 || st < 0 || st > 2) { server.send(400, "text/plain", "Invalid values"); return; }
    if (configMutex) xSemaphoreTake(configMutex, portMAX_DELAY);
    scr.clockShow = cs; scr.clockStyle = st; scr.clock24 = server.arg("c24") == "1";
    scr.clockAtNight = server.arg("cn") == "1";
    if (configMutex) xSemaphoreGive(configMutex);
    prefs.putUChar("clk_show", scr.clockShow); prefs.putUChar("clk_style", scr.clockStyle);
    prefs.putBool("clk_24", scr.clock24); prefs.putBool("clk_night", scr.clockAtNight);
    lastScreen = 255;
    server.send(200, "text/plain", "OK");
  });

  // Collection: the CSV file, and starting over
  server.on("/types.csv", HTTP_GET, []() {
    File f = fsOk ? LittleFS.open("/types.csv", "r") : File();
    if (!f) { server.send(200, "text/csv", ""); return; }
    server.streamFile(f, "text/csv");
    f.close();
  });
  server.on("/collection-reset", HTTP_POST, []() {
    collectionResetRequested = true;
    server.send(200, "text/plain", "OK");
  });

  // POST /theme — color theme
  server.on("/theme", HTTP_POST, []() {
    int t = server.arg("t").toInt();
    if (t < 0 || t >= NUM_THEMES) { server.send(400, "text/plain", "Invalid theme"); return; }
    themeIdx = t; nightRedTheme = server.arg("nr") == "1";
    prefs.putUChar("theme", themeIdx);
    prefs.putBool("night_red", nightRedTheme);
    server.send(200, "text/plain", "OK");
  });

  // POST /night — night mode settings
  server.on("/night", HTTP_POST, []() {
    int m = server.arg("m").toInt(), f = server.arg("f").toInt(), t = server.arg("t").toInt(), l = server.arg("l").toInt();
    if (m < 0 || m > 2 || f < 0 || f >= 1440 || t < 0 || t >= 1440 || l < 0 || l > 100) {
      server.send(400, "text/plain", "Invalid values"); return;
    }
    nightCfg.mode = m; nightCfg.fromMin = f; nightCfg.toMin = t; nightCfg.level = l;
    nightCfg.ledOff = server.arg("led") == "1";
    prefs.putUChar("night_mode", nightCfg.mode);
    prefs.putUShort("night_from", nightCfg.fromMin);
    prefs.putUShort("night_to", nightCfg.toMin);
    prefs.putUChar("night_lvl", nightCfg.level);
    prefs.putBool("night_led", nightCfg.ledOff);
    server.send(200, "text/plain", "OK");
  });

  // GET /today — today's log as JSON
  server.on("/today", HTTP_GET, []() {
    auto hm = [](time_t t) { char b[6]; struct tm lt; localtime_r(&t, &lt); strftime(b, sizeof(b), "%H:%M", &lt); return String(b); };
    JsonDocument d;
    if (logMutex) xSemaphoreTake(logMutex, portMAX_DELAY);
    d["ok"] = dayLog.yday >= 0;
    if (dayLog.yday >= 0) {
      time_t now = time(nullptr); struct tm lt; localtime_r(&now, &lt);
      d["total"] = dayLog.total; d["since"] = hm(dayLog.since); d["hour"] = lt.tm_hour;
      d["unit"] = DIST_UNIT;
      JsonArray h = d["hours"].to<JsonArray>();
      for (int i = 0; i < 24; i++) h.add(dayLog.hours[i]);
      auto rec = [&](const char* key, const LogRecord& r) {
        if (!r.at) return;
        JsonObject o = d[key].to<JsonObject>();
        o["cs"] = r.cs[0] ? r.cs : "?"; o["t"] = r.type; o["alt"] = r.alt; o["kt"] = (int)lroundf(r.kt);
        o["d"] = serialized(String(r.km / KM_PER_UNIT, 1)); o["at"] = hm(r.at);
      };
      rec("low", dayLog.lowest); rec("fast", dayLog.fastest); rec("close", dayLog.closest);
      JsonArray n = d["notable"].to<JsonArray>();
      for (int i = dayLog.nNotable - 1; i >= 0; i--) {     // newest first
        const LogNotable& a = dayLog.notable[i];
        JsonObject o = n.add<JsonObject>();
        o["cs"] = a.cs; o["t"] = a.type; o["k"] = alertName(a.kind); o["at"] = hm(a.first);
        o["d"] = serialized(String(a.km / KM_PER_UNIT, 1));
        RGB c = alertColor(a.kind); char hex[8]; snprintf(hex, sizeof(hex), "#%02x%02x%02x", c.r, c.g, c.b);
        o["c"] = hex;
      }
      d["more"] = dayLog.moreNotable;
    }
    if (logMutex) xSemaphoreGive(logMutex);
    String out; serializeJson(d, out);
    server.send(200, "application/json", out);
  });

  // POST /rollback — restart into the firmware in the other OTA slot
  server.on("/rollback", HTTP_POST, []() {
    if (!switchToPreviousFirmware()) { server.send(500, "text/plain", "No valid previous firmware"); return; }
    server.sendHeader("Connection", "close");
    server.send(200, "text/plain", "OK");
    delay(1000);
    ESP.restart();
  });

  // POST /calibrate — run touch calibration on the device
  server.on("/calibrate", HTTP_POST, []() {
    calibrateRequested = true;
    server.send(200, "text/plain", "OK");
  });

  // POST /wifi-reset — forget saved WiFi and reboot into the setup portal
  server.on("/wifi-reset", HTTP_POST, []() {
    server.send(200, "text/plain", "OK");
    delay(500);
    WiFiManager wm;
    wm.resetSettings();
    ESP.restart();
  });

  server.begin();
  Serial.println("Web server started on http://" + WiFi.localIP().toString() + "/");
}

// ---------------------------------------------------------------------------
// Main Arduino Loop
// ---------------------------------------------------------------------------
void setup() {
  Serial.begin(115200);

  // Mutexes first: setLocation() may run during WiFi setup
  dataMutex = xSemaphoreCreateMutex();
  configMutex = xSemaphoreCreateMutex();
  logMutex = xSemaphoreCreateMutex();

  touchSpi.begin(XPT2046_CLK, XPT2046_MISO, XPT2046_MOSI, -1);
  ts.begin(touchSpi);
  ts.setRotation(1);

  Serial.println();
  Serial.println("=== CYD Plane Spotter boot ===");
#if defined(ST7789_DRIVER)
  Serial.println("Display driver: ST7789 (env cyd2usb)");
#elif defined(ILI9341_2_DRIVER)
  Serial.println("Display driver: ILI9341_2 (env cyd)");
#else
  Serial.println("Display driver: ILI9341 (env cyd-alt)");
#endif
  tft.begin();
  tft.setRotation(1);
  tft.invertDisplay(DISPLAY_INVERT);

  // Identify the panel controller: ILI9341 answers 0xD3 with 00 93 41,
  // ST7789 answers 0x04 with 85 85 52. Helps pick the right firmware.
  Serial.printf("Panel ID 0x04: %02X %02X %02X   0xD3: %02X %02X %02X\n",
                tft.readcommand8(0x04, 1), tft.readcommand8(0x04, 2), tft.readcommand8(0x04, 3),
                tft.readcommand8(0xD3, 1), tft.readcommand8(0xD3, 2), tft.readcommand8(0xD3, 3));

  // Quick visible self-test: red, green, blue. If these don't show up
  // (screen stays white or black), this build's driver doesn't match the panel.
  const uint16_t testColors[] = {TFT_RED, TFT_GREEN, TFT_BLUE};
  for (uint16_t c : testColors) { tft.fillScreen(c); delay(150); }

  radarSpr.setColorDepth(8);   // 40 KB instead of 80 KB: leaves room for TLS connections
  radarSpr.createSprite(SCOPE, SCOPE);


  tft.fillScreen(TFT_BLACK);
  tft.setTextFont(4); tft.setTextColor(TFT_GREEN);
  tft.drawString("CYD PLANE SPOTTER", 20, 100);
  delay(1500);

  for (int i = 0; i < ROUTE_CACHE_SIZE; i++) routeCache[i].valid = false;

  // Load persisted config (or use defaults) before WiFi setup, which may change the location.
  // New keys: older firmware stored the radius in degrees and the range in km.
  prefs.begin("cyd-spotter", false);

  // Firmware history: remember the version that ran before this one ("switch back" label)
  String fwCur = prefs.isKey("fw_cur") ? prefs.getString("fw_cur") : String("");
  if (fwCur != FW_VERSION) {
    if (fwCur.length()) prefs.putString("fw_prev", fwCur);
    prefs.putString("fw_cur", FW_VERSION);
  }
  if (prefs.isKey("fw_prev")) prevFwVersion = prefs.getString("fw_prev");

  // Crash-loop guard: 3 crashes in a row within a minute of boot -> back to the previous firmware
  esp_reset_reason_t rr = esp_reset_reason();
  bool crashed = rr == ESP_RST_PANIC || rr == ESP_RST_INT_WDT || rr == ESP_RST_TASK_WDT || rr == ESP_RST_WDT;
  uint8_t bootFails = crashed ? prefs.getUChar("bootfail", 0) + 1 : 0;
  prefs.putUChar("bootfail", bootFails);
  if (bootFails >= 3) {
    prefs.putUChar("bootfail", 0);
    Serial.printf("Crashed %u times in a row after boot\n", bootFails);
    if (switchToPreviousFirmware()) {
      tft.fillScreen(TFT_BLACK); tft.setTextFont(4); tft.setTextColor(TFT_ORANGE, TFT_BLACK);
      tft.drawString("Firmware keeps crashing", 10, 90);
      tft.drawString("Going back to the", 10, 120);
      tft.drawString("previous version...", 10, 150);
      delay(3000);
      ESP.restart();
    }
  }
  // Read only keys that exist: the ESP32 Preferences library logs an error for
  // every missing key, which floods the serial log on first boot.
  #define PREF(key, getter, var) if (prefs.isKey(key)) var = prefs.getter(key, var)
  PREF("lat", getDouble, homeLat);
  PREF("lon", getDouble, homeLon);
  if (prefs.isKey("name")) copyAscii(homeName, prefs.getString("name").c_str(), sizeof(homeName));
  PREF("radius_nm", getFloat, searchRadiusNm);
  PREF("range_u", getFloat, radarRange);
  searchRadiusNm = constrain(searchRadiusNm, 1.0f, MAX_SEARCH_NM);
  radarRange = constrain(radarRange, RANGE_MIN, RANGE_MAX);
  Serial.printf("Config loaded: %s lat=%.6f lon=%.6f radius=%.0f nm range=%.0f %s\n",
                homeName, homeLat, homeLon, searchRadiusNm, radarRange, DIST_UNIT);

  PREF("led_prox", getBool, ledCfg.proximity);
  PREF("led_alert", getBool, ledCfg.alerts);
  PREF("led_heli", getBool, ledCfg.heli);
  PREF("led_mil", getBool, ledCfg.military);
  PREF("led_intr", getBool, ledCfg.interesting);
  PREF("led_lowalt", getInt, ledCfg.lowAltFt);
  PREF("led_range", getFloat, ledCfg.alertRange);
  PREF("led_bright", getUChar, ledCfg.brightness);
  if (prefs.isKey("led_watch")) copyAscii(ledCfg.watch, prefs.getString("led_watch").c_str(), sizeof(ledCfg.watch));
  PREF("bl", getUChar, backlightPct);
  PREF("navbtn", getBool, navButtons);
  PREF("ntf_on", getBool, notifyCfg.on);
  if (prefs.isKey("ntf_topic")) copyAscii(notifyCfg.topic, prefs.getString("ntf_topic").c_str(), sizeof(notifyCfg.topic));
  if (prefs.isKey("ntf_srv")) copyAscii(notifyCfg.server, prefs.getString("ntf_srv").c_str(), sizeof(notifyCfg.server));
  PREF("ntf_range", getFloat, notifyCfg.range);
  PREF("ntf_cats", getUChar, notifyCfg.cats);
  PREF("ntf_low", getInt, notifyCfg.lowAltFt);
  PREF("ntf_cool", getUChar, notifyCfg.coolH);
  PREF("ntf_quiet", getBool, notifyCfg.quietNight);
  PREF("rot_on", getBool, scr.rotate);
  PREF("rot_sec", getUShort, scr.rotateSec);
  PREF("clk_show", getUChar, scr.clockShow);
  PREF("clk_style", getUChar, scr.clockStyle);
  PREF("clk_24", getBool, scr.clock24);
  PREF("clk_night", getBool, scr.clockAtNight);
  PREF("mtr_show", getUChar, scr.metarShow);
  if (prefs.isKey("mtr_icao")) copyAscii(scr.metarIcao, prefs.getString("mtr_icao").c_str(), sizeof(scr.metarIcao));
  PREF("idle_rng", getFloat, scr.idleRange);
  PREF("emg_on", getBool, emergencyBanner);
  PREF("heli_only", getBool, heliOnly);
  PREF("mirror", getBool, screenMirror);
  PREF("col_start", getUInt, colStart);
  // Notification kinds added in 1.5 are on by default for existing setups too
  if (!prefs.isKey("ntf_v2")) { notifyCfg.cats |= NT_EMERG | NT_NEWTYPE; prefs.putBool("ntf_v2", true); prefs.putUChar("ntf_cats", notifyCfg.cats); }
  PREF("theme", getUChar, themeIdx);
  if (themeIdx >= NUM_THEMES) themeIdx = 0;
  PREF("night_red", getBool, nightRedTheme);
  th = &THEMES[themeIdx];
  PREF("night_mode", getUChar, nightCfg.mode);
  PREF("night_from", getUShort, nightCfg.fromMin);
  PREF("night_to", getUShort, nightCfg.toMin);
  PREF("night_lvl", getUChar, nightCfg.level);
  PREF("night_led", getBool, nightCfg.ledOff);
  #undef PREF
  ledInit();

  // Backlight on GPIO21 through PWM (TFT_eSPI only switches it on)
  ledcSetup(BL_CH, 5000, 8);
  ledcAttachPin(TFT_BL, BL_CH);
  setBacklight(constrain(backlightPct, (uint8_t)10, (uint8_t)100));

  // Touch calibration: load it, or run it on first boot / when the screen is held at power-on
  bool haveCal = prefs.isKey("tcal") && prefs.getBytesLength("tcal") == sizeof(touchCal);
  if (haveCal) prefs.getBytes("tcal", touchCal, sizeof(touchCal));
  if (!haveCal || ts.touched()) runTouchCalibration();

  connectWiFiAtBoot();
  configTime(0, 0, "pool.ntp.org", "time.google.com");
  setenv("TZ", TIMEZONE, 1); tzset();

  nearest.valid = false;

  // Internal file system (the 128 KB "spiffs" partition) for the type collection
  fsOk = LittleFS.begin(true);
  Serial.printf("LittleFS: %s\n", fsOk ? "mounted" : "failed");
  if (fsOk) collectionLoad();

  initWebServer();
  setupArduinoOta();

  // Launch background fetcher on Core 0 (Core 1 = Arduino loop)
  xTaskCreatePinnedToCore(
    dataFetcherTask,   // Function
    "DataFetch",       // Name
    20480,             // Stack (20 KB — JSON parsing + TLS)
    NULL,              // Parameter
    1,                 // Priority
    &fetchTaskHandle,  // Handle
    0                  // Core 0 — leaves Core 1 free for rendering
  );
}

// Page rotation and the "no aircraft nearby" screens (clock / METAR)
void autoScreens() {
  static bool idle = false; static uint8_t beforeIdle = S_NEAREST;
  static uint32_t quietSince = 0, lastSwitch = 0;
  uint32_t now = millis();

  // Night: while the screen is dimmed, only the clock; a tap (wake) brings back the page
  static bool nightClock = false; static uint8_t beforeNight = S_NEAREST;
  if (scr.clockAtNight && nightDimmed()) {
    if (!nightClock) { nightClock = true; beforeNight = screen == S_CLOCK ? S_NEAREST : screen; }
    if (screen != S_CLOCK) screen = S_CLOCK;
    return;
  }
  if (nightClock) {
    nightClock = false; lastSwitch = now;
    if (screen == S_CLOCK) screen = beforeNight;
    return;
  }

  if (screen == SCREEN_SETTINGS) return;
  bool userActive = lastTouchMs && now - lastTouchMs < 60000UL;
  bool idleClock = scr.clockShow == SHOW_IDLE, idleMetar = scr.metarShow == SHOW_IDLE;

  bool any; double nk;
  if (dataMutex) xSemaphoreTake(dataMutex, portMAX_DELAY);
  any = nearest.valid; nk = nearest.distanceKm;
  if (dataMutex) xSemaphoreGive(dataMutex);
  bool quiet = lastDataMs && (!any || nk > scr.idleRange * KM_PER_UNIT);
  if (!quiet || !(idleClock || idleMetar)) quietSince = 0; else if (!quietSince) quietSince = now;

  if (idle) {
    if (!quiet || userActive || !(idleClock || idleMetar)) {
      idle = false; lastSwitch = now;
      if (!userActive && (screen == S_CLOCK || screen == S_METAR)) screen = beforeIdle;
      return;
    }
    if (idleClock && idleMetar && now - lastSwitch >= (scr.rotateSec > 20 ? scr.rotateSec : 20) * 1000UL) {
      screen = screen == S_CLOCK ? S_METAR : S_CLOCK; lastSwitch = now;
    }
    return;
  }
  // A page that was switched off (or an idle-only page after a tap): back to the first page
  if (!pageEnabled(screen) && !userActive) { screen = S_NEAREST; lastSwitch = now; }

  if (quietSince && now - quietSince > 30000UL && !userActive) {
    idle = true; beforeIdle = screen; lastSwitch = now;
    screen = idleClock ? S_CLOCK : S_METAR;
    return;
  }
  if (scr.rotate && !userActive && now - lastSwitch >= (uint32_t)scr.rotateSec * 1000UL) {
    screen = nextPage(screen, 1); lastSwitch = now;
  }
}

void loop() {
  // WiFi drops are handled by the ESP32 auto-reconnect; the fetch task
  // simply skips polls while offline and the settings screen shows the state.
  if (portalRequested) {
    portalRequested = false;
    runWifiPortal(true);
  }
  if (calibrateRequested) {
    calibrateRequested = false;
    runTouchCalibration();
  }

  server.handleClient();
  ArduinoOTA.handle();
  if (otaActive) { delay(5); return; }   // OTA screen owns the display

  // Crash-loop guard: a minute of normal running clears the crash counter
  static bool bootOk = false;
  if (!bootOk && millis() > 60000) { bootOk = true; if (prefs.getUChar("bootfail", 0)) prefs.putUChar("bootfail", 0); }

  checkTouch();
  autoScreens();
  themeUpdate();
  nightUpdate();
  render();
  updateBanner();
  updateNavOverlay();
  ledUpdate();
  delay(33);
}
