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

enum AlertKind : uint8_t { ALERT_NONE, ALERT_LOWALT, ALERT_HELI, ALERT_INTERESTING, ALERT_MILITARY, ALERT_WATCH };
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


// Top 5 Tracker
const uint8_t MAX_TOP5 = 5;
Aircraft top5[MAX_TOP5];
uint8_t top5Count = 0;

// Blips for Radar (the MAX_BLIPS nearest aircraft)
struct Blip { double lat; double lon; float track; float speedKt; char callsign[10]; int altFt; double distKm; };
const uint8_t MAX_BLIPS = 20;
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
volatile bool portalRequested = false;
volatile bool calibrateRequested = false;   // from the settings menu or the web page
const uint8_t NUM_SCREENS = 4;              // screens cycled by tapping
const uint8_t SCREEN_SETTINGS = 4;          // settings menu, opened with the gear icon
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
uint8_t backlightPct = 100;

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
                char* err = nullptr, size_t errLen = 0) {
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

  int code = https.GET();
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

// Fetches aircraft data into caller-provided local buffers (thread-safe — no globals touched)
bool fetchAircraftTo(Aircraft* top5Out, uint8_t& top5CntOut,
                     Aircraft& nearOut, Blip* blipsOut, uint8_t& blipCntOut,
                     AlertInfo& alertOut) {
  // Snapshot config under mutex (Core 0 reads, Core 1 may write via web UI)
  double hLat, hLon; float sRadNm; LedConfig lc;
  if (configMutex) xSemaphoreTake(configMutex, portMAX_DELAY);
  hLat = homeLat; hLon = homeLon; sRadNm = searchRadiusNm; lc = ledCfg;
  if (configMutex) xSemaphoreGive(configMutex);

  WatchList wl; wl.parse(lc.watch);
  double alertKm = lc.alertRange * KM_PER_UNIT;
  alertOut = AlertInfo();

  // Keep only the fields we use — near a big airport the raw payload is large.
  // The sources use the same readsb format; the array is "ac" (some use "aircraft").
  JsonDocument filter;
  for (const char* key : {"ac", "aircraft"}) {
    JsonObject f = filter[key].add<JsonObject>();
    for (const char* fld : {"hex", "flight", "r", "t", "lat", "lon", "alt_baro", "alt_geom", "gs",
                            "track", "baro_rate", "geom_rate", "category", "dbFlags"})
      f[fld] = true;
  }

  // Try the last source that worked first, then the others
  JsonDocument doc;
  bool ok = false;
  int start = activeSource;
  for (int k = 0; k < NUM_SOURCES && !ok; k++) {
    int i = (start + k) % NUM_SOURCES;
    String url = sourceUrl(i, hLat, hLon, (int)ceilf(sRadNm));
    char err[72] = "";
    doc.clear();
    if (httpGetJson(url, doc, &filter, err, sizeof(err)) == HTTP_CODE_OK) {
      ok = true;
      activeSource = i;
      lastOkMs = millis();
    } else {
      snprintf(lastFetchErr, sizeof(lastFetchErr), "%s: %s", SOURCES[i].name, err);
      Serial.printf("Aircraft source %s failed: %s\n", SOURCES[i].name, err);
    }
  }
  if (!ok) { stats.requestsFail++; return false; }

  JsonArray acList = doc["ac"].is<JsonArray>() ? doc["ac"].as<JsonArray>() : doc["aircraft"].as<JsonArray>();

  uint16_t count = 0;
  blipCntOut = 0;
  top5CntOut = 0;

  for (JsonObject s : acList) {
    if (!s["lat"].is<double>() || !s["lon"].is<double>()) continue;

    // Skip aircraft on the ground (alt_baro == "ground") — only track those in flight
    if (s["alt_baro"].is<const char*>()) continue;

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

    // LED alerts: keep the highest-priority (then nearest) special aircraft in range
    if (lc.alerts && d <= alertKm) {
      AlertKind k = classifyAlert(lc, wl, cs, type, category, dbFlags, altFt);
      if (k != ALERT_NONE && (k > alertOut.kind || (k == alertOut.kind && d < alertOut.distKm))) {
        alertOut.kind = k; alertOut.distKm = d;
        strcpy(alertOut.callsign, cs[0] ? cs : "?");
        strcpy(alertOut.type, type);
      }
    }

    // Radar keeps the MAX_BLIPS nearest aircraft (replace the farthest when full)
    int slot = -1;
    if (blipCntOut < MAX_BLIPS) {
      slot = blipCntOut++;
    } else {
      double farthest = d;
      for (int i = 0; i < MAX_BLIPS; i++)
        if (blipsOut[i].distKm > farthest) { farthest = blipsOut[i].distKm; slot = i; }
    }
    if (slot >= 0) {
      Blip& b = blipsOut[slot];
      b.lat = lat; b.lon = lon; b.track = track; b.speedKt = gsKt;
      b.altFt = altFt; b.distKm = d;
      strcpy(b.callsign, cs);
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
  }

  stats.inView = count; if (count > stats.maxInView) stats.maxInView = count;
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
               "&current=temperature_2m,relative_humidity_2m,wind_speed_10m,wind_direction_10m,weather_code,pressure_msl";
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
  locNearest.valid = false;

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

      if (fetchAircraftTo(locTop5, locTop5Count, locNearest, locBlips, locBlipCount, locAlert)) {
        // Routes for the top 5 (cached, so usually no network call)
        for (int i = 0; i < locTop5Count; i++)
          locTop5[i].hasRoute = getRoute(locTop5[i]);
        if (locTop5Count > 0) { getAircraftModel(locTop5[0]); locNearest = locTop5[0]; }

        // Atomically publish results to the globals used by render()
        xSemaphoreTake(dataMutex, portMAX_DELAY);
        memcpy(blips, locBlips, sizeof(locBlips));
        blipCount = locBlipCount;
        memcpy(top5, locTop5, sizeof(locTop5));
        top5Count = locTop5Count;
        nearest = locNearest;
        currentAlert = locAlert;
        if (locAlert.kind != ALERT_NONE) { lastAlert = locAlert; lastAlertMs = millis(); }
        lastDataMs = millis();
        stats.lastUpdateMs = lastDataMs;
        xSemaphoreGive(dataMutex);

      }
    }

    // Wait for the next poll interval; a location change or button press cuts it short
    for (int t = 0; t < UPDATE_INTERVAL_MS / 250; t++) {
      if (refreshRequested || ipLocateRequested || ghCheckRequested || ghUpdateRequested) {
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
// UI — navy / cyan design system shared by all screens
// ===========================================================================
#define C_BG     0x08A4   // #0A1520 background
#define C_PANEL  0x08E5   // #0E1C2B card fill
#define C_EDGE   0x19CA   // #1F3B57 card border
#define C_CYAN   0x065F   // #00C8FF accent
#define C_TRACK  0x21C9   // #26384A bars / unlit icon parts
#define C_SOFT   0x8E1C   // #8FC3E6 secondary text
#define C_LABEL  0x9D98   // #9DB2C6 labels
#define C_BADGE  0x11CC   // #173B63 dark blue
#define C_BLUEB  0x131C   // #1060E0 bright blue
#define C_GREEN  0x2F0D   // #2EE06B aircraft / OK
#define C_SUN    0xFE60   // #FFCC00 weather sun
#define C_PILLOK 0x3F2B   // #3CE65A "connected" pill
#define C_GOLD   0xF5C8   // #F0B840 (kept for older code paths)

// The radar sprite is 8-bit (RGB332): colors keep only their top bits, so radar
// colors are picked to survive that conversion
#define C_GRID8  0x2310   // dim steel blue rings
#define C_RING8  0x0495   // teal outer ring / sweep tail
#define C_DIM8   0x024A   // dark teal sweep tail

// Currently selected aircraft (tap a row on the traffic page); falls back to nearest
char selectedCs[10] = "";
uint32_t selectedUntil = 0;
const uint32_t SELECT_MS = 120000;

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
    uint16_t cc = code >= 45 ? 0xBDF7 : TFT_WHITE;
    tft.fillCircle(cx - 7, cy + 5, 7, cc); tft.fillCircle(cx + 3, cy, 9, cc); tft.fillCircle(cx + 12, cy + 6, 6, cc);
    tft.fillRect(cx - 7, cy + 5, 19, 8, cc);
    if ((code >= 51 && code <= 67) || (code >= 80 && code <= 82))
      for (int k = -6; k <= 10; k += 8) tft.drawLine(cx + k, cy + 16, cx + k - 3, cy + 22, C_CYAN);
    if ((code >= 71 && code <= 77) || code == 85 || code == 86)
      for (int k = -6; k <= 10; k += 8) tft.fillCircle(cx + k, cy + 19, 2, TFT_WHITE);
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
    tft.setTextFont(2); tft.setTextDatum(TR_DATUM); tft.setTextColor(TFT_WHITE, C_BG);
    tft.setTextPadding(44);
    tft.drawString(t, 282, 5);
    tft.setTextPadding(0); tft.setTextDatum(TL_DATUM);
  }
  int bars = wifiBars();
  if (bars != lastBars) {
    lastBars = bars;
    tft.fillRect(287, 2, 32, 22, C_BG);
    iconWifi(303, 19, bars, screen == SCREEN_SETTINGS ? TFT_ORANGE : C_CYAN, C_TRACK, C_BG);
  }
}

// ===========================================================================
// 1. NEAREST AIRCRAFT
// ===========================================================================
// Selected aircraft if one was tapped recently and is still in the top 5, else nearest
const Aircraft* targetAircraft(bool* isSelected = nullptr) {
  if (isSelected) *isSelected = false;
  if (selectedCs[0] && stillBefore(selectedUntil)) {
    for (int i = 0; i < top5Count; i++)
      if (strcmp(top5[i].callsign, selectedCs) == 0) {
        if (isSelected) *isSelected = i != 0;
        return i == 0 ? &nearest : &top5[i];
      }
  }
  return nearest.valid ? &nearest : nullptr;
}

void screenTargetIntel(bool newPage) {
  bool sel = false;
  const Aircraft* ap = targetAircraft(&sel);
  static bool lastSel = false;
  if (sel != lastSel) { lastSel = sel; newPage = true; tft.fillScreen(C_BG); }
  drawHeader(sel ? "SELECTED AIRCRAFT" : "NEAREST AIRCRAFT", newPage);

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
    tft.drawString("No aircraft in range", 160, 128);
    tft.setTextDatum(TL_DATUM); tft.setTextFont(2);
    return;
  }
  const Aircraft& a = *ap;
  char buf[48];

  // --- Identity card ---
  card(4, 29, 312, 52);
  tft.setFreeFont(&FreeSansBold18pt7b); tft.setTextColor(TFT_WHITE, C_PANEL);
  tft.drawString(fitPx(a.callsign, 196), 12, 31);
  tft.setFreeFont(&FreeSans9pt7b); tft.setTextColor(C_SOFT, C_PANEL);
  tft.drawString(fitPx(a.airline[0] ? a.airline : (a.model[0] ? a.model : "Unknown operator"), 200), 12, 62);
  bool mil = a.dbFlags & 1, heli = strcmp(a.category, "A7") == 0;
  drawPlaneGlyph(240, 51, 15, -90, mil ? TFT_ORANGE : C_CYAN);
  tft.setTextFont(2); tft.setTextColor(C_SOFT, C_PANEL); tft.setTextDatum(TR_DATUM);
  tft.drawString(mil ? "MIL" : heli ? "HELI" : (a.type[0] ? a.type : ""), 306, 58);
  tft.setTextDatum(TL_DATUM);

  // --- Route card: the line doubles as a progress bar, the plane sits where it is ---
  card(4, 85, 312, 44);
  if (a.hasRoute) {
    tft.setFreeFont(&FreeSansBold12pt7b); tft.setTextColor(TFT_WHITE, C_PANEL);
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
    drawPlaneGlyph(px, ly, 8, 0, C_CYAN);
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
  tft.setFreeFont(&FreeSansBold12pt7b); tft.setTextColor(TFT_WHITE, C_PANEL);
  tft.drawString(altText(a.altFt, false), 12, R1 + 18);
  snprintf(buf, sizeof(buf), "%d KT", (int)lround(a.gsKt));
  tft.drawString(buf, 116, R1 + 18);
  snprintf(buf, sizeof(buf), "%03d", ((int)lround(a.trackDeg)) % 360);
  tft.drawString(buf, 220, R1 + 18);
  degree(220 + tft.textWidth(buf) + 4, R1 + 21, 3, TFT_WHITE);
  iconCompass(296, R1 + 26, a.trackDeg, C_CYAN);

  // --- Distance / position / climb ---
  const int R2 = 181, H2 = 55;
  card(4, R2, 100, H2); card(108, R2, 134, H2); card(246, R2, 70, H2);
  label("DISTANCE", 12, R2 + 6);
  label("POSITION", 116, R2 + 6);
  label(a.vrateFpm >= 300 ? "CLIMB" : a.vrateFpm <= -300 ? "DESCENT" : "LEVEL", 254, R2 + 6);
  tft.setFreeFont(&FreeSansBold12pt7b); tft.setTextColor(TFT_WHITE, C_PANEL);
  snprintf(buf, sizeof(buf), "%.1f %s", toDisplayDist(a.distanceKm), DIST_UNIT);
  tft.drawString(fitPx(buf, 86), 12, R2 + 22);
  tft.setTextFont(2); tft.setTextColor(TFT_WHITE, C_PANEL);
  for (int k = 0; k < 2; k++) {
    double v = k == 0 ? a.lat : a.lon;
    snprintf(buf, sizeof(buf), "%.2f", fabs(v));
    int y = R2 + 18 + k * 18;
    tft.drawString(buf, 116, y);
    int w = tft.textWidth(buf);
    degree(116 + w + 3, y + 3, 2, TFT_WHITE);
    tft.drawString(k == 0 ? (v >= 0 ? "N" : "S") : (v >= 0 ? "E" : "W"), 116 + w + 8, y);
  }
  bool up = a.vrateFpm >= 300, down = a.vrateFpm <= -300;
  drawPlaneGlyph(274, R2 + 32, 11, up ? -35 : down ? 35 : 0, up ? C_GREEN : down ? TFT_ORANGE : C_SOFT);
  if (up || down) {
    arrowUpDown(304, R2 + 26, 4, up, up ? C_GREEN : TFT_ORANGE);
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
  drawHeader("TRAFFIC NEAR YOU", newPage);

  const Aircraft* ap = targetAircraft();
  static uint32_t drawnData = 0;
  static const Aircraft* drawnAp = nullptr;
  if (!newPage && drawnData == lastDataMs && drawnAp == ap) return;
  drawnData = lastDataMs; drawnAp = ap;

  tft.setTextPadding(0); tft.setTextDatum(TL_DATUM);
  tft.fillRect(0, 26, 320, 214, C_BG);

  if (top5Count == 0) {
    card(10, 82, 300, 80);
    tft.setFreeFont(&FreeSans9pt7b); tft.setTextColor(TFT_WHITE, C_PANEL); tft.setTextDatum(MC_DATUM);
    tft.drawString("No aircraft in range", 160, 112);
    tft.setTextFont(1); tft.setTextColor(C_SOFT, C_PANEL);
    tft.drawString("Traffic will appear here when detected", 160, 138);
    tft.setTextDatum(TL_DATUM); tft.setTextFont(2);
    return;
  }

  char buf[24];
  for (int i = 0; i < top5Count && i < MAX_TOP5; i++) {
    const Aircraft& a = top5[i];
    int y = TR_Y + i * TR_STEP;
    bool isTarget = ap && strcmp(ap->callsign, a.callsign) == 0;
    card(4, y, 312, TR_H);
    if (isTarget) { tft.drawRoundRect(4, y, 312, TR_H, 6, C_CYAN); tft.drawRoundRect(5, y + 1, 310, TR_H - 2, 5, C_CYAN); }
    bool mil = a.dbFlags & 1;
    drawPlaneGlyph(22, y + 19, 9, -90, mil ? TFT_ORANGE : isTarget ? C_CYAN : C_GREEN);

    tft.setFreeFont(&FreeSansBold9pt7b); tft.setTextColor(TFT_WHITE, C_PANEL);
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

void drawRangeButtons(float current) {
  for (int k = 0; k < 4; k++) {
    int y = RB_Y + k * RB_STEP;
    bool on = lroundf(current) == RANGE_PRESETS[k];
    tft.fillRoundRect(RB_X, y, RB_W, RB_H, 6, on ? C_BADGE : C_PANEL);
    tft.drawRoundRect(RB_X, y, RB_W, RB_H, 6, on ? C_CYAN : C_EDGE);
    if (on) tft.drawRoundRect(RB_X + 1, y + 1, RB_W - 2, RB_H - 2, 5, C_CYAN);
    tft.setTextDatum(MC_DATUM);
    tft.setTextFont(2); tft.setTextColor(TFT_WHITE, on ? C_BADGE : C_PANEL);
    tft.drawString(String(RANGE_PRESETS[k]), RB_X + RB_W / 2, y + 17);
    tft.setTextFont(1); tft.setTextColor(C_SOFT, on ? C_BADGE : C_PANEL);
    tft.drawString(DIST_UNIT, RB_X + RB_W / 2, y + 33);
    tft.setTextDatum(TL_DATUM);
  }
}

void screenRadar(bool newPage) {
  drawHeader("RADAR PPI", newPage);

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
    tft.fillRoundRect(SCOPE_X - 2, SCOPE_Y - 2, SCOPE + 4, SCOPE + 4, 8, TFT_BLACK);   // scope bezel
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
    tft.setFreeFont(&FreeSansBold9pt7b); tft.setTextColor(TFT_WHITE, C_PANEL);
    tft.drawString(String((int)lroundf(rRange)) + " " + DIST_UNIT, 243, 47);
  }
  if (stats.inView != drawnTracks) {
    drawnTracks = stats.inView;
    tft.fillRect(242, 83, 70, 18, C_PANEL);
    tft.setFreeFont(&FreeSansBold9pt7b); tft.setTextColor(TFT_WHITE, C_PANEL);
    tft.drawString(String(stats.inView), 243, 83);
  }

  // Scope (sprite, every frame)
  const int cx = SCOPE / 2, cy = SCOPE / 2, R = 80;
  radarSpr.fillSprite(TFT_BLACK);
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
  radarSpr.setTextFont(1); radarSpr.setTextColor(TFT_WHITE, TFT_BLACK); radarSpr.setTextDatum(MC_DATUM);
  radarSpr.drawString("N", cx, cy - R - 6); radarSpr.drawString("S", cx, cy + R + 6);
  radarSpr.drawString("W", cx - R - 6, cy); radarSpr.drawString("E", cx + R + 6, cy);
  radarSpr.setTextDatum(TL_DATUM);
  radarSpr.fillCircle(cx, cy, 2, C_CYAN);

  float elapsed = (millis() - lastDataMs) / 1000.0f;
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
    bool isTarget = ap && strcmp(ap->callsign, blips[i].callsign) == 0;
    if (isTarget) {
      radarSpr.drawCircle(bx, by, 11, C_CYAN);
      drawPlaneGlyph(radarSpr, bx, by, 8, blips[i].track - 90, C_CYAN);
      radarSpr.setTextColor(C_CYAN, TFT_BLACK);
      int lx = bx + 13; if (lx + 50 > SCOPE) lx = bx - 63;
      radarSpr.drawString(blips[i].callsign, lx, by - 4);
    } else {
      drawPlaneGlyph(radarSpr, bx, by, 6, blips[i].track - 90, C_GREEN);
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
    tft.drawString("Wind", 186, 40); tft.drawString("Humidity", 186, 64); tft.drawString("Pressure", 186, 88);
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
      tft.setFreeFont(&FreeSansBold18pt7b); tft.setTextColor(TFT_WHITE, C_PANEL);
      snprintf(buf, sizeof(buf), "%d", (int)lroundf(weather.temp));
      tft.drawString(buf, 66, 50);
      int w = tft.textWidth(buf);
      degree(66 + w + 5, 55, 4, TFT_WHITE);
      tft.drawString(TEMP_UNIT, 66 + w + 12, 50);
      tft.setTextFont(2); tft.setTextColor(C_SOFT, C_PANEL);
      tft.drawString(fitPx(wmoText(weather.code), 108), 66, 92);

      tft.setTextColor(TFT_WHITE, C_PANEL); tft.setTextDatum(TR_DATUM);
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
  tft.setFreeFont(&FreeSansBold12pt7b); tft.setTextColor(TFT_WHITE, C_PANEL);
  tft.drawString(buf, 42, 141);

  bool online = WiFi.status() == WL_CONNECTED;
  tft.fillRect(168, 140, 144, 28, C_PANEL);
  tft.fillRect(240, 126, 72, 14, C_PANEL);
  iconWifi(180, 163, wifiBars(), online ? C_CYAN : TFT_ORANGE, C_TRACK, C_PANEL);
  if (online) { snprintf(buf, sizeof(buf), "%d dBm", WiFi.RSSI()); }
  else snprintf(buf, sizeof(buf), "no link");
  tft.setFreeFont(&FreeSansBold9pt7b); tft.setTextColor(TFT_WHITE, C_PANEL);
  tft.drawString(buf, 200, 146);
  pill(244, 127, 66, 12, online ? "CONNECTED" : "OFFLINE", online ? C_PILLOK : TFT_RED, online ? TFT_BLACK : TFT_WHITE);

  // Memory: free / total heap, bar = used share; largest free block (TLS needs ~45 KB)
  uint32_t freeH = ESP.getFreeHeap(), totalH = ESP.getHeapSize(), blk = ESP.getMaxAllocHeap();
  tft.fillRect(34, 192, 72, 40, C_PANEL);
  tft.setTextFont(2); tft.setTextColor(TFT_WHITE, C_PANEL);
  snprintf(buf, sizeof(buf), "%u/%uK", (unsigned)(freeH / 1024), (unsigned)(totalH / 1024));
  tft.drawString(buf, 36, 194);
  tft.setTextFont(1); tft.setTextColor(blk < 45000 ? TFT_ORANGE : C_SOFT, C_PANEL);
  snprintf(buf, sizeof(buf), "blk %uK", (unsigned)(blk / 1024));
  tft.drawString(buf, 36, 212);
  int used = totalH ? (int)(88L * (totalH - freeH) / totalH) : 0;
  tft.fillRoundRect(12, 224, 88, 5, 2, C_TRACK);
  tft.fillRoundRect(12, 224, max(4, used), 5, 2, blk < 45000 ? TFT_ORANGE : C_GREEN);

  // API status: online if data arrived within a minute
  uint32_t ago = lastOkMs ? (millis() - lastOkMs) / 1000 : 0;
  const char* st; uint16_t sc, sf = TFT_BLACK;
  if (!online) { st = "Offline"; sc = TFT_RED; sf = TFT_WHITE; }
  else if (lastOkMs && ago < 60) { st = "Online"; sc = C_PILLOK; }
  else if (lastOkMs) { st = "Stale"; sc = TFT_ORANGE; }
  else { st = "Waiting"; sc = C_TRACK; sf = TFT_WHITE; }
  tft.fillRect(146, 198, 64, 36, C_PANEL);
  pill(148, 202, 58, 18, st, sc, sf);
  if (lastFetchErr[0] && !(lastOkMs && ago < 60)) {
    tft.setTextFont(1); tft.setTextColor(TFT_ORANGE, C_PANEL);
    tft.drawString(fitPx(lastFetchErr, 90), 118, 225);
  }

  tft.fillRect(248, 198, 64, 30, C_PANEL);
  tft.setFreeFont(&FreeSansBold9pt7b); tft.setTextColor(TFT_WHITE, C_PANEL);
  if (!lastOkMs) snprintf(buf, sizeof(buf), "--");
  else if (ago < 120) snprintf(buf, sizeof(buf), "%lus ago", (unsigned long)ago);
  else if (ago < 7200) snprintf(buf, sizeof(buf), "%lum ago", (unsigned long)(ago / 60));
  else snprintf(buf, sizeof(buf), "%luh ago", (unsigned long)(ago / 3600));
  tft.drawString(fitPx(buf, 64), 248, 203);
  tft.setTextFont(2);
}

// ===========================================================================
// 5. SETTINGS
// ===========================================================================
struct MenuBtn { int x, y, w, h; };
const int MENU_N = 7;
const MenuBtn MENU_BTNS[MENU_N] = {
  {4, 30, 154, 52},  {162, 30, 154, 52},
  {4, 86, 154, 52},  {162, 86, 154, 52},
  {4, 142, 154, 52}, {162, 142, 154, 52},
  {4, 198, 312, 38},
};
enum { MB_WIFI, MB_LOCATE, MB_CALIBRATE, MB_BRIGHT, MB_LED, MB_NAV, MB_BACK };

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
  tft.setFreeFont(&FreeSansBold9pt7b); tft.setTextColor(TFT_WHITE, C_PANEL);
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
  uint32_t key = (wifiArmed ? 1 : 0) | (ledAlerts ? 2 : 0) | (navButtons ? 4 : 0) | (online ? 8 : 0) |
                 ((uint32_t)backlightPct << 8) | ((uint32_t)(name[0] + name[1] * 3) << 16);
  if (!newPage && key == lastKey && strcmp(lastStatus, statusMsg) == 0) return;
  lastKey = key; strcpy(lastStatus, statusMsg);

  tft.setTextPadding(0); tft.setTextDatum(TL_DATUM);
  if (newPage) tft.fillRect(0, 26, 320, 214, C_BG);
  char buf[40];
  String wifiSub = wifiArmed ? String("Tap again") : online ? WiFi.localIP().toString() : String("Not connected");
  menuCard(MB_WIFI, "WiFi", wifiSub.c_str(), wifiArmed ? TFT_ORANGE : C_EDGE);
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
  int ax = 132, ay = bb.y + bb.h / 2;
  tft.fillTriangle(ax - 8, ay, ax - 1, ay - 6, ax - 1, ay + 6, TFT_WHITE);
  tft.fillRect(ax - 2, ay - 1, 12, 3, TFT_WHITE);
  tft.setFreeFont(&FreeSansBold9pt7b); tft.setTextColor(TFT_WHITE, C_PANEL);
  tft.drawString("Back", 152, bb.y + 10);
  tft.setTextFont(2);
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
    case 0: screenTargetIntel(newPage); break;
    case 1: screenTop5(newPage);        break;
    case 2: screenRadar(newPage);       break;
    case 3: screenWeatherSystem(newPage); break;
    case 4: screenSettings(newPage);    break;
  }
  if (dataMutex) xSemaphoreGive(dataMutex);
}

// ---------------------------------------------------------------------------
// Back RGB LED
// ---------------------------------------------------------------------------
struct RGB { uint8_t r, g, b; };

RGB alertColor(AlertKind k) {
  switch (k) {
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
void setBacklight(uint8_t pct) {
  backlightPct = pct;
  ledcWrite(BL_CH, (uint32_t)pct * 255 / 100);
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
    case MB_BACK:
      screen = screenBeforeSettings;
      break;
  }
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
void checkTouch() {
  static bool wasDown = false;
  int rx, ry;
  bool down = readTouchRaw(rx, ry);
  if (!down) { if (!ts.touched()) wasDown = false; return; }
  if (wasDown) return;                       // still the same press
  wasDown = true;
  if (millis() - lastTouchMs < 150) return;  // bounce
  lastTouchMs = millis();

  int sx, sy;
  mapTouch(rx, ry, sx, sy);
  Serial.printf("Touch: raw(%d,%d) -> screen(%d,%d)\n", rx, ry, sx, sy);

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
    if (sx < NAV_W + 20)        { navShownUntil = millis() + NAV_TIMEOUT_MS; screen = (screen + NUM_SCREENS - 1) % NUM_SCREENS; return; }
    if (sx >= 320 - NAV_W - 20) { navShownUntil = millis() + NAV_TIMEOUT_MS; screen = (screen + 1) % NUM_SCREENS; return; }
  }
  if (navButtons) navShownUntil = millis() + NAV_TIMEOUT_MS;

  // Radar screen: range preset buttons on the left
  if (screen == 2 && sx < RB_X + RB_W + 6 && sy >= RB_Y - 3) {
    int k = constrain((sy - RB_Y + 3) / RB_STEP, 0, 3);
    if (configMutex) xSemaphoreTake(configMutex, portMAX_DELAY);
    radarRange = RANGE_PRESETS[k];
    prefs.putFloat("range_u", radarRange);
    if (configMutex) xSemaphoreGive(configMutex);
    return;
  }

  // Traffic screen: tap a row to show that aircraft on the first page
  // (the header strip and empty space below the rows still flip to the next page)
  if (screen == 1 && sy >= TR_Y) {
    int row = (sy - TR_Y) / TR_STEP;
    if (row < top5Count && sy - TR_Y - row * TR_STEP <= TR_H) {
      if (dataMutex) xSemaphoreTake(dataMutex, portMAX_DELAY);
      strcpy(selectedCs, top5[row].callsign);
      if (dataMutex) xSemaphoreGive(dataMutex);
      selectedUntil = millis() + SELECT_MS;
      screen = 0;
      return;
    }
  }

  // Navigation arrows mode: other taps only show the arrows; only the arrows switch pages
  if (navButtons) return;

  // Anywhere else: next screen
  screen = (screen + 1) % NUM_SCREENS;
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
  <h2>Firmware</h2>
  <div style="font-size:0.9em;color:#aaa">Version <b>)rawliteral" + String(FW_VERSION) + R"rawliteral(</b>, build <b>)rawliteral" + String(BUILD_ENV) + R"rawliteral(</b>. Upload <code>firmware.bin</code> or <code>*-merged.bin</code> from the <b>)rawliteral" + String(BUILD_ENV) + R"rawliteral(</b> build on GitHub.</div>
  <input type="file" id="fw" accept=".bin" style="margin-top:12px">
  <progress id="fwp" max="100" value="0" style="width:100%;margin-top:10px;display:none"></progress>
  <div id="gh" style="font-size:0.9em;color:#aaa;margin-top:12px">Checking GitHub...</div>
  <button type="button" id="ghi" style="display:none" onclick="ghInstall()">Install from GitHub</button>
  <button type="button" class="opt" style="text-align:center" onclick="ghCheck()">Check for updates</button>
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
    d["available"] = ghLatest[0] && strcmp(ghLatest, FW_VERSION) != 0;
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

  checkTouch();
  render();
  updateNavOverlay();
  ledUpdate();
  delay(33);
}
