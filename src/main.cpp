/*
 * CYD28 (ESP32) Plane Spotter - Direct TFT Rendering
 * --------------------------------------------------------------------------
 * Data sources (no API keys needed):
 *   - airplanes.live  : live ADS-B aircraft around a point
 *   - adsbdb.com      : callsign -> airline + route (origin/destination)
 *   - Open-Meteo      : current weather
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
#include <ArduinoOTA.h>
#include <ESPmDNS.h>
#include <cydqr.h>
#include <TJpg_Decoder.h>
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
  static const float  RANGE_STEP = 5, RANGE_MIN = 5, RANGE_MAX = 150;
  static const float  DEFAULT_ALERT_RANGE = 3;
#else
  #define DIST_UNIT "km"
  #define TEMP_UNIT "C"
  #define WIND_UNIT "km/h"
  static const double KM_PER_UNIT = 1.0;
  static const float  RANGE_STEP = 10, RANGE_MIN = 10, RANGE_MAX = 250;
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
  char hex[7];                          // ICAO 24-bit address, for model/photo lookups
  char flightIata[9];                   // e.g. "UA1288"
  char depCity[20]; char arrCity[20];
  double depLat, depLon, arrLat, arrLon; bool hasCoords;
  char model[32];                       // e.g. "Boeing 737-7V3" (nearest aircraft only)
};
Aircraft nearest;

// Photo of the nearest aircraft (planespotters.net thumbnail), protected by dataMutex
uint8_t* photoJpg = nullptr; size_t photoLen = 0;
char photoCredit[28] = "";
uint32_t photoVersion = 0;              // bumps when the photo changes, triggers a redraw

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
volatile bool newDataReady = false;

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
  float temp; float wind; int humidity; bool valid = false;
} weather;

struct Stats {
  uint32_t requestsOk = 0; uint32_t requestsFail = 0;
  uint16_t inView = 0; uint16_t maxInView = 0;
  double closestEver = 1e9; uint32_t lastUpdateMs = 0;
} stats;

uint8_t screen = 0;
uint8_t lastScreen = 255;
uint32_t lastScreenSwap = 0;
uint32_t lastTouchMs = 0;
uint32_t wifiConfirmUntil = 0;   // "WiFi setup" needs a second tap within 3 s
volatile bool portalRequested = false;
volatile bool calibrateRequested = false;   // from the settings menu or the web page
const uint8_t NUM_SCREENS = 4;              // screens cycled by tapping
const uint8_t SCREEN_SETTINGS = 4;          // settings menu, opened with the gear icon
uint8_t screenBeforeSettings = 0;

// Touch calibration: screen = affine(raw). Default matches a typical CYD panel.
float touchCal[6] = {0.0f, -0.086486f, 337.30f,    // sx = a*rx + b*ry + c
                     -0.064865f, 0.0f, 252.97f};   // sy = d*rx + e*ry + f

// Screen backlight (GPIO21) via LEDC channel 0
#define BL_CH 0
uint8_t backlightPct = 100;
const uint32_t SCREEN_SWAP_MS = 10000;

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
      Serial.printf("HTTP error %d (%s): %s\n", code, https.errorToString(code).c_str(), url.c_str());
      if (err) snprintf(err, errLen, "%s", https.errorToString(code).c_str());
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

// GET a URL into a newly allocated buffer (caller frees). Returns length, 0 on failure.
size_t httpGetBytes(const String& url, uint8_t** out, size_t maxLen) {
  *out = nullptr;
  WiFiClientSecure client; client.setInsecure();
  HTTPClient http;
  http.useHTTP10(true);
  http.setTimeout(15000);
  http.setUserAgent("CYD28PlaneSpotter/1.2 (+https://github.com/argonfatl/CYD28PlaneSpotter)");
  if (!http.begin(client, url)) return 0;
  int code = http.GET();
  int size = http.getSize();
  if (code != HTTP_CODE_OK || size <= 0 || (size_t)size > maxLen) {
    Serial.printf("Photo download: HTTP %d, size %d\n", code, size);
    http.end(); return 0;
  }
  uint8_t* buf = (uint8_t*)malloc(size);
  if (!buf) { http.end(); return 0; }
  WiFiClient* st = http.getStreamPtr();
  size_t got = 0; uint32_t t0 = millis();
  while (got < (size_t)size && millis() - t0 < 15000) {
    int n = st->read(buf + got, size - got);
    if (n > 0) got += n; else delay(5);
  }
  http.end();
  if (got != (size_t)size) { free(buf); return 0; }
  *out = buf;
  return got;
}

// Fetch a photo of the aircraft from planespotters.net and publish it for the screen
void updatePhoto(const char* hex) {
  static char lastHex[7] = "";
  if (!hex[0] || strcmp(hex, lastHex) == 0) return;
  strcpy(lastHex, hex);

  uint8_t* jpg = nullptr; size_t len = 0; char credit[28] = "";
  JsonDocument filter;
  filter["photos"][0]["thumbnail"]["src"] = true;
  filter["photos"][0]["photographer"] = true;
  JsonDocument d;
  if (httpGetJson(String("https://api.planespotters.net/pub/photos/hex/") + hex, d, &filter) == HTTP_CODE_OK) {
    const char* src = d["photos"][0]["thumbnail"]["src"] | "";
    if (src[0]) {
      len = httpGetBytes(src, &jpg, 24 * 1024);
      const char* who = d["photos"][0]["photographer"] | "";
      if (who[0]) snprintf(credit, sizeof(credit), "(c) %s", who);
    }
  }
  xSemaphoreTake(dataMutex, portMAX_DELAY);
  free(photoJpg);
  photoJpg = jpg; photoLen = len;
  copyAscii(photoCredit, credit, sizeof(photoCredit));
  photoVersion++;
  xSemaphoreGive(dataMutex);
  Serial.printf("Photo %s: %u bytes %s\n", hex, (unsigned)len, credit);
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
char lastFetchErr[48] = "";         // last aircraft fetch error, for the status screen

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
    char err[40] = "";
    doc.clear();
    if (httpGetJson(url, doc, &filter, err, sizeof(err)) == HTTP_CODE_OK) {
      ok = true;
      activeSource = i;
      lastFetchErr[0] = '\0';
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
    if (nearOut.distanceKm < stats.closestEver)
      stats.closestEver = nearOut.distanceKm;
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
               "&current=temperature_2m,relative_humidity_2m,wind_speed_10m";
#if UNITS_IMPERIAL
  url += "&temperature_unit=fahrenheit&wind_speed_unit=mph";
#endif

  JsonDocument doc;
  if (httpGetJson(url, doc, nullptr) != HTTP_CODE_OK) return false;
  JsonObject c = doc["current"]; if (c.isNull()) return false;

  w.temp = c["temperature_2m"] | 0.0f;
  w.humidity = c["relative_humidity_2m"] | 0;
  w.wind = c["wind_speed_10m"] | 0.0f;
  w.valid = true;
  return true;
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
    if (otaActive) { vTaskDelay(pdMS_TO_TICKS(500)); continue; }
    refreshRequested = false;
    if (WiFi.status() == WL_CONNECTED) {
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
        newDataReady = true;
        xSemaphoreGive(dataMutex);

        // Photo of the nearest aircraft (only re-fetched when the target changes)
        if (locNearest.valid) updatePhoto(locNearest.hex);
      }
    }

    // Wait for the next poll interval; a location change or button press cuts it short
    for (int t = 0; t < UPDATE_INTERVAL_MS / 250; t++) {
      if (refreshRequested || ipLocateRequested) {
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
bool timeReady() { return time(nullptr) > 1700000000; }
void fmtClock(char* buf, size_t n, bool withSecs) {
  if (!timeReady()) { strncpy(buf, withSecs ? "--:--:--" : "--:--", n); return; }
  time_t t = time(nullptr); struct tm lt; localtime_r(&t, &lt);
  strftime(buf, n, withSecs ? "%H:%M:%S" : "%H:%M", &lt);
}

// Gear icon in the top-right corner: opens the settings menu
const int GEAR_X = 304, GEAR_Y = 12;
void drawGear(uint16_t color) {
  for (int i = 0; i < 8; i++) {
    float a = i * PI / 4;
    tft.fillCircle(GEAR_X + (int)lroundf(cosf(a) * 8), GEAR_Y + (int)lroundf(sinf(a) * 8), 2, color);
  }
  tft.fillCircle(GEAR_X, GEAR_Y, 7, color);
  tft.fillCircle(GEAR_X, GEAR_Y, 3, TFT_BLACK);
}

void drawHeader(const char* title, bool newPage) {
  tft.setTextFont(2);
  tft.setTextDatum(TL_DATUM);
  tft.setTextColor(TFT_LIGHTGREY, TFT_BLACK);

  if (newPage) {
    char left[30];
    if (screen < NUM_SCREENS) snprintf(left, sizeof(left), "%s [%d/%d]", title, screen + 1, NUM_SCREENS);
    else snprintf(left, sizeof(left), "%s", title);
    tft.setTextPadding(200);
    tft.drawString(left, 10, 5);
    tft.drawLine(0, 25, 320, 25, TFT_DARKGREY);
    drawGear(screen == SCREEN_SETTINGS ? TFT_ORANGE : TFT_LIGHTGREY);
  }

  char t[10]; fmtClock(t, sizeof(t), true);
  tft.setTextColor(TFT_LIGHTGREY, TFT_BLACK);
  tft.setTextPadding(62);
  tft.drawString(t, 222, 5);
  tft.setTextPadding(0);
}

void drawArrow(int cx, int cy, int r, double angleDeg, uint16_t color) {
  double a = deg2rad(angleDeg);
  int tx = cx + (int)(sin(a) * r), ty = cy - (int)(cos(a) * r);
  int bx = cx - (int)(sin(a) * r), by = cy + (int)(cos(a) * r);
  tft.drawLine(bx, by, tx, ty, color);
  double left = a + deg2rad(150), right = a - deg2rad(150);
  tft.drawLine(tx, ty, tx + (int)(sin(left) * (r/2)), ty - (int)(cos(left) * (r/2)), color);
  tft.drawLine(tx, ty, tx + (int)(sin(right)* (r/2)), ty - (int)(cos(right)* (r/2)), color);
}

// ---- Target Intel: flight card (callsign, route progress, photo) ----
#define C_BAND   0x2124   // #262626 top band
#define C_PANEL  0x18E3   // #1C1C1C cards
#define C_TRACK  0x39C7   // #3A3A3A progress track
#define C_GOLD   0xF5C8   // #F0B840 callsign / progress
#define C_BADGE  0x5ACB   // #5A5A5A grey badge
#define C_BLUEB  0x328D   // #34506E blue badge
#define C_LABEL  0x9CF3   // #9C9C9C labels

// Photo box (top right of the band)
const int PH_X = 216, PH_Y = 27, PH_W = 100, PH_H = 67;

bool jpgToTft(int16_t x, int16_t y, uint16_t w, uint16_t h, uint16_t* bitmap) {
  tft.pushImage(x, y, w, h, bitmap);   // clipped by the viewport set around the photo box
  return true;
}

// Text that fits in maxW pixels (font must be set), shortened with "."
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

// Small airplane silhouette, nose pointing along angleDeg (0 = right, + = nose down)
void drawPlaneGlyph(int cx, int cy, float sc, float angleDeg, uint16_t color) {
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
    tft.fillTriangle(p[0], p[1], p[2], p[3], p[4], p[5], color);
  }
}

// Rounded badge with centered text (font 2); returns its width
int drawBadge(int x, int y, const char* text, uint16_t bg) {
  tft.setTextFont(2);
  int w = tft.textWidth(text) + 10;
  tft.fillRoundRect(x, y, w, 20, 4, bg);
  tft.setTextColor(TFT_WHITE, bg);
  tft.setTextDatum(ML_DATUM);
  tft.drawString(text, x + 5, y + 10);
  tft.setTextDatum(TL_DATUM);
  return w;
}

void drawPhotoBox() {
  if (photoJpg && photoLen) {
    uint16_t w = 0, h = 0;
    TJpgDec.getJpgSize(&w, &h, photoJpg, photoLen);
    uint8_t scale = 1;
    while (scale < 8 && (w / scale > PH_W + 20 || h / scale > PH_H + 14)) scale *= 2;
    TJpgDec.setJpgScale(scale);
    tft.setViewport(PH_X, PH_Y, PH_W, PH_H);    // clip to the box; coordinates become relative
    tft.fillScreen(C_BAND);
    TJpgDec.drawJpg((PH_W - (int)(w / scale)) / 2, (PH_H - (int)(h / scale)) / 2, photoJpg, photoLen);
    tft.resetViewport();
    if (photoCredit[0]) {
      tft.fillRect(PH_X, PH_Y + PH_H - 11, PH_W, 11, C_BLUEB);
      tft.setTextFont(1); tft.setTextColor(TFT_WHITE, C_BLUEB);
      tft.drawString(fitPx(photoCredit, PH_W - 4), PH_X + 2, PH_Y + PH_H - 10);
    }
  } else {
    tft.fillRoundRect(PH_X, PH_Y, PH_W, PH_H, 4, C_PANEL);
    drawPlaneGlyph(PH_X + PH_W / 2, PH_Y + PH_H / 2, 30, -15, C_TRACK);
  }
}

void screenTargetIntel(bool newPage) {
  drawHeader("TARGET INTEL", newPage);

  // Redraw the card only when the data or the photo changes (no flicker)
  static uint32_t drawnData = 0, drawnPhoto = 0;
  static bool drawnValid = false;
  if (!newPage && drawnData == lastDataMs && drawnPhoto == photoVersion && drawnValid == nearest.valid) return;
  drawnData = lastDataMs; drawnPhoto = photoVersion; drawnValid = nearest.valid;

  tft.setTextPadding(0);
  tft.setTextDatum(TL_DATUM);
  tft.fillRect(0, 26, 320, 214, TFT_BLACK);

  if (!nearest.valid) {
    tft.setTextFont(4);
    tft.setTextColor(TFT_RED, TFT_BLACK);
    tft.drawString("NO TARGET IN RANGE", 40, 100);
    return;
  }
  const Aircraft& a = nearest;
  char buf[48];

  // --- Top band: callsign + badges, airline, distance from you; photo on the right ---
  tft.fillRect(0, 26, 320, 70, C_BAND);
  tft.setTextFont(4); tft.setTextColor(C_GOLD, C_BAND);
  tft.drawString(a.callsign, 6, 30);
  int bx = 6 + tft.textWidth(a.callsign) + 6;
  if (a.flightIata[0] && strcmp(a.flightIata, a.callsign) != 0) {
    tft.setTextFont(2);
    if (bx + tft.textWidth(a.flightIata) + 10 <= PH_X - 4) bx += drawBadge(bx, 32, a.flightIata, C_BADGE) + 4;
  }
  const char* tag = (a.dbFlags & 1) ? "MIL" : strcmp(a.category, "A7") == 0 ? "HELI" : a.type;
  if (tag[0]) {
    tft.setTextFont(2);
    if (bx + tft.textWidth(tag) + 10 <= PH_X - 4)
      drawBadge(bx, 32, tag, (a.dbFlags & 1) ? 0x8000 : C_BLUEB);
  }
  tft.setTextFont(2); tft.setTextColor(TFT_WHITE, C_BAND);
  tft.drawString(fitPx(a.airline[0] ? a.airline : "Unknown operator", PH_X - 12), 6, 58);
  tft.setTextColor(C_LABEL, C_BAND);
  snprintf(buf, sizeof(buf), "%.1f %s %s of you  HDG %03.0f",
           toDisplayDist(a.distanceKm), DIST_UNIT, compass(a.bearingDeg), a.trackDeg);
  tft.drawString(fitPx(buf, PH_X - 12), 6, 76);
  drawPhotoBox();

  // --- Route card ---
  const int RX = 4, RY = 100, RW = 208, RH = 76;
  tft.fillRoundRect(RX, RY, RW, RH, 6, C_PANEL);
  if (a.hasRoute) {
    tft.setTextDatum(TC_DATUM);
    tft.setTextFont(4); tft.setTextColor(TFT_WHITE, C_PANEL);
    tft.drawString(a.dep, RX + 44, RY + 6);
    tft.drawString(a.arr, RX + RW - 44, RY + 6);
    tft.setTextFont(1); tft.setTextColor(C_LABEL, C_PANEL);
    String dc(a.depCity), ac(a.arrCity); dc.toUpperCase(); ac.toUpperCase();
    tft.drawString(fitPx(dc.c_str(), 84), RX + 44, RY + 34);
    tft.drawString(fitPx(ac.c_str(), 84), RX + RW - 44, RY + 34);
    tft.setTextDatum(TL_DATUM);

    // Plane icon: tilted by climb / descent
    float tilt = a.vrateFpm <= -300 ? 20 : a.vrateFpm >= 300 ? -20 : 0;
    tft.fillCircle(RX + RW / 2, RY + 18, 13, TFT_WHITE);
    drawPlaneGlyph(RX + RW / 2, RY + 18, 9, tilt, C_GOLD);

    // Progress along the great circle, time to go at current ground speed
    const int PX = RX + 10, PY = RY + 47, PW = RW - 20;
    tft.fillRoundRect(PX, PY, PW, 6, 3, C_TRACK);
    if (a.hasCoords) {
      double flown = haversineKm(a.depLat, a.depLon, a.lat, a.lon);
      double left  = haversineKm(a.lat, a.lon, a.arrLat, a.arrLon);
      float frac = (flown + left) > 1 ? (float)(flown / (flown + left)) : 0;
      tft.fillRoundRect(PX, PY, max(6, (int)(PW * frac)), 6, 3, C_GOLD);
      tft.setTextFont(2); tft.setTextColor(C_LABEL, C_PANEL);
      snprintf(buf, sizeof(buf), "%s %s to go", withCommas((int)lround(toDisplayDist(left))).c_str(), DIST_UNIT);
      tft.drawString(buf, PX, PY + 10);
      if (a.gsKt > 60) {
        int mins = (int)lround(left / (a.gsKt * 1.852) * 60.0);
        snprintf(buf, sizeof(buf), "~%d:%02d", mins / 60, mins % 60);
        tft.setTextDatum(TR_DATUM);
        tft.drawString(buf, PX + PW, PY + 10);
        tft.setTextDatum(TL_DATUM);
      }
    }
  } else {
    tft.setTextFont(2); tft.setTextColor(C_LABEL, C_PANEL);
    tft.drawString("Route unknown", RX + 12, RY + 12);
    drawPlaneGlyph(RX + RW / 2, RY + 48, 18, 0, C_TRACK);
  }

  // --- Altitude / speed card ---
  const int SX = 216, SW = 100;
  tft.fillRoundRect(SX, RY, SW, RH, 6, C_PANEL);
  tft.setTextFont(1); tft.setTextColor(C_LABEL, C_PANEL);
  tft.drawString("ALTITUDE FT", SX + 7, RY + 6);
  tft.drawString("SPEED KT", SX + 7, RY + 42);
  if (a.vrateFpm <= -300) tft.fillTriangle(SX + 86, RY + 5, SX + 94, RY + 5, SX + 90, RY + 12, TFT_ORANGE);
  else if (a.vrateFpm >= 300) tft.fillTriangle(SX + 86, RY + 12, SX + 94, RY + 12, SX + 90, RY + 5, TFT_GREEN);
  tft.setTextFont(4); tft.setTextColor(TFT_WHITE, C_PANEL);
  tft.drawString(withCommas(a.altFt), SX + 7, RY + 16);
  tft.drawString(String((int)lround(a.gsKt)), SX + 7, RY + 52);

  // --- Aircraft model / registration ---
  const int BY = 180, BH = 38;
  tft.fillRoundRect(RX, BY, RW, BH, 6, C_PANEL);
  tft.fillRoundRect(SX, BY, SW, BH, 6, C_PANEL);
  tft.setTextFont(1); tft.setTextColor(C_LABEL, C_PANEL);
  tft.drawString("AIRCRAFT", RX + 8, BY + 5);
  tft.drawString("REG.", SX + 7, BY + 5);
  tft.setTextFont(2); tft.setTextColor(TFT_WHITE, C_PANEL);
  tft.drawString(fitPx(a.model[0] ? a.model : (a.type[0] ? a.type : "Unknown"), RW - 16), RX + 8, BY + 16);
  tft.drawString(fitPx(a.reg[0] ? a.reg : "-", SW - 12), SX + 7, BY + 16);

  // Footer: vertical rate in words
  if (a.vrateFpm >= 300 || a.vrateFpm <= -300) {
    tft.setTextFont(1); tft.setTextColor(C_LABEL, TFT_BLACK);
    snprintf(buf, sizeof(buf), "%s %s fpm", a.vrateFpm > 0 ? "Climbing" : "Descending",
             withCommas(abs(a.vrateFpm)).c_str());
    tft.drawString(buf, RX + 4, 226);
  }
}

void screenTop5(bool newPage) {
  drawHeader("TOP 5 IN RANGE", newPage);

  if (top5Count == 0) {
    tft.setTextFont(4); tft.setTextColor(TFT_RED, TFT_BLACK);
    tft.drawString("NO TARGETS", 100, 100);
    return;
  }

  if (newPage) {
    tft.setTextFont(2); tft.setTextColor(TFT_CYAN, TFT_BLACK);
    tft.drawString("FLIGHT", 10, 40);
    tft.drawString("ROUTE", 100, 40);
    tft.drawString("ALTITUDE", 190, 40);
    tft.drawLine(10, 60, 310, 60, TFT_DARKGREY);
  }

  tft.setTextFont(2); tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setTextPadding(80);

  int y = 70;
  for (int i=0; i<5; i++) {
    if (i < top5Count) {
      tft.drawString(top5[i].callsign, 10, y);

      char rte[16];
      if (top5[i].hasRoute) snprintf(rte, sizeof(rte), "%s->%s", top5[i].dep, top5[i].arr);
      else strcpy(rte, "N/A");
      tft.drawString(rte, 100, y);

      char alt[32];
      tft.setTextPadding(120);
      snprintf(alt, sizeof(alt), "%d ft (FL%03d)", top5[i].altFt, (top5[i].altFt + 50) / 100);
      tft.drawString(alt, 190, y);
      tft.setTextPadding(80);

    } else {
      tft.drawString(" ", 10, y);
      tft.drawString(" ", 100, y);
      tft.setTextPadding(120);
      tft.drawString(" ", 190, y);
      tft.setTextPadding(80);
    }
    y += 30;
  }
  tft.setTextPadding(0);
}

// Range buttons on the radar page (drawn and hit-tested from the same constants)
const int BTN_Y = 215, BTN_H = 20, BTN_W = 30;
const int BTN_MINUS_X = 10, BTN_PLUS_X = 50;

void screenRadar(bool newPage) {
  drawHeader("RADAR PPI", newPage);

  // Snapshot config under mutex
  double hLat, hLon; float rRange;
  if (configMutex) xSemaphoreTake(configMutex, portMAX_DELAY);
  hLat = homeLat; hLon = homeLon; rRange = radarRange;
  if (configMutex) xSemaphoreGive(configMutex);
  double rMaxKm = rRange * KM_PER_UNIT;

  tft.setTextFont(2); tft.setTextColor(TFT_WHITE, TFT_BLACK); tft.setTextPadding(110);
  char buf[32]; snprintf(buf, sizeof(buf), "Contacts: %u", stats.inView);
  tft.drawString(buf, 5, 50);
  if(nearest.valid) {
    snprintf(buf, sizeof(buf), "TGT: %s", nearest.callsign);
    tft.drawString(buf, 5, 80);
    snprintf(buf, sizeof(buf), "%.1f %s", toDisplayDist(nearest.distanceKm), DIST_UNIT);
    tft.drawString(buf, 5, 110);
  } else {
    tft.drawString("No TGT", 5, 80);
    tft.drawString(" ", 5, 110);
  }
  tft.setTextPadding(0);

  const int cx = 100, cy = 100, R = 95;
  float elapsed = (millis() - lastDataMs) / 1000.0f;

  radarSpr.fillSprite(TFT_BLACK);
  radarSpr.drawCircle(cx, cy, R, TFT_DARKGREY);
  radarSpr.drawCircle(cx, cy, R*2/3, TFT_DARKGREY);
  radarSpr.drawCircle(cx, cy, R/3, TFT_DARKGREY);
  radarSpr.drawLine(cx-R, cy, cx+R, cy, TFT_DARKGREY);
  radarSpr.drawLine(cx, cy-R, cx, cy+R, TFT_DARKGREY);

  float sweepDeg = fmodf(millis() / 15.0f, 360.0f);
  double sw = deg2rad(sweepDeg);
  radarSpr.drawLine(cx, cy, cx + (int)(sin(sw)*R), cy - (int)(cos(sw)*R), TFT_GREEN);

  radarSpr.setTextFont(1);
  radarSpr.setTextColor(TFT_WHITE, TFT_BLACK);

  for (uint8_t i = 0; i < blipCount; i++) {
    double la = blips[i].lat, lo = blips[i].lon;
    if (blips[i].speedKt > 0 && elapsed > 0)
      projectLatLon(blips[i].lat, blips[i].lon, blips[i].track,
                    blips[i].speedKt * 0.514444f * elapsed, la, lo);

    double dist = haversineKm(hLat, hLon, la, lo);
    float fr = (float)(dist / rMaxKm);
    if (fr > 1) continue;

    int rr = (int)(fr * R);
    double brg = bearingDeg(hLat, hLon, la, lo);
    int bx = cx + (int)(sin(deg2rad(brg)) * rr);
    int by = cy - (int)(cos(deg2rad(brg)) * rr);

    float behind = fmodf(sweepDeg - (float)brg + 360.0f, 360.0f);
    if (behind < 30) radarSpr.fillCircle(bx, by, 3, TFT_YELLOW);
    else radarSpr.fillRect(bx, by, 2, 2, TFT_GREEN);

    // Callsign, flight level and speed in the smallest legible font
    if (blips[i].callsign[0]) {
      int lx = bx + 4;
      int ly = by - 4;
      if (lx + 54 > 200) lx = bx - 54;
      if (ly < 0) ly = 0;
      if (ly > 192) ly = 192;
      radarSpr.drawString(blips[i].callsign, lx, ly);

      char flBuf[16];
      snprintf(flBuf, sizeof(flBuf), "FL%03d", (blips[i].altFt + 50) / 100);
      int ly2 = ly + 8;
      if (ly2 > 192) ly2 = 192;
      radarSpr.drawString(flBuf, lx, ly2);

      char spBuf[10];
      snprintf(spBuf, sizeof(spBuf), "%ukn", (unsigned int)(blips[i].speedKt + 0.5f));
      int ly3 = ly + 16;
      if (ly3 > 192) ly3 = 192;
      radarSpr.drawString(spBuf, lx, ly3);
    }
  }
  radarSpr.pushSprite(115, 35);

  // --- Range indicator & -/+ buttons (bottom-left, side by side) ---
  tft.setTextFont(2);
  tft.setTextColor(TFT_CYAN, TFT_BLACK);
  tft.setTextPadding(80);
  char rng[16]; snprintf(rng, sizeof(rng), "RNG %d %s", (int)rRange, DIST_UNIT);
  tft.drawString(rng, 10, 193);

  tft.fillRoundRect(BTN_MINUS_X, BTN_Y, BTN_W, BTN_H, 4, TFT_DARKGREY);
  tft.fillRoundRect(BTN_PLUS_X,  BTN_Y, BTN_W, BTN_H, 4, TFT_DARKGREY);
  tft.setTextColor(TFT_WHITE, TFT_DARKGREY);
  tft.setTextPadding(0);
  tft.setTextDatum(MC_DATUM);
  tft.drawString("-", BTN_MINUS_X + BTN_W / 2, BTN_Y + BTN_H / 2);
  tft.drawString("+", BTN_PLUS_X + BTN_W / 2,  BTN_Y + BTN_H / 2);
  tft.setTextDatum(TL_DATUM);
}

// Unified Weather and System Screen
void screenWeatherSystem(bool newPage) {
  drawHeader("WEATHER & SYSTEM", newPage);

  if (newPage) {
    tft.drawLine(10, 115, 310, 115, TFT_DARKGREY);
  }

  // --- TOP HALF: WEATHER ---
  if (!weather.valid) {
    tft.setTextFont(4);
    tft.setTextColor(TFT_RED, TFT_BLACK);
    tft.drawString("NO WX DATA", 20, 50);
  } else {
    tft.setTextFont(4);
    tft.setTextColor(TFT_YELLOW, TFT_BLACK);
    tft.setTextPadding(130);
    char buf[32]; snprintf(buf, sizeof(buf), "%.1f %s", weather.temp, TEMP_UNIT);
    tft.drawString(buf, 20, 45);

    tft.setTextFont(2);
    tft.setTextColor(TFT_CYAN, TFT_BLACK);
    tft.setTextPadding(150);
    snprintf(buf, sizeof(buf), "Humidity: %d%%", weather.humidity);
    tft.drawString(buf, 160, 45);
    snprintf(buf, sizeof(buf), "Wind: %.0f %s", weather.wind, WIND_UNIT);
    tft.drawString(buf, 160, 75);
  }

  // --- BOTTOM HALF: SYSTEM ---
  tft.setTextFont(2);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setTextPadding(160);

  char bufSys[64];
  uint32_t up = millis() / 1000;
  snprintf(bufSys, sizeof(bufSys), "Uptime: %02lu:%02lu:%02lu",
           (unsigned long)(up / 3600), (unsigned long)((up % 3600) / 60), (unsigned long)(up % 60));
  tft.drawString(bufSys, 20, 130);

  snprintf(bufSys, sizeof(bufSys), "WiFi RSSI: %d dBm", WiFi.RSSI());
  tft.drawString(bufSys, 180, 130);

  snprintf(bufSys, sizeof(bufSys), "RAM Free: %d KB", ESP.getFreeHeap() / 1024);
  tft.drawString(bufSys, 20, 160);

  snprintf(bufSys, sizeof(bufSys), "Max seen: %u", stats.maxInView);
  tft.drawString(bufSys, 180, 160);

  tft.setTextPadding(300);
  snprintf(bufSys, sizeof(bufSys), "API Req: %lu OK / %lu FAIL",
           (unsigned long)stats.requestsOk, (unsigned long)stats.requestsFail);
  tft.drawString(bufSys, 20, 190);

  if (lastFetchErr[0]) {
    tft.setTextColor(TFT_ORANGE, TFT_BLACK);
    snprintf(bufSys, sizeof(bufSys), "Err %s", lastFetchErr);
  } else {
    tft.setTextColor(TFT_DARKGREY, TFT_BLACK);
    snprintf(bufSys, sizeof(bufSys), "Source: %s", SOURCES[activeSource].name);
  }
  tft.drawString(bufSys, 20, 212);

  tft.setTextPadding(0);
}

// Settings menu: status lines + 2x3 grid of big buttons
struct MenuBtn { int x, y, w, h; };
const MenuBtn MENU_BTNS[6] = {
  {6, 98, 150, 42},  {164, 98, 150, 42},
  {6, 146, 150, 42}, {164, 146, 150, 42},
  {6, 194, 150, 42}, {164, 194, 150, 42},
};
enum { MB_WIFI, MB_LOCATE, MB_CALIBRATE, MB_BRIGHT, MB_LED, MB_BACK };

void drawMenuButton(int i, const char* label, uint16_t bg) {
  const MenuBtn& b = MENU_BTNS[i];
  tft.fillRoundRect(b.x, b.y, b.w, b.h, 6, bg);
  tft.setTextFont(2); tft.setTextDatum(MC_DATUM); tft.setTextPadding(0);
  tft.setTextColor(TFT_WHITE, bg);
  tft.drawString(label, b.x + b.w / 2, b.y + b.h / 2);
  tft.setTextDatum(TL_DATUM);
}

int menuButtonAt(int sx, int sy) {
  for (int i = 0; i < 6; i++) {
    const MenuBtn& b = MENU_BTNS[i];
    if (sx >= b.x - 3 && sx < b.x + b.w + 3 && sy >= b.y - 3 && sy < b.y + b.h + 3) return i;
  }
  return -1;
}

void screenSettings(bool newPage) {
  drawHeader("SETTINGS", newPage);

  char name[48]; bool ledAlerts;
  if (configMutex) xSemaphoreTake(configMutex, portMAX_DELAY);
  strcpy(name, homeName); ledAlerts = ledCfg.alerts;
  if (configMutex) xSemaphoreGive(configMutex);

  char buf[64];
  tft.setTextFont(2); tft.setTextDatum(TL_DATUM); tft.setTextPadding(306);
  if (WiFi.status() == WL_CONNECTED) {
    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    snprintf(buf, sizeof(buf), "WiFi: %s (%d dBm)", WiFi.SSID().c_str(), WiFi.RSSI());
    tft.drawString(buf, 8, 30);
    tft.setTextColor(TFT_CYAN, TFT_BLACK);
    snprintf(buf, sizeof(buf), "%s.local  %s", DEVICE_HOSTNAME, WiFi.localIP().toString().c_str());
    tft.drawString(buf, 8, 46);
  } else {
    tft.setTextColor(TFT_RED, TFT_BLACK);
    tft.drawString("WiFi: not connected", 8, 30);
    tft.drawString(" ", 8, 46);
  }
  tft.setTextColor(TFT_GREEN, TFT_BLACK);
  snprintf(buf, sizeof(buf), "Location: %s", name);
  tft.drawString(buf, 8, 62);

  // Status line: recent message, else last LED alert, else firmware version
  if (statusMsg[0]) {
    tft.setTextColor(TFT_YELLOW, TFT_BLACK); snprintf(buf, sizeof(buf), "%s", statusMsg);
  } else if (lastAlertMs && millis() - lastAlertMs < 30UL * 60 * 1000) {
    tft.setTextColor(TFT_ORANGE, TFT_BLACK);
    snprintf(buf, sizeof(buf), "Alert: %s %s %s %.1f%s, %lum ago",
             alertName(lastAlert.kind), lastAlert.callsign, lastAlert.type,
             toDisplayDist(lastAlert.distKm), DIST_UNIT, (unsigned long)((millis() - lastAlertMs) / 60000UL));
  } else {
    tft.setTextColor(TFT_DARKGREY, TFT_BLACK);
    snprintf(buf, sizeof(buf), "Firmware %s (%s)", FW_VERSION, BUILD_ENV);
  }
  tft.drawString(buf, 8, 78);
  tft.setTextPadding(0);

  // Buttons: redraw only when a label or state changes (no flicker)
  static uint32_t lastKey = 0xFFFFFFFF;
  bool wifiArmed = millis() < wifiConfirmUntil;
  uint32_t key = (wifiArmed ? 1 : 0) | (ledAlerts ? 2 : 0) | ((uint32_t)backlightPct << 8);
  if (newPage || key != lastKey) {
    lastKey = key;
    drawMenuButton(MB_WIFI, wifiArmed ? "TAP AGAIN" : "WIFI SETUP", wifiArmed ? TFT_ORANGE : C_BLUEB);
    drawMenuButton(MB_LOCATE, "LOCATE BY IP", C_BLUEB);
    drawMenuButton(MB_CALIBRATE, "CALIBRATE TOUCH", C_BLUEB);
    snprintf(buf, sizeof(buf), "BRIGHTNESS %u%%", backlightPct);
    drawMenuButton(MB_BRIGHT, buf, C_BLUEB);
    drawMenuButton(MB_LED, ledAlerts ? "LED ALERTS ON" : "LED ALERTS OFF", ledAlerts ? 0x0400 : C_BADGE);
    drawMenuButton(MB_BACK, "< BACK", C_BADGE);
  }
}

void render() {
  bool newPage = false;
  if (screen != lastScreen) {
    tft.fillScreen(TFT_BLACK);
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
  if (onDemand) server.begin();

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
void changeRange(float delta) {
  if (configMutex) xSemaphoreTake(configMutex, portMAX_DELAY);
  radarRange = constrain(radarRange + delta, RANGE_MIN, RANGE_MAX);
  float r = radarRange;
  prefs.putFloat("range_u", r);
  if (configMutex) xSemaphoreGive(configMutex);
  Serial.printf("Radar range: %d %s\n", (int)r, DIST_UNIT);
}

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
      if (millis() < wifiConfirmUntil) { wifiConfirmUntil = 0; portalRequested = true; }
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
    case MB_BACK:
      screen = screenBeforeSettings;
      break;
  }
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

  // Gear icon (generous hit area in the top-right corner)
  if (sy < 34 && sx >= 270) {
    if (screen == SCREEN_SETTINGS) screen = screenBeforeSettings; else openSettings();
    return;
  }

  if (screen == SCREEN_SETTINGS) {
    int b = menuButtonAt(sx, sy);
    if (b >= 0) handleMenuButton(b);
    return;                                  // taps elsewhere on the menu do nothing
  }

  // Radar screen: -/+ range buttons
  if (screen == 2 && sy >= BTN_Y - 10) {
    if (sx >= BTN_MINUS_X - 8 && sx < BTN_MINUS_X + BTN_W + 5) { changeRange(-RANGE_STEP); return; }
    if (sx >= BTN_PLUS_X - 5 && sx <= BTN_PLUS_X + BTN_W + 8) { changeRange(+RANGE_STEP); return; }
  }

  // Anywhere else: next screen
  screen = (screen + 1) % NUM_SCREENS;
  lastScreenSwap = millis();
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
  body{font-family:Arial,sans-serif;background:#0a0a1a;color:#e0e0e0;margin:0;padding:20px}
  h1{color:#00ff88;text-align:center;font-size:1.4em}
  h2{color:#00ff88;font-size:1em;margin:0 0 8px}
  .card{background:#1a1a2e;border-radius:12px;padding:20px;margin:12px 0;max-width:420px;margin-left:auto;margin-right:auto}
  label{display:block;margin:10px 0 4px;color:#aaa;font-size:0.85em}
  input{width:100%;padding:10px;border:1px solid #333;border-radius:6px;background:#0d0d1f;color:#fff;font-size:1em;box-sizing:border-box}
  input:focus{border-color:#00ff88;outline:none}
  button{width:100%;padding:12px;margin-top:16px;background:#00ff88;color:#000;border:none;border-radius:8px;font-size:1.1em;font-weight:bold;cursor:pointer}
  button:hover{background:#00cc6a}
  .row{display:flex;gap:8px}.row input{flex:1}.row button{width:auto;margin-top:0;padding:10px 16px}
  .opt{background:#0d0d1f;color:#e0e0e0;border:1px solid #333;font-weight:normal;font-size:0.95em;margin-top:6px;text-align:left}
  .opt:hover{background:#22223a}
  .danger{background:#3a1a1a;color:#ff8080}.danger:hover{background:#552222}
  .info{text-align:center;color:#666;font-size:0.8em;margin-top:12px}
  .msg{color:#00ff88;text-align:center;display:none;position:fixed;left:50%;bottom:20px;transform:translateX(-50%);background:#1a1a2e;border:1px solid #00ff88;padding:10px 18px;border-radius:8px}
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

  radarSpr.setColorDepth(16);
  radarSpr.createSprite(200, 200);

  TJpgDec.setSwapBytes(true);
  TJpgDec.setCallback(jpgToTft);

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
  newDataReady = false;

  initWebServer();
  setupArduinoOta();

  // Launch background fetcher on Core 0 (Core 1 = Arduino loop)
  xTaskCreatePinnedToCore(
    dataFetcherTask,   // Function
    "DataFetch",       // Name
    20480,             // Stack (20 KB — JSON + SSL + photo download)
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
  ledUpdate();
  delay(33);
}
