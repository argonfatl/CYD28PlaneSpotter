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
#include <cydqr.h>
#include <math.h>
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
volatile bool newDataReady = false;

// Route Cache
const uint8_t ROUTE_CACHE_SIZE = 15;
struct CachedRoute {
  char callsign[10];
  char dep[5]; char arr[5]; char airline[28];
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
const uint8_t NUM_SCREENS = 5;
const uint8_t SCREEN_SETTINGS = 4;
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
bool getRoute(Aircraft& a) {
  a.dep[0] = a.arr[0] = a.airline[0] = '\0';
  const char* cs = a.callsign;
  if (strlen(cs) < 3 || strcmp(cs, "(no id)") == 0) return false;

  for (int i = 0; i < ROUTE_CACHE_SIZE; i++) {
    if (routeCache[i].valid && strcmp(routeCache[i].callsign, cs) == 0) {
      strcpy(a.dep, routeCache[i].dep);
      strcpy(a.arr, routeCache[i].arr);
      strcpy(a.airline, routeCache[i].airline);
      return a.dep[0] != '\0';
    }
  }

  JsonDocument filter;
  filter["response"]["flightroute"]["airline"]["name"] = true;
  filter["response"]["flightroute"]["origin"]["iata_code"] = true;
  filter["response"]["flightroute"]["origin"]["icao_code"] = true;
  filter["response"]["flightroute"]["destination"]["iata_code"] = true;
  filter["response"]["flightroute"]["destination"]["icao_code"] = true;

  JsonDocument d;
  int code = httpGetJson(String("https://api.adsbdb.com/v0/callsign/") + cs, d, &filter);
  if (code != HTTP_CODE_OK && code != HTTP_CODE_NOT_FOUND) return false;  // transient: don't cache

  if (code == HTTP_CODE_OK) {
    JsonObject r = d["response"]["flightroute"];
    if (!r.isNull()) {
      const char* o = r["origin"]["iata_code"] | "";
      if (!o[0]) o = r["origin"]["icao_code"] | "";
      const char* t = r["destination"]["iata_code"] | "";
      if (!t[0]) t = r["destination"]["icao_code"] | "";
      if (o[0] && t[0]) {
        copyAscii(a.dep, o, sizeof(a.dep));
        copyAscii(a.arr, t, sizeof(a.arr));
      }
      copyAscii(a.airline, r["airline"]["name"] | "", sizeof(a.airline));
    }
  }

  // Cache both hits and "unknown callsign" answers
  int replaceIdx = 0;
  uint32_t oldest = 0xFFFFFFFF;
  for (int i = 0; i < ROUTE_CACHE_SIZE; i++) {
    if (!routeCache[i].valid) { replaceIdx = i; break; }
    if (routeCache[i].fetchTime < oldest) { oldest = routeCache[i].fetchTime; replaceIdx = i; }
  }
  CachedRoute& c = routeCache[replaceIdx];
  c.valid = true;
  strcpy(c.callsign, cs);
  strcpy(c.dep, a.dep);
  strcpy(c.arr, a.arr);
  strcpy(c.airline, a.airline);
  c.fetchTime = millis();

  return a.dep[0] != '\0';
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
    for (const char* fld : {"flight", "r", "t", "lat", "lon", "alt_baro", "alt_geom", "gs",
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
        if (locTop5Count > 0) locNearest = locTop5[0];

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

void drawHeader(const char* title, bool newPage) {
  tft.setTextFont(2);
  tft.setTextColor(TFT_LIGHTGREY, TFT_BLACK);

  if (newPage) {
    char left[30]; snprintf(left, sizeof(left), "%s [%d/%d]", title, screen + 1, NUM_SCREENS);
    tft.setTextPadding(200);
    tft.drawString(left, 10, 5);
    tft.drawLine(0, 25, 320, 25, TFT_DARKGREY);
  }

  char t[10]; fmtClock(t, sizeof(t), true);
  tft.setTextPadding(80);
  tft.drawString(t, 240, 5);
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

void screenTargetIntel(bool newPage) {
  drawHeader("TARGET INTEL", newPage);

  if (!nearest.valid) {
    tft.setTextPadding(300);
    tft.setTextFont(4);
    tft.setTextColor(TFT_RED, TFT_BLACK);
    tft.drawString("NO TARGET IN RANGE", 40, 100);
    tft.setTextPadding(0);
    return;
  }

  tft.setTextPadding(140);
  tft.setTextFont(4);
  tft.setTextColor(TFT_GREEN, TFT_BLACK);
  tft.drawString(nearest.callsign, 20, 36);

  tft.setTextFont(2);
  tft.setTextColor(TFT_CYAN, TFT_BLACK);
  tft.setTextPadding(150);
  if (nearest.hasRoute) {
    char rte[32]; snprintf(rte, sizeof(rte), "Route: %s -> %s", nearest.dep, nearest.arr);
    tft.drawString(rte, 160, 42);
  } else {
    tft.drawString("Route: Unknown", 160, 42);
  }

  // Text rows stop short of the heading arrow (x >= 215)
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setTextPadding(195);
  char buf[64];

  tft.setTextColor(TFT_YELLOW, TFT_BLACK);
  tft.drawString(nearest.airline[0] ? nearest.airline : " ", 20, 68);

  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  const char* tag = (nearest.dbFlags & 1) ? " MIL" : strcmp(nearest.category, "A7") == 0 ? " HELI" : "";
  snprintf(buf, sizeof(buf), "Type: %s  Reg: %s%s",
           nearest.type[0] ? nearest.type : "?", nearest.reg[0] ? nearest.reg : "?", tag);
  tft.drawString(buf, 20, 93);

  snprintf(buf, sizeof(buf), "Distance: %.1f %s (%s)",
           toDisplayDist(nearest.distanceKm), DIST_UNIT, compass(nearest.bearingDeg));
  tft.drawString(buf, 20, 118);

  if (nearest.vrateFpm >= 300 || nearest.vrateFpm <= -300)
    snprintf(buf, sizeof(buf), "Alt: %d ft  %+d fpm", nearest.altFt, nearest.vrateFpm);
  else
    snprintf(buf, sizeof(buf), "Alt: %d ft (FL%03d)", nearest.altFt, (nearest.altFt + 50) / 100);
  tft.drawString(buf, 20, 143);

#if UNITS_IMPERIAL
  snprintf(buf, sizeof(buf), "Speed: %.0f kt (%.0f mph)", nearest.gsKt, nearest.gsKt * 1.15078f);
#else
  snprintf(buf, sizeof(buf), "Speed: %.0f kt (%.0f km/h)", nearest.gsKt, nearest.gsKt * 1.852f);
#endif
  tft.drawString(buf, 20, 168);

  tft.fillRect(220, 110, 60, 60, TFT_BLACK);
  drawArrow(250, 140, 25, nearest.trackDeg, TFT_CYAN);
  tft.setTextPadding(80);
  snprintf(buf, sizeof(buf), "HDG %03.0f", nearest.trackDeg);
  tft.drawString(buf, 225, 180);

  tft.setTextPadding(0);
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

// Settings screen: WiFi / location info + two touch buttons
const int SET_BTN_Y = 185, SET_BTN_H = 40;
const int SET_BTN_WIFI_X = 10, SET_BTN_LOC_X = 165, SET_BTN_W = 145;

void screenSettings(bool newPage) {
  drawHeader("SETTINGS", newPage);

  double hLat, hLon; float sRad; char name[48];
  if (configMutex) xSemaphoreTake(configMutex, portMAX_DELAY);
  hLat = homeLat; hLon = homeLon; sRad = searchRadiusNm;
  strcpy(name, homeName);
  if (configMutex) xSemaphoreGive(configMutex);

  char buf[64];
  tft.setTextFont(2);
  tft.setTextDatum(TL_DATUM);
  tft.setTextPadding(300);

  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  if (WiFi.status() == WL_CONNECTED) {
    snprintf(buf, sizeof(buf), "WiFi: %s (%d dBm)", WiFi.SSID().c_str(), WiFi.RSSI());
    tft.drawString(buf, 10, 34);
    tft.setTextColor(TFT_CYAN, TFT_BLACK);
    snprintf(buf, sizeof(buf), "Web settings: http://%s", WiFi.localIP().toString().c_str());
    tft.drawString(buf, 10, 54);
  } else {
    tft.setTextColor(TFT_RED, TFT_BLACK);
    tft.drawString("WiFi: not connected", 10, 34);
    tft.drawString(" ", 10, 54);
  }

  tft.setTextColor(TFT_LIGHTGREY, TFT_BLACK);
  tft.drawString("Location:", 10, 80);
  tft.setTextColor(TFT_GREEN, TFT_BLACK);
  tft.drawString(name, 10, 98);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  snprintf(buf, sizeof(buf), "%.4f, %.4f   search %d NM", hLat, hLon, (int)sRad);
  tft.drawString(buf, 10, 116);

  // LED: last alert (if within 30 min), otherwise what the LED is set to do
  bool ledAlerts, ledProx;
  if (configMutex) xSemaphoreTake(configMutex, portMAX_DELAY);
  ledAlerts = ledCfg.alerts; ledProx = ledCfg.proximity;
  if (configMutex) xSemaphoreGive(configMutex);
  if (lastAlertMs && millis() - lastAlertMs < 30UL * 60 * 1000) {
    tft.setTextColor(TFT_ORANGE, TFT_BLACK);
    snprintf(buf, sizeof(buf), "Alert: %s %s %s %.1f%s, %lum ago",
             alertName(lastAlert.kind), lastAlert.callsign, lastAlert.type,
             toDisplayDist(lastAlert.distKm), DIST_UNIT, (millis() - lastAlertMs) / 60000UL);
  } else {
    tft.setTextColor(TFT_LIGHTGREY, TFT_BLACK);
    snprintf(buf, sizeof(buf), "LED: alerts %s, distance colors %s",
             ledAlerts ? "on" : "off", ledProx ? "on" : "off");
  }
  tft.drawString(buf, 10, 138);

  tft.setTextColor(TFT_YELLOW, TFT_BLACK);
  tft.drawString(statusMsg[0] ? statusMsg : " ", 10, 160);
  tft.setTextPadding(0);

  // Buttons: redraw only when their state changes (avoids flicker)
  static int lastWifiBtn = -1;
  int wifiBtn = millis() < wifiConfirmUntil ? 1 : 0;
  if (newPage || wifiBtn != lastWifiBtn) {
    lastWifiBtn = wifiBtn;
    tft.setTextDatum(MC_DATUM);
    tft.fillRoundRect(SET_BTN_WIFI_X, SET_BTN_Y, SET_BTN_W, SET_BTN_H, 6, wifiBtn ? TFT_ORANGE : TFT_DARKGREY);
    tft.setTextColor(TFT_WHITE, wifiBtn ? TFT_ORANGE : TFT_DARKGREY);
    tft.drawString(wifiBtn ? "TAP AGAIN" : "WIFI SETUP", SET_BTN_WIFI_X + SET_BTN_W / 2, SET_BTN_Y + SET_BTN_H / 2);
    if (newPage) {
      tft.fillRoundRect(SET_BTN_LOC_X, SET_BTN_Y, SET_BTN_W, SET_BTN_H, 6, TFT_DARKGREY);
      tft.setTextColor(TFT_WHITE, TFT_DARKGREY);
      tft.drawString("LOCATE BY IP", SET_BTN_LOC_X + SET_BTN_W / 2, SET_BTN_Y + SET_BTN_H / 2);
    }
    tft.setTextDatum(TL_DATUM);
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

void checkTouch() {
  if (ts.touched()) {
    TS_Point p = ts.getPoint();

    // STRICT FILTER:
    // 1. p.z > 400 ignores light ghost touches and case pinching.
    // 2. p.x and p.y bounds ignore the 0 and 4095 coordinate spikes caused by SPI noise.
    if (p.z > 400 && p.x > 100 && p.x < 4000 && p.y > 100 && p.y < 4000) {

      if (millis() - lastTouchMs > 400) { // 400ms debounce to prevent double-skips
        lastTouchMs = millis();

        // Map raw touch to screen coordinates (CYD rotation 1)
        // Calibrated: raw ~200-3900, X-axis inverted for this panel
        int sx = 320 - (int)((p.y - 200.0f) * 320.0f / 3700.0f);
        int sy = 240 - (int)((p.x - 200.0f) * 240.0f / 3700.0f);
        sx = constrain(sx, 0, 319);
        sy = constrain(sy, 0, 239);

        static uint32_t lastDebug = 0;
        if (millis() - lastDebug > 2000) {
          Serial.printf("Touch: raw(%d,%d,%d) -> screen(%d,%d)\n", p.x, p.y, p.z, sx, sy);
          lastDebug = millis();
        }

        // Radar screen: -/+ range buttons. Hit areas match the drawn buttons,
        // padded ~8 px because the resistive panel isn't pixel-accurate.
        if (screen == 2 && sy >= BTN_Y - 10) {
          if (sx >= BTN_MINUS_X - 8 && sx < BTN_MINUS_X + BTN_W + 5) {
            changeRange(-RANGE_STEP); lastScreenSwap = millis(); return;
          }
          if (sx >= BTN_PLUS_X - 5 && sx <= BTN_PLUS_X + BTN_W + 8) {
            changeRange(+RANGE_STEP); lastScreenSwap = millis(); return;
          }
        }

        // Settings screen buttons (bottom row)
        if (screen == SCREEN_SETTINGS && sy >= SET_BTN_Y - 10) {
          if (sx < 160) {
            // WiFi setup interrupts tracking, so it needs a confirming second tap
            if (millis() < wifiConfirmUntil) { wifiConfirmUntil = 0; portalRequested = true; }
            else wifiConfirmUntil = millis() + 3000;
          } else {
            if (WiFi.status() == WL_CONNECTED) { setStatus("Locating..."); ipLocateRequested = true; }
            else setStatus("No WiFi connection");
          }
          return;
        }

        // Default: cycle to next screen
        screen = (screen + 1) % NUM_SCREENS;
        wifiConfirmUntil = 0;
        lastScreenSwap = millis();
      }
    }
  }
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
  <h2>WiFi</h2>
  <div style="font-size:0.9em;color:#aaa">Connected to <b>)rawliteral" + ssid + R"rawliteral(</b>. Forgetting it restarts the device into setup mode: join the <b>)rawliteral" + String(SETUP_AP_NAME) + R"rawliteral(</b> network from your phone to pick a new one.</div>
  <button type="button" class="danger" onclick="forget()">Forget WiFi &amp; restart</button>
</div>
<div class="info">IP: )rawliteral" + WiFi.localIP().toString() + R"rawliteral(</div>
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
  for (uint16_t c : testColors) { tft.fillScreen(c); delay(400); }

  radarSpr.setColorDepth(16);
  radarSpr.createSprite(200, 200);

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
  #undef PREF
  ledInit();

  connectWiFiAtBoot();
  configTime(0, 0, "pool.ntp.org", "time.google.com");
  setenv("TZ", TIMEZONE, 1); tzset();

  nearest.valid = false;
  newDataReady = false;

  initWebServer();

  // Launch background fetcher on Core 0 (Core 1 = Arduino loop)
  xTaskCreatePinnedToCore(
    dataFetcherTask,   // Function
    "DataFetch",       // Name
    16384,             // Stack (16 KB — enough for JSON + SSL)
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

  checkTouch();
  server.handleClient();
  render();
  ledUpdate();
  delay(33);
}
