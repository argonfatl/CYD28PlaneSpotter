/*
 * CYD28 (ESP32) Plane Spotter - Direct TFT Rendering
 * --------------------------------------------------------------------------
 * UPDATE: Merged Weather & System Status into a single unified screen.
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
#include <math.h>
#include <time.h>
#include <sys/time.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>

// =======================================================================
// CONFIGURATION — loaded from config.env
// =======================================================================
#include "config.env"

// Runtime-configurable location & radar (persisted via NVS)
// Defaults below; overridden by web UI and saved to flash.
double homeLat = DEFAULT_HOME_LAT;
double homeLon = DEFAULT_HOME_LON;
double searchRadiusDeg = DEFAULT_SEARCH_RADIUS;
float  radarMaxKm = DEFAULT_RADAR_MAX_KM;

SemaphoreHandle_t configMutex = NULL;  // protects homeLat/Lon/Radius/radarMaxKm
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
  char callsign[10]; char country[24];
  double lat; double lon; float altitudeM; float velocityMs;
  float trackDeg; float vrateMs; bool onGround; int category;
  double distanceKm; double bearingDeg; bool valid;
  char dep[5]; char arr[5]; bool hasRoute;
};
Aircraft nearest;

// Top 5 Tracker
const uint8_t MAX_TOP5 = 5;
Aircraft top5[MAX_TOP5];
uint8_t top5Count = 0;

// Blips for Radar
struct Blip { double lat; double lon; float track; float speedMs; char callsign[10]; float altitudeM; };
const uint8_t MAX_BLIPS = 20;
Blip blips[MAX_BLIPS];
uint8_t blipCount = 0;
uint32_t lastDataMs = 0;

// Async data fetch (FreeRTOS task on Core 0)
SemaphoreHandle_t dataMutex = NULL;
TaskHandle_t fetchTaskHandle = NULL;
volatile bool newDataReady = false;
volatile bool fetchInProgress = false;

// Route Cache
struct CachedRoute {
  char callsign[10];
  char dep[5]; char arr[5];
  bool valid; uint32_t fetchTime;
};
CachedRoute routeCache[15];

struct Weather {
  float tempC; float windKmh; int humidity; int code; bool valid = false;
} weather;
uint32_t lastWeatherPoll = 0;

struct Stats {
  uint32_t requestsOk = 0; uint32_t requestsFail = 0;
  uint16_t inView = 0; uint16_t maxInView = 0;
  double closestEver = 1e9; uint32_t lastUpdateMs = 0;
} stats;

uint8_t screen = 0;
uint8_t lastScreen = 255;
uint32_t lastScreenSwap = 0;
uint32_t lastTouchMs = 0;
bool firstWeatherDone = false;
const uint8_t NUM_SCREENS = 4; // Reduced from 5 to 4 screens
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

// ---------------------------------------------------------------------------
// Network & APIs
// ---------------------------------------------------------------------------
void connectWiFi() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  tft.fillScreen(TFT_BLACK);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setTextFont(4);
  tft.drawString("Connecting WiFi...", 20, 100);
  while (WiFi.status() != WL_CONNECTED) { delay(400); }
  tft.fillScreen(TFT_BLACK);
}

bool getRoute(const char* callsign, char* dep, char* arr) {
  if (strlen(callsign) == 0 || strcmp(callsign, "(no id)") == 0) return false;
  
  for (int i=0; i<15; i++) {
    if (routeCache[i].valid && strcmp(routeCache[i].callsign, callsign) == 0) {
      if (routeCache[i].dep[0]) {
         strcpy(dep, routeCache[i].dep);
         strcpy(arr, routeCache[i].arr);
         return true;
      }
      return false;
    }
  }

  WiFiClientSecure client; client.setInsecure();
  HTTPClient https; https.setReuse(false);
  String url = String("https://hexdb.io/api/v1/route/icao/") + callsign;
  bool found = false;
  dep[0] = '\0'; arr[0] = '\0';

  if (https.begin(client, url)) {
    int code = https.GET();
    if (code == HTTP_CODE_OK) {
      JsonDocument d;
      if (!deserializeJson(d, https.getString())) {
        const char* r = d["route"] | "";
        const char* dash = strchr(r, '-');
        if (r[0] && dash) {
          size_t dl = dash - r;
          if (dl < 5) {
            strncpy(dep, r, dl); dep[dl] = '\0';
            strncpy(arr, dash + 1, 4); arr[4] = '\0';
            found = true;
          }
        }
      }
    }
    https.end();
  }

  int replaceIdx = 0;
  uint32_t oldest = 0xFFFFFFFF;
  for (int i=0; i<15; i++) {
    if (!routeCache[i].valid) { replaceIdx = i; break; }
    if (routeCache[i].fetchTime < oldest) { oldest = routeCache[i].fetchTime; replaceIdx = i; }
  }
  routeCache[replaceIdx].valid = true;
  strcpy(routeCache[replaceIdx].callsign, callsign);
  strcpy(routeCache[replaceIdx].dep, dep);
  strcpy(routeCache[replaceIdx].arr, arr);
  routeCache[replaceIdx].fetchTime = millis();

  return found;
}

// Fetches aircraft data into caller-provided local buffers (thread-safe — no globals touched)
bool fetchAircraftTo(Aircraft* top5Out, uint8_t& top5CntOut,
                     Aircraft& nearOut, Blip* blipsOut, uint8_t& blipCntOut) {
  // Snapshot config under mutex (Core 0 reads, Core 1 may write via web UI)
  double hLat, hLon, sRad; float rMax;
  if (configMutex) xSemaphoreTake(configMutex, portMAX_DELAY);
  hLat = homeLat; hLon = homeLon; sRad = searchRadiusDeg; rMax = radarMaxKm;
  if (configMutex) xSemaphoreGive(configMutex);

  WiFiClientSecure client; client.setInsecure();
  HTTPClient https; https.setReuse(false);
  String url = "https://opensky-network.org/api/states/all?lamin=" + String(hLat - sRad, 4) + 
         "&lomin=" + String(hLon - sRad, 4) + "&lamax=" + String(hLat + sRad, 4) + 
         "&lomax=" + String(hLon + sRad, 4) + "&extended=1";

  if (!https.begin(client, url)) { stats.requestsFail++; return false; }
  int code = https.GET();
  if (code != HTTP_CODE_OK) { https.end(); stats.requestsFail++; return false; }

  JsonDocument filter;
  JsonArray el = filter["states"].to<JsonArray>().add<JsonArray>();
  for (int i = 0; i <= 17; i++) el[i] = false;
  el[0] = el[1] = el[2] = el[5] = el[6] = el[7] = el[8] = el[9] = el[10] = el[11] = el[13] = el[17] = true;

  String payload = https.getString(); https.end();
  JsonDocument doc;
  if (deserializeJson(doc, payload, DeserializationOption::Filter(filter))) { stats.requestsFail++; return false; }

  JsonArray states = doc["states"].as<JsonArray>();
  uint16_t count = 0; 
  blipCntOut = 0;
  top5CntOut = 0;

  for (JsonArray s : states) {
    if (s.isNull() || s[5].isNull() || s[6].isNull()) continue;

    // Skip aircraft on the ground — only track those in flight
    if (s[8] | false) continue;

    double lon = s[5].as<double>(), lat = s[6].as<double>();
    double d = haversineKm(hLat, hLon, lat, lon);
    double brg = bearingDeg(hLat, hLon, lat, lon);
    count++;

    if (blipCntOut < MAX_BLIPS) {
      blipsOut[blipCntOut].lat = lat; blipsOut[blipCntOut].lon = lon;
      blipsOut[blipCntOut].track = s[10] | 0.0f;
      blipsOut[blipCntOut].speedMs = (s[8] | false) ? 0.0f : (s[9] | 0.0f);
      blipsOut[blipCntOut].altitudeM = s[13].isNull() ? (s[7] | 0.0f) : s[13].as<float>();
      const char* cs = s[1] | "";
      strncpy(blipsOut[blipCntOut].callsign, cs, 9);
      blipsOut[blipCntOut].callsign[9] = '\0';
      blipCntOut++;
    }

    Aircraft a;
    a.distanceKm = d; a.lat = lat; a.lon = lon; a.bearingDeg = brg;
    a.onGround = s[8] | false; a.category = s[17] | 0;
    a.altitudeM = s[13].isNull() ? (s[7] | 0.0f) : s[13].as<float>();
    a.velocityMs = s[9] | 0.0f; a.trackDeg = s[10] | 0.0f; a.vrateMs = s[11] | 0.0f;
    a.hasRoute = false; a.dep[0] = '\0'; a.arr[0] = '\0';
    a.valid = true;
    
    strncpy(a.callsign, s[1] | "", 9); a.callsign[9] = '\0';
    for (int i = strlen(a.callsign) - 1; i >= 0 && a.callsign[i] == ' '; i--) a.callsign[i] = '\0';
    if (a.callsign[0] == '\0') strcpy(a.callsign, "(no id)");
    strncpy(a.country, s[2] | "?", 23); a.country[23] = '\0';

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
  locNearest.valid = false;

  for (;;) {
    // Wait for the next poll interval (sleep in 1s chunks so task is responsive)
    for (int t = 0; t < UPDATE_INTERVAL_MS / 1000; t++) {
      vTaskDelay(pdMS_TO_TICKS(1000));
    }

    fetchInProgress = true;
    bool ok = fetchAircraftTo(locTop5, locTop5Count, locNearest, locBlips, locBlipCount);
    fetchInProgress = false;

    if (ok) {
      // Fetch routes for the top 5 (quick per-call, but may still block a bit)
      for (int i = 0; i < locTop5Count; i++) {
        locTop5[i].hasRoute = getRoute(locTop5[i].callsign, locTop5[i].dep, locTop5[i].arr);
        locNearest = locTop5[0]; // nearest = closest after route resolution
      }

      // Atomically publish results to the globals used by render()
      xSemaphoreTake(dataMutex, portMAX_DELAY);
      memcpy(blips, locBlips, sizeof(locBlips));
      blipCount = locBlipCount;
      memcpy(top5, locTop5, sizeof(locTop5));
      top5Count = locTop5Count;
      nearest = locNearest;
      lastDataMs = millis();
      stats.lastUpdateMs = lastDataMs;
      newDataReady = true;
      xSemaphoreGive(dataMutex);
    }
  }
}

bool fetchWeather() {
  // Snapshot config under mutex
  double hLat, hLon;
  if (configMutex) xSemaphoreTake(configMutex, portMAX_DELAY);
  hLat = homeLat; hLon = homeLon;
  if (configMutex) xSemaphoreGive(configMutex);

  WiFiClientSecure client; client.setInsecure(); HTTPClient https; https.setReuse(false);
  String url = "https://api.open-meteo.com/v1/forecast?latitude=" + String(hLat, 4) + "&longitude=" + String(hLon, 4) + "&current=temperature_2m,relative_humidity_2m,wind_speed_10m";
  if (!https.begin(client, url)) return false;
  int code = https.GET();
  if (code != HTTP_CODE_OK) { https.end(); return false; }
  String payload = https.getString(); https.end();

  JsonDocument doc; if (deserializeJson(doc, payload)) return false;
  JsonObject c = doc["current"]; if (c.isNull()) return false;
  
  weather.tempC = c["temperature_2m"] | 0.0f; weather.humidity = c["relative_humidity_2m"] | 0;
  weather.windKmh = c["wind_speed_10m"] | 0.0f; weather.valid = true;
  return true;
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
  const uint16_t ink = tft.color565(7, 15, 29);
  const uint16_t edge = tft.color565(28, 49, 68);
  const uint16_t cyan = tft.color565(71, 220, 232);
  if (newPage) {
    tft.fillRect(0, 0, 320, 27, ink);
    tft.fillRect(0, 26, 320, 1, edge);
  }
  tft.setTextFont(2);
  tft.setTextColor(cyan, ink);
  tft.setTextPadding(0);
  tft.drawString(title, 10, 5);
  char t[10]; fmtClock(t, sizeof(t), true);
  tft.setTextColor(TFT_LIGHTGREY, ink);
  tft.setTextPadding(60);
  tft.drawString(t, 250, 5);
  tft.setTextPadding(0);
  // Small page markers make the four-screen carousel easy to scan.
  for (uint8_t i = 0; i < NUM_SCREENS; i++)
    tft.fillCircle(286 + i * 8, 31, 2, i == screen ? cyan : edge);
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
  const uint16_t panel = tft.color565(13, 27, 43);
  const uint16_t edge = tft.color565(31, 56, 76);
  const uint16_t cyan = tft.color565(71, 220, 232);
  const uint16_t muted = tft.color565(137, 163, 181);
  drawHeader("NEAREST AIRCRAFT", newPage);
  if (!nearest.valid) {
    tft.fillRoundRect(12, 58, 296, 112, 10, panel);
    tft.drawRoundRect(12, 58, 296, 112, 10, edge);
    tft.fillCircle(160, 90, 8, tft.color565(255, 180, 70));
    tft.setTextFont(4); tft.setTextColor(TFT_WHITE, panel);
    tft.setTextPadding(0); tft.drawCentreString("NO CONTACT", 160, 112, 4);
    tft.setTextFont(2); tft.setTextColor(muted, panel);
    tft.drawCentreString("Waiting for aircraft in range", 160, 146, 2);
    return;
  }
  // Identity card
  tft.fillRoundRect(10, 39, 300, 52, 9, panel);
  tft.drawRoundRect(10, 39, 300, 52, 9, edge);
  tft.fillRoundRect(10, 39, 4, 52, 2, cyan);
  tft.setTextFont(4); tft.setTextColor(TFT_WHITE, panel); tft.setTextPadding(0);
  tft.drawString(nearest.callsign, 22, 47);
  tft.setTextFont(2); tft.setTextColor(cyan, panel);
  char route[24];
  if (nearest.hasRoute) snprintf(route, sizeof(route), "%s  >  %s", nearest.dep, nearest.arr);
  else strcpy(route, "ROUTE UNKNOWN");
  tft.drawRightString(route, 298, 56, 2);
  tft.setTextColor(muted, panel);
  tft.drawRightString(nearest.country, 298, 72, 2);

  // Four compact data tiles
  const int xs[2] = {10, 164};
  for (int i = 0; i < 2; i++) {
    tft.fillRoundRect(xs[i], 99, 146, 52, 8, panel);
    tft.drawRoundRect(xs[i], 99, 146, 52, 8, edge);
    tft.fillRoundRect(xs[i], 160, 146, 52, 8, panel);
    tft.drawRoundRect(xs[i], 160, 146, 52, 8, edge);
  }
  char value[30];
  tft.setTextFont(2); tft.setTextColor(muted, panel);
  tft.drawString("DISTANCE / BEARING", 20, 105);
  snprintf(value, sizeof(value), "%.1f km  %s", nearest.distanceKm, compass(nearest.bearingDeg));
  tft.setTextFont(4); tft.setTextColor(TFT_WHITE, panel); tft.drawString(value, 20, 121);

  tft.setTextFont(2); tft.setTextColor(muted, panel);
  tft.drawString("ALTITUDE", 174, 105);
  float altFeet = nearest.altitudeM * 3.28084f;
  if (nearest.onGround) strcpy(value, "GROUND");
  else snprintf(value, sizeof(value), "FL%03.0f", altFeet / 100.0f);
  tft.setTextFont(4); tft.setTextColor(TFT_WHITE, panel); tft.drawString(value, 174, 121);

  tft.setTextFont(2); tft.setTextColor(muted, panel);
  tft.drawString("GROUND SPEED", 20, 166);
  snprintf(value, sizeof(value), "%.0f km/h", nearest.velocityMs * 3.6f);
  tft.setTextFont(4); tft.setTextColor(TFT_WHITE, panel); tft.drawString(value, 20, 182);

  tft.setTextFont(2); tft.setTextColor(muted, panel);
  tft.drawString("HEADING", 174, 166);
  drawArrow(267, 184, 15, nearest.trackDeg, cyan);
  snprintf(value, sizeof(value), "%03.0f°", nearest.trackDeg);
  tft.setTextColor(TFT_WHITE, panel); tft.setTextPadding(42);
  tft.drawString(value, 174, 184); tft.setTextPadding(0);
}

void screenTop5(bool newPage) {
  const uint16_t panel = tft.color565(13, 27, 43);
  const uint16_t edge = tft.color565(31, 56, 76);
  const uint16_t cyan = tft.color565(71, 220, 232);
  const uint16_t muted = tft.color565(137, 163, 181);
  drawHeader("TRAFFIC NEAR YOU", newPage);
  tft.setTextFont(2); tft.setTextColor(muted, TFT_BLACK);
  tft.setTextPadding(0);
  tft.drawString("FLIGHT", 18, 42);
  tft.drawString("ROUTE", 112, 42);
  tft.drawRightString("ALT / DIST", 302, 42, 2);
  tft.drawLine(12, 60, 308, 60, edge);
  if (top5Count == 0) {
    tft.fillRoundRect(12, 76, 296, 80, 9, panel);
    tft.drawRoundRect(12, 76, 296, 80, 9, edge);
    tft.setTextFont(4); tft.setTextColor(TFT_WHITE, panel);
    tft.drawCentreString("NO TRAFFIC", 160, 91, 4);
    tft.setTextFont(2); tft.setTextColor(muted, panel);
    tft.drawCentreString("No aircraft detected nearby", 160, 128, 2);
    return;
  }
  for (int i = 0; i < MAX_TOP5; i++) {
    const int y = 66 + i * 33;
    tft.fillRoundRect(10, y, 300, 29, 6, panel);
    if (i < top5Count) {
      tft.fillRoundRect(10, y, 3, 29, 2, i == 0 ? cyan : edge);
      tft.setTextFont(2); tft.setTextColor(TFT_WHITE, panel);
      tft.setTextPadding(82); tft.drawString(top5[i].callsign, 18, y + 8);
      char route[16];
      if (top5[i].hasRoute) snprintf(route, sizeof(route), "%s>%s", top5[i].dep, top5[i].arr);
      else strcpy(route, "--");
      tft.setTextColor(cyan, panel); tft.setTextPadding(65);
      tft.drawString(route, 112, y + 8);
      char data[24];
      if (top5[i].onGround) snprintf(data, sizeof(data), "GND  %.0fkm", top5[i].distanceKm);
      else snprintf(data, sizeof(data), "FL%03.0f %.0fkm",
                    top5[i].altitudeM * 3.28084f / 100.0f, top5[i].distanceKm);
      tft.setTextColor(TFT_WHITE, panel); tft.setTextPadding(0);
      tft.drawRightString(data, 302, y + 8, 2);
    }
  }
  tft.setTextPadding(0);
}

void screenRadar(bool newPage) {
  drawHeader("RADAR PPI", newPage);

  // Snapshot config under mutex
  double hLat, hLon; float rMax;
  if (configMutex) xSemaphoreTake(configMutex, portMAX_DELAY);
  hLat = homeLat; hLon = homeLon; rMax = radarMaxKm;
  if (configMutex) xSemaphoreGive(configMutex);

  tft.setTextFont(2); tft.setTextColor(TFT_WHITE, TFT_BLACK); tft.setTextPadding(110);
  char buf[32]; snprintf(buf, sizeof(buf), "Contacts: %u", blipCount);
  tft.drawString(buf, 5, 50);
  if(nearest.valid) {
    snprintf(buf, sizeof(buf), "TGT: %s", nearest.callsign);
    tft.drawString(buf, 5, 80);
    snprintf(buf, sizeof(buf), "%.1f km", nearest.distanceKm);
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
    if (blips[i].speedMs > 0 && elapsed > 0)
      projectLatLon(blips[i].lat, blips[i].lon, blips[i].track, blips[i].speedMs * elapsed, la, lo);

    double dist = haversineKm(hLat, hLon, la, lo);
    float fr = (float)(dist / rMax);
    if (fr > 1) continue;

    int rr = (int)(fr * R);
    double brg = bearingDeg(hLat, hLon, la, lo);
    int bx = cx + (int)(sin(deg2rad(brg)) * rr);
    int by = cy - (int)(cos(deg2rad(brg)) * rr);

    float behind = fmodf(sweepDeg - (float)brg + 360.0f, 360.0f);
    if (behind < 30) radarSpr.fillCircle(bx, by, 3, TFT_YELLOW);
    else radarSpr.fillRect(bx, by, 2, 2, TFT_GREEN);

    // Draw callsign label in smallest legible font
    if (blips[i].callsign[0] && strcmp(blips[i].callsign, "(no id)") != 0) {
      int lx = bx + 4;
      int ly = by - 4;
      if (lx + 54 > 200) lx = bx - 54;
      if (ly < 0) ly = 0;
      if (ly > 192) ly = 192;
      radarSpr.drawString(blips[i].callsign, lx, ly);

      // Draw FL below callsign
      float altFt = blips[i].altitudeM * 3.28084f;
      char flBuf[10];
      snprintf(flBuf, sizeof(flBuf), "FL%03.0f", altFt / 100.0f);
      int ly2 = ly + 8;
      if (ly2 > 192) ly2 = 192;
      radarSpr.drawString(flBuf, lx, ly2);

      // Draw speed (knots) below FL
      float speedKt = blips[i].speedMs * 1.94384f;
      char spBuf[10];
      snprintf(spBuf, sizeof(spBuf), "%ukn", (unsigned int)(speedKt + 0.5f));
      int ly3 = ly + 16;
      if (ly3 > 192) ly3 = 192;
      radarSpr.drawString(spBuf, lx, ly3);
    }
  }
  radarSpr.pushSprite(115, 35);

  // --- Range indicator & +/- buttons (bottom-left, side by side) ---
  // Range label
  tft.setTextFont(2);
  tft.setTextColor(TFT_CYAN, TFT_BLACK);
  tft.setTextPadding(80);
  char rng[16]; snprintf(rng, sizeof(rng), "RNG %d km", (int)rMax);
  tft.drawString(rng, 10, 193);

  // "-" button (left, 30x20)
  tft.fillRoundRect(10, 215, 30, 20, 4, TFT_DARKGREY);
  tft.setTextColor(TFT_WHITE, TFT_DARKGREY);
  tft.setTextFont(2);
  tft.setTextPadding(30);
  tft.drawString("-", 20, 216);

  // "+" button (right of "-", 30x20)
  tft.fillRoundRect(50, 215, 30, 20, 4, TFT_DARKGREY);
  tft.drawString("+", 60, 216);

  tft.setTextPadding(0);
}

// Unified Weather and System Screen
void screenWeatherSystem(bool newPage) {
  const uint16_t panel = tft.color565(13, 27, 43);
  const uint16_t edge = tft.color565(31, 56, 76);
  const uint16_t cyan = tft.color565(71, 220, 232);
  const uint16_t muted = tft.color565(137, 163, 181);
  drawHeader("WEATHER & STATUS", newPage);

  // Weather summary
  tft.fillRoundRect(10, 42, 300, 75, 9, panel);
  tft.drawRoundRect(10, 42, 300, 75, 9, edge);
  tft.fillRoundRect(10, 42, 4, 75, 2, cyan);
  tft.setTextFont(2); tft.setTextColor(muted, panel); tft.setTextPadding(0);
  tft.drawString("LOCAL CONDITIONS", 22, 49);
  if (!weather.valid) {
    tft.setTextFont(4); tft.setTextColor(TFT_WHITE, panel);
    tft.drawString("NO WX DATA", 22, 72);
  } else {
    char value[24];
    tft.setTextFont(4); tft.setTextColor(TFT_WHITE, panel);
    snprintf(value, sizeof(value), "%.1f° C", weather.tempC);
    tft.drawString(value, 22, 68);
    tft.setTextFont(2); tft.setTextColor(cyan, panel);
    snprintf(value, sizeof(value), "HUM  %d%%", weather.humidity);
    tft.drawString(value, 176, 72);
    snprintf(value, sizeof(value), "WIND  %.0f km/h", weather.windKmh);
    tft.drawString(value, 176, 94);
  }

  // System health in a clean two-column grid
  tft.setTextFont(2); tft.setTextColor(muted, TFT_BLACK);
  tft.drawString("DEVICE STATUS", 12, 126);
  const int x[2] = {10, 164};
  for (int i = 0; i < 2; i++) {
    tft.fillRoundRect(x[i], 146, 146, 40, 7, panel);
    tft.drawRoundRect(x[i], 146, 146, 40, 7, edge);
    tft.fillRoundRect(x[i], 194, 146, 40, 7, panel);
    tft.drawRoundRect(x[i], 194, 146, 40, 7, edge);
  }
  char value[32];
  uint32_t up = millis() / 1000;
  snprintf(value, sizeof(value), "%02lu:%02lu:%02lu", up / 3600, (up % 3600) / 60, up % 60);
  tft.setTextColor(muted, panel); tft.drawString("UPTIME", 20, 150);
  tft.setTextFont(4); tft.setTextColor(TFT_WHITE, panel); tft.drawString(value, 20, 164);
  tft.setTextFont(2); tft.setTextColor(muted, panel);
  snprintf(value, sizeof(value), "WIFI  %d dBm", WiFi.RSSI());
  tft.drawString(value, 174, 160);
  snprintf(value, sizeof(value), "MEMORY  %d KB", ESP.getFreeHeap() / 1024);
  tft.drawString(value, 20, 209);
  snprintf(value, sizeof(value), "API  %lu OK / %lu ERR", stats.requestsOk, stats.requestsFail);
  tft.drawString(value, 174, 209);
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
  }
  if (dataMutex) xSemaphoreGive(dataMutex);
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

        // Debug: print touch coords for calibration
        static uint32_t lastDebug = 0;
        if (millis() - lastDebug > 2000) {
          Serial.printf("Touch: raw(%d,%d,%d) -> screen(%d,%d)\n", p.x, p.y, p.z, sx, sy);
          lastDebug = millis();
        }

        // Screen-specific touch handling
        if (screen == 2) {
          // Radar screen: check +/- range buttons (bottom-left, stacked vertically)
          // "+" button: sx=10..50, sy=160..205
          if (sx >= 10 && sx <= 50 && sy >= 160 && sy <= 205) {
            if (configMutex) xSemaphoreTake(configMutex, portMAX_DELAY);
            radarMaxKm = min(200.0f, radarMaxKm + 10.0f);
            if (configMutex) xSemaphoreGive(configMutex);
            prefs.putFloat("range", radarMaxKm);
            Serial.printf("Range increased: %d km\n", (int)radarMaxKm);
            lastScreenSwap = millis();
            return;
          }
          // "-" button: sx=10..50, sy=206..240
          if (sx >= 10 && sx <= 50 && sy >= 206 && sy <= 240) {
            if (configMutex) xSemaphoreTake(configMutex, portMAX_DELAY);
            radarMaxKm = max(10.0f, radarMaxKm - 10.0f);
            if (configMutex) xSemaphoreGive(configMutex);
            prefs.putFloat("range", radarMaxKm);
            Serial.printf("Range decreased: %d km\n", (int)radarMaxKm);
            lastScreenSwap = millis();
            return;
          }
        }

        // Default: cycle to next screen
        screen = (screen + 1) % NUM_SCREENS;
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
    // Snapshot current values
    double hLat, hLon, sRad; float rMax;
    if (configMutex) xSemaphoreTake(configMutex, portMAX_DELAY);
    hLat = homeLat; hLon = homeLon; sRad = searchRadiusDeg; rMax = radarMaxKm;
    if (configMutex) xSemaphoreGive(configMutex);

    String html = R"rawliteral(
<!DOCTYPE html><html><head>
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>CYD Plane Spotter</title>
<style>
  body{font-family:Arial,sans-serif;background:#0a0a1a;color:#e0e0e0;margin:0;padding:20px}
  h1{color:#00ff88;text-align:center;font-size:1.4em}
  .card{background:#1a1a2e;border-radius:12px;padding:20px;margin:12px 0;max-width:420px;margin-left:auto;margin-right:auto}
  label{display:block;margin:10px 0 4px;color:#aaa;font-size:0.85em}
  input{width:100%;padding:10px;border:1px solid #333;border-radius:6px;background:#0d0d1f;color:#fff;font-size:1em;box-sizing:border-box}
  input:focus{border-color:#00ff88;outline:none}
  button{width:100%;padding:12px;margin-top:16px;background:#00ff88;color:#000;border:none;border-radius:8px;font-size:1.1em;font-weight:bold;cursor:pointer}
  button:hover{background:#00cc6a}
  .info{text-align:center;color:#666;font-size:0.8em;margin-top:12px}
  .saved{color:#00ff88;text-align:center;display:none;margin-top:8px}
</style></head><body>
<h1>&#9992; CYD Plane Spotter</h1>
<div class="card">
  <form id="cfg" onsubmit="save(event)">
    <label>Home Latitude (&deg;)</label>
    <input type="number" step="any" name="lat" id="lat" value=")rawliteral" + String(hLat, 6) + R"rawliteral(" required>
    <label>Home Longitude (&deg;)</label>
    <input type="number" step="any" name="lon" id="lon" value=")rawliteral" + String(hLon, 6) + R"rawliteral(" required>
    <label>Search Radius (&deg;)</label>
    <input type="number" step="any" name="radius" id="radius" value=")rawliteral" + String(sRad, 4) + R"rawliteral(" required>
    <label>Radar Max Range (km)</label>
    <input type="number" step="any" name="range" id="range" value=")rawliteral" + String(rMax, 1) + R"rawliteral(" required>
    <button type="submit">Save Settings</button>
  </form>
  <div class="saved" id="msg">&#10003; Settings saved!</div>
</div>
<div class="info">IP: )rawliteral" + WiFi.localIP().toString() + R"rawliteral(</div>
<script>
function save(e){
  e.preventDefault();
  var x=new XMLHttpRequest();
  x.open('POST','/save',true);
  x.setRequestHeader('Content-Type','application/x-www-form-urlencoded');
  x.onload=function(){
    if(x.status==200){document.getElementById('msg').style.display='block';
    setTimeout(function(){document.getElementById('msg').style.display='none'},3000)}
  };
  x.send('lat='+encodeURIComponent(document.getElementById('lat').value)+
         '&lon='+encodeURIComponent(document.getElementById('lon').value)+
         '&radius='+encodeURIComponent(document.getElementById('radius').value)+
         '&range='+encodeURIComponent(document.getElementById('range').value));
}
</script></body></html>)rawliteral";

    server.send(200, "text/html", html);
  });

  // POST /save — persist new config
  server.on("/save", []() {
    if (!server.hasArg("lat") || !server.hasArg("lon") || !server.hasArg("radius") || !server.hasArg("range")) {
      server.send(400, "text/plain", "Missing fields");
      return;
    }

    double newLat = server.arg("lat").toDouble();
    double newLon = server.arg("lon").toDouble();
    double newRad = server.arg("radius").toDouble();
    float  newRng = server.arg("range").toFloat();

    // Basic validation
    if (newLat < -90 || newLat > 90 || newLon < -180 || newLon > 180 ||
        newRad <= 0 || newRad > 10 || newRng <= 0 || newRng > 500) {
      server.send(400, "text/plain", "Invalid values");
      return;
    }

    // Atomically update
    if (configMutex) xSemaphoreTake(configMutex, portMAX_DELAY);
    homeLat = newLat;
    homeLon = newLon;
    searchRadiusDeg = newRad;
    radarMaxKm = newRng;
    if (configMutex) xSemaphoreGive(configMutex);

    // Persist to NVS
    prefs.putDouble("lat", newLat);
    prefs.putDouble("lon", newLon);
    prefs.putDouble("radius", newRad);
    prefs.putFloat("range", newRng);

    Serial.printf("Config saved: lat=%.6f lon=%.6f radius=%.4f range=%.1f\n",
                  newLat, newLon, newRad, newRng);
    server.send(200, "text/plain", "OK");
  });

  server.begin();
  Serial.println("Web server started on http://" + WiFi.localIP().toString() + "/");
}

// ---------------------------------------------------------------------------
// Main Arduino Loop
// ---------------------------------------------------------------------------
void setup() {
  Serial.begin(115200);
  
  touchSpi.begin(XPT2046_CLK, XPT2046_MISO, XPT2046_MOSI, -1);
  ts.begin(touchSpi); 
  ts.setRotation(1);

  tft.begin();
  tft.setRotation(1); 
  tft.invertDisplay(false); 
  
  radarSpr.setColorDepth(16);
  radarSpr.createSprite(200, 200);

  tft.fillScreen(TFT_BLACK);
  tft.setTextFont(4); tft.setTextColor(TFT_GREEN);
  tft.drawString("CYD PLANE SPOTTER", 20, 100);
  delay(1500);

  for (int i=0; i<15; i++) routeCache[i].valid = false;

  connectWiFi();
  configTime(0, 0, "pool.ntp.org", "time.google.com");
  setenv("TZ", TIMEZONE, 1); tzset();

  nearest.valid = false;
  newDataReady = false;
  fetchInProgress = false;

  // Load persisted config (or use defaults)
  prefs.begin("cyd-spotter", false);
  homeLat = prefs.getDouble("lat", homeLat);
  homeLon = prefs.getDouble("lon", homeLon);
  searchRadiusDeg = prefs.getDouble("radius", searchRadiusDeg);
  radarMaxKm = prefs.getFloat("range", radarMaxKm);
  Serial.printf("Config loaded: lat=%.6f lon=%.6f radius=%.4f range=%.1f\n",
                homeLat, homeLon, searchRadiusDeg, radarMaxKm);

  // Create mutexes before the task (task uses them)
  dataMutex = xSemaphoreCreateMutex();
  configMutex = xSemaphoreCreateMutex();

  // Start web config server
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
  uint32_t now = millis();

  // WiFi health — reconnect if needed (draws to TFT so must be Core 1)
  if (WiFi.status() != WL_CONNECTED) connectWiFi();

  // Weather fetch still runs in loop (it's infrequent, every 10 min)
  if (!firstWeatherDone || now - lastWeatherPoll >= WEATHER_INTERVAL_MS) {
    if (WiFi.status() == WL_CONNECTED) fetchWeather();
    lastWeatherPoll = now; firstWeatherDone = true;
  }

  checkTouch();

  // Serve web config requests
  server.handleClient();

  // Auto-screen rotation (re-enabled)
  //if (now - lastScreenSwap >= SCREEN_SWAP_MS) {
  //  screen = (screen + 1) % NUM_SCREENS;
  //  lastScreenSwap = now;
  //}

  render();
  delay(33);
}