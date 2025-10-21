#include <Wire.h>
#include "SSD1306Ascii.h"
#include "SSD1306AsciiWire.h"
#include <OneButton.h>
#include <ESP32Servo.h>
#include <Preferences.h>
#include <math.h>
#include "esp_sleep.h"
#include <WiFi.h>
#include <WebServer.h>
#include <time.h>
#include <sys/time.h>
#include <string.h>
#include <stdlib.h>
#include "driver/gpio.h"

// ============================================================================
//  External 32 kHz RTC clock – includes & guards
// ============================================================================
extern "C" {
  #if __has_include("rtc_clk.h")
    #include "rtc_clk.h"
  #endif
  #if __has_include("soc/rtc.h")
    #include "soc/rtc.h"
  #endif
}

// ============================================================================
//  Board / IO
// ============================================================================
#define LED_PIN (-1)         // disabled (SDA=8 is used for I2C on many C3 boards)
#define I2C_ADDRESS 0x3C
#define I2C_SDA 8
#define I2C_SCL 9

// Buttons
#define BTN_UP     5
#define BTN_DOWN   6
#define BTN_RESET  7
#define BTN_START  10

// Servo
#define SERVO_PIN 0

// Battery/VIN ADC pins (ESP32-C3 ADC1 channels)
#define BAT_ADC 1
#define VIN_ADC 2

// ============================================================================
//  Globals
// ============================================================================
SSD1306AsciiWire oled;
OneButton btnUp(BTN_UP, true);
OneButton btnDown(BTN_DOWN, true);
OneButton btnReset(BTN_RESET, true);
OneButton btnStart(BTN_START, true);
Servo myServo;
int servoPos = 0;
Preferences prefs;
WebServer server(80);

// Timer
const unsigned long MAX_SECONDS = 359999UL; // 99:59:59
unsigned long countdownSeconds = 0;
unsigned long originalDuration = 0;
unsigned long lastUpdate = 0;
bool running = false;

// Mode
enum LockMode : uint8_t { MODE_RELATIVE = 0, MODE_ABSOLUTE = 1 };
LockMode lockMode = MODE_RELATIVE;
time_t unlockAt = 0;            // target (epoch, today)
time_t lockStartedAt = 0;       // when locking started (for absolute mode)
bool timeIsSet = false;

// Stats
unsigned long totalReclaimed = 0;

// Fast increment
bool fastIncrementUp = false;
bool fastIncrementDown = false;
unsigned long lastIncrementUp = 0;
unsigned long lastIncrementDown = 0;

// Display
char lastDisplay[9] = "00:00:00";
const uint8_t COL_CENTER_REL = 16;
const uint8_t COL_CENTER_ABS = 34;

// Power / sleep
const unsigned long INACTIVITY_MS = 60000UL;
unsigned long lastInteractionMs = 0;
bool displayIsOn = true;

// Absolute target (today)
int absHour = 0;
int absMin  = 0;

// Battery / VIN
const float R1 = 100000.0f, R2 = 100000.0f;
const float VOLT_DIV = (R1 + R2) / R2; // 2.0
const float CAL = 1.00f;
const unsigned long BATT_INTERVAL_MS = 3000;
const int BATT_SAMPLES = 12;
unsigned long lastBattMs = 0;
float lastBattV = -1.0f;
int lastBattPct = -1;

const float VR1 = 100000.0f, VR2 = 100000.0f;
const float VOLT_DIV_VIN = (VR1 + VR2) / VR2; // 2.0
const float CAL_VIN = 1.00f;
const unsigned long VIN_INTERVAL_MS = 2000;
const int VIN_SAMPLES = 10;
unsigned long lastVinMs = 0;
float lastVin = -1.0f;

// Charging detection interval (actual VIN in volts)
const float VIN_CHARGE_MIN = 3.30f;  // lower bound (e.g., node 1.7 V with 1:1 divider)
const float VIN_CHARGE_MAX = 5.6f;  // upper bound (protect against noise / mis-wire)
bool lastChargingShown = false;
bool isChargingNow = false;
float lastVinDisplay = 0.0f;  // Debug: store last measured VIN for OLED

// Menu
bool showMenu = false;
int menuIndex = 0;              // 0=Stats, 1=About, 2=Mode, 3=Clock, 4=Setup Portal
const int MENU_COUNT = 5;

// Persistence
const uint32_t CONFIG_MAGIC = 0xC0FFEE11;

// Wi-Fi / NTP
char wifiSsid[33] = {0};
char wifiPass[65] = {0};

int   lastNtpSyncYday   = -1;
time_t lastNtpSyncEpoch = 0;

const char* NTP_SERVER1 = "pool.ntp.org";
const char* TZ_STR      = "CET-1CEST,M3.5.0/2,M10.5.0/3";

// Preference keys
const char* KEY_WIFI_SSID = "wifi_ssid";
const char* KEY_WIFI_PASS = "wifi_pass";
const char* KEY_NTP_YDAY  = "ntp_yday";
const char* KEY_NTP_EPOCH = "ntp_epoch";
const char* KEY_LOCKSTART = "lockStart";     // persist lockStartedAt

// AP / Web
bool portalActive = false;
unsigned long portalStartedMs = 0;
const unsigned long PORTAL_TIMEOUT_MS = 300000UL;
const char* AP_SSID = "SafeSetup";
const char* AP_PASS = "12345678";

// External 32 kHz
bool ext32k_ok = false;

// ============================================================================
//  Prototypes
// ============================================================================
void enableExternal32K();
void noteInteraction();
void oledDisplayOn();
void oledDisplayOff();
void setupLightSleepWakePins();
void maybeEnterLightSleep();
void lockServo();
void unlockServo();

void clearRows(uint8_t row_from, uint8_t rows);
void print2XBlock(uint8_t rowTop, uint8_t col, const char* txt);
void displayMainHeader(int pct);
void displayTimeRel(bool force);
void displayTimeAbs(bool force);
void displayChargingLine();

float readBatteryVoltage();
float readVinVoltage();
int liionPercentFromVoltage(float v);

void showStatsPage();
void showMainPage();
void showAboutPage();
void showModePage();
void showClockPage();
void showPortalPage();
void renderMenuPage();


void startTimePortal();
void stopTimePortal();
void handleRoot();
void handleSet();
void handleWifi();

void setSystemEpoch(time_t nowEpoch);
time_t nowEpoch();

void absInitFromNowOrUnlock();
void absClampToToday();
time_t absMakeEpochToday();
void absAdjustMinutes(int delta);

void applyDefaultsAndSave(bool keepTotal);
void saveState();
void loadState();

void loadWifiCreds();
void saveWifiCreds();
bool connectWifiSTA(uint32_t timeoutMs);
bool syncTimeViaNtp(uint32_t waitMs);
void disconnectWifi();
void maybeHourlyNtpSync();

// ============================================================================
//  External 32 kHz
// ============================================================================
void enableExternal32K() {
#if defined(RTC_SLOW_FREQ_32K_XTAL) && defined(RTC_PLL_FREQ_480M)
  #if __has_include("rtc_clk.h")
    rtc_clk_32k_enable(true);
    bool stable = false;
    for (int i = 0; i < 200; ++i) {
      if (rtc_clk_32k_enabled()) { stable = true; break; }
      delay(10);
    }
    if (stable) {
      rtc_clk_slow_freq_set(RTC_SLOW_FREQ_32K_XTAL);
      ext32k_ok = true;
    } else {
      rtc_clk_32k_enable(false);
      rtc_clk_slow_freq_set(RTC_SLOW_FREQ_RTC);
      ext32k_ok = false;
    }
  #endif
#else
  ext32k_ok = false;
#endif
}

// ============================================================================
//  Web portal HTML (English)
// ============================================================================
const char PAGE_HTML[] PROGMEM = R"HTML(
<!doctype html><meta name=viewport content="width=device-width,initial-scale=1">
<title>Safe Setup Portal</title>
<style>
  body{font-family:sans-serif;margin:20px;max-width:560px}
  button,input{font-size:1rem;padding:.5rem .7rem}
  label{display:block;margin:.4rem 0 .2rem}
  .card{border:1px solid #ddd;border-radius:8px;padding:12px;margin:14px 0}
  .row{display:flex;gap:8px;align-items:center}
</style>

<h2>System Time & Wi-Fi</h2>

<div class=card>
  <h3>Set System Time</h3>
  <p>This only sets the internal clock.</p>
  <button onclick="sendNow()">Use device time</button>
  <span id=resp style="margin-left:10px"></span>
</div>

<div class=card>
  <h3>Save Home Wi-Fi (for automatic hourly NTP sync)</h3>
  <label>SSID</label>
  <input id=ssid placeholder="Wi-Fi name" maxlength=32>
  <label>Password</label>
  <input id=pass type=password placeholder="Wi-Fi password" maxlength=64>
  <div class=row style="margin-top:.6rem">
    <button onclick="saveWifi()">Save</button>
    <span id=wifiResp></span>
  </div>
  <p style="font-size:.9rem;color:#555">Credentials are stored locally on the device.</p>
</div>

<script>
function epochSec(d){return Math.floor(d.getTime()/1000);}
function sendNow(){
  const now=epochSec(new Date());
  fetch('/set?now='+now).then(r=>r.text()).then(t=>{document.getElementById('resp').textContent=t}).catch(e=>alert(e));
}
function saveWifi(){
  const s=document.getElementById('ssid').value.trim();
  const p=document.getElementById('pass').value;
  fetch('/wifi?ssid='+encodeURIComponent(s)+'&pass='+encodeURIComponent(p))
    .then(r=>r.text()).then(t=>{document.getElementById('wifiResp').textContent=t})
    .catch(e=>alert(e));
}
</script>
)HTML";

// ============================================================================
//  Absolute mode helpers (today)
// ============================================================================
void absInitFromNowOrUnlock() {
  time_t now = nowEpoch();
  if (!timeIsSet || now <= 0) { absHour = 22; absMin = 0; return; }
  struct tm tmNow; localtime_r(&now, &tmNow);
  if (unlockAt > now) {
    struct tm tmTgt; time_t tgt=unlockAt; localtime_r(&tgt, &tmTgt);
    absHour = (tmTgt.tm_yday==tmNow.tm_yday)? tmTgt.tm_hour : tmNow.tm_hour;
    absMin  = (tmTgt.tm_yday==tmNow.tm_yday)? tmTgt.tm_min  : tmNow.tm_min;
  } else {
    absHour = tmNow.tm_hour; absMin = tmNow.tm_min;
  }
}

void absClampToToday() {
  time_t now = nowEpoch();
  if (!timeIsSet || now <= 0) {
    if (absHour < 0) absHour = 0;
    if (absHour > 23) absHour = 23;
    if (absMin  < 0) absMin  = 0;
    if (absMin  > 59) absMin  = 59;
    return;
  }
  struct tm tmNow; localtime_r(&now, &tmNow);
  int nowM = tmNow.tm_hour*60 + tmNow.tm_min;
  int tgtM = absHour*60 + absMin;
  if (tgtM < nowM) tgtM = nowM;
  if (tgtM > 23*60 + 59) tgtM = 23*60 + 59;
  absHour = tgtM / 60;
  absMin  = tgtM % 60;
}

time_t absMakeEpochToday() {
  time_t now = nowEpoch(); if (now <= 0) return 0;
  struct tm t; localtime_r(&now,&t);
  t.tm_sec = 0; t.tm_hour = absHour; t.tm_min  = absMin;
  return mktime(&t);
}

void absAdjustMinutes(int delta) {
  int tgtM = absHour*60 + absMin + delta;
  if (tgtM < 0) tgtM = 0;
  if (tgtM > 23*60 + 59) tgtM = 23*60 + 59;
  absHour = tgtM / 60;
  absMin  = tgtM % 60;
  absClampToToday();
}

// ============================================================================
//  Defaults / Persistence
// ============================================================================
void applyDefaultsAndSave(bool keepTotal) {
  unsigned long keepTotalVal = keepTotal ? prefs.getULong("total", 0UL) : 0UL;
  countdownSeconds = 0;
  originalDuration = 0;
  running = false;
  unlockAt = 0;
  lockStartedAt = 0;   // reset
  timeIsSet = false;
  lockMode = MODE_RELATIVE;
  servoPos = 0;
  totalReclaimed = keepTotalVal;
  absHour = 22; absMin = 0;
  prefs.putUInt("magic", CONFIG_MAGIC);
  saveState();
}

void saveState() {
  prefs.putULong("remain", countdownSeconds);
  prefs.putULong("orig", originalDuration);
  prefs.putBool("running", running);
  prefs.putULong("total", totalReclaimed);
  prefs.putUChar("srvpos", (uint8_t)servoPos);

  prefs.putUChar("mode", (uint8_t)lockMode);
  prefs.putULong("unlockAt", (uint32_t)unlockAt);
  prefs.putBool("timeSet", timeIsSet);
  prefs.putUInt("magic", CONFIG_MAGIC);

  // persist absolute-mode start
  prefs.putULong(KEY_LOCKSTART, (uint32_t)lockStartedAt);
}

void loadState() {
  countdownSeconds = prefs.getULong("remain", 0UL);
  originalDuration = prefs.getULong("orig", 0UL);
  running = prefs.getBool("running", false);
  totalReclaimed = prefs.getULong("total", 0UL);
  servoPos = (int)prefs.getUChar("srvpos", 0);

  lockMode = (LockMode)prefs.getUChar("mode", (uint8_t)MODE_RELATIVE);
  unlockAt = (time_t)prefs.getULong("unlockAt", 0UL);
  timeIsSet = prefs.getBool("timeSet", false);

  // restore absolute-mode start
  lockStartedAt = (time_t)prefs.getULong(KEY_LOCKSTART, 0UL);
}

// ============================================================================
//  Time helpers
// ============================================================================
void setSystemEpoch(time_t t) {
  struct timeval tv = { t, 0 };
  settimeofday(&tv, nullptr);
  timeIsSet = (t > 1600000000L);
}

time_t nowEpoch() { return time(nullptr); }

// ============================================================================
//  Wi-Fi / NTP
// ============================================================================
void loadWifiCreds() {
  String s = prefs.getString(KEY_WIFI_SSID, "");
  String p = prefs.getString(KEY_WIFI_PASS, "");
  memset(wifiSsid, 0, sizeof(wifiSsid));
  memset(wifiPass, 0, sizeof(wifiPass));
  s.toCharArray(wifiSsid, sizeof(wifiSsid));
  p.toCharArray(wifiPass, sizeof(wifiPass));
  lastNtpSyncYday  = (int)prefs.getInt(KEY_NTP_YDAY, -1);
  lastNtpSyncEpoch = (time_t)prefs.getULong(KEY_NTP_EPOCH, 0UL);
}

void saveWifiCreds() {
  prefs.putString(KEY_WIFI_SSID, wifiSsid);
  prefs.putString(KEY_WIFI_PASS, wifiPass);
}

bool connectWifiSTA(uint32_t timeoutMs) {
  if (wifiSsid[0] == '\0') return false;
  WiFi.mode(WIFI_STA);
  WiFi.begin(wifiSsid, wifiPass);
  uint32_t start = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - start < timeoutMs) {
    delay(100);
  }
  return WiFi.status() == WL_CONNECTED;
}

bool syncTimeViaNtp(uint32_t waitMs) {
  // Timezone-safe NTP
  configTzTime(TZ_STR, NTP_SERVER1);

  uint32_t start = millis();
  time_t now = 0;
  do {
    delay(200);
    now = time(nullptr);
  } while ((now < 1600000000L) && (millis() - start < waitMs));

  if (now >= 1600000000L) {
    // Ensure TZ stays applied
    setenv("TZ", TZ_STR, 1);
    tzset();

    struct timeval tv = { now, 0 };
    settimeofday(&tv, nullptr);
    timeIsSet = true;
    lastNtpSyncEpoch = now;
    prefs.putULong(KEY_NTP_EPOCH, (uint32_t)lastNtpSyncEpoch);
    return true;
  }
  return false;
}

void disconnectWifi() {
  if (WiFi.getMode() != WIFI_OFF) {
    WiFi.disconnect(true);
    WiFi.mode(WIFI_OFF);
  }
}

// Hourly auto NTP sync (first 5 minutes each hour)
void maybeHourlyNtpSync() {
  static unsigned long lastCheckMs = 0;
  if (millis() - lastCheckMs < 30000UL) return;  // throttle our own CPU usage
  lastCheckMs = millis();

  // If you want to allow sync while locked, remove "|| running" below.
  if (portalActive /*|| running*/) return; 
  if (wifiSsid[0] == '\0') return;

  time_t now = nowEpoch();
  if (now <= 0) return;

  // Try if it's been an hour since last success
  if (lastNtpSyncEpoch > 0 && (now - lastNtpSyncEpoch) < 3600) return;

  // Bring up Wi-Fi briefly, sync, then go back down
  if (connectWifiSTA(8000)) {
    if (syncTimeViaNtp(8000)) {
      // Update "last sync" markers
      struct tm tmNow; localtime_r(&now, &tmNow);
      lastNtpSyncEpoch = now;
      lastNtpSyncYday = tmNow.tm_yday;
      prefs.putULong(KEY_NTP_EPOCH, (uint32_t)lastNtpSyncEpoch);
      prefs.putInt(KEY_NTP_YDAY, lastNtpSyncYday);
      saveState();
    }
    disconnectWifi();
  }
}

// ============================================================================
//  AP / Web
// ============================================================================
void startTimePortal() {
  if (portalActive) return;
  WiFi.mode(WIFI_AP);
  WiFi.softAP(AP_SSID, AP_PASS);
  delay(200);
  server.on("/", handleRoot);
  server.on("/set", handleSet);
  server.on("/wifi", handleWifi);
  server.begin();
  portalActive = true;
  portalStartedMs = millis();
  displayIsOn = true; oledDisplayOn();
}
void stopTimePortal() {
  if (!portalActive) return;
  server.stop();
  WiFi.softAPdisconnect(true);
  WiFi.mode(WIFI_OFF);
  portalActive = false;
}
void handleRoot() { server.send(200, "text/html", PAGE_HTML); }
void handleSet() {
  time_t now = server.hasArg("now") ? (time_t)server.arg("now").toInt() : 0;
  if (now > 1600000000L) {
    setSystemEpoch(now);
    prefs.putULong(KEY_NTP_EPOCH, (uint32_t)now);
    server.send(200, "text/plain", "System time set!");
  } else {
    server.send(400, "text/plain", "Invalid time");
  }
}
void handleWifi() {
  String ssid = server.hasArg("ssid") ? server.arg("ssid") : "";
  String pass = server.hasArg("pass") ? server.arg("pass") : "";
  ssid.trim();
  if (ssid.length() == 0) { server.send(400, "text/plain", "SSID missing"); return; }
  if (ssid.length() > 32 || pass.length() > 64) { server.send(400, "text/plain", "Too long"); return; }
  memset(wifiSsid, 0, sizeof(wifiSsid));
  memset(wifiPass, 0, sizeof(wifiPass));
  ssid.toCharArray(wifiSsid, sizeof(wifiSsid));
  pass.toCharArray(wifiPass, sizeof(wifiPass));
  saveWifiCreds();
  server.send(200, "text/plain", "Wi-Fi saved");
}

// ============================================================================
//  OLED helpers & pages
// ============================================================================
void clearRows(uint8_t row_from, uint8_t rows) {
  oled.set1X();
  for (uint8_t r = 0; r < rows; r++) {
    oled.setCursor(0, row_from + r);
    oled.print("                              ");
  }
}
void print2XBlock(uint8_t rowTop, uint8_t col, const char* txt) {
  clearRows(rowTop, 2);
  oled.set2X();
  oled.setCursor(col, rowTop);
  oled.print(txt);
  oled.set1X();
}

void displayChargingLine() {
  clearRows(7, 1);
  oled.set1X();
  oled.setCursor(0, 7);

  if (isChargingNow) {
    oled.printf("Charging", lastVinDisplay);
  } 
}

void displayMainHeader(int pct) {
  clearRows(1, 1);
  oled.set1X(); oled.setCursor(0,1);
  if (lockMode==MODE_RELATIVE) {
    oled.printf("%d%% | Timer", pct);
  } else {
    oled.printf("%d%% | Opens at...", pct);
  }
}
void displayTimeRel(bool forceUpdate) {
  unsigned long seconds = countdownSeconds;
  if (seconds > MAX_SECONDS) seconds = MAX_SECONDS;
  unsigned int h = seconds / 3600UL; seconds %= 3600UL;
  unsigned int m = seconds / 60UL;   unsigned int s = seconds % 60UL;
  char buf[9]; snprintf(buf, sizeof(buf), "%02u:%02u:%02u", h, m, s);
  if (forceUpdate || strcmp(buf, lastDisplay) != 0) {
    print2XBlock(3, COL_CENTER_REL, buf);
    strcpy(lastDisplay, buf);
  }
  clearRows(5,1);
  displayChargingLine();
}
void displayTimeAbs(bool force) {
  static int lastHr=-1, lastMin=-1; static bool lastRun=false;
  if (force || lastHr!=absHour || lastMin!=absMin || lastRun!=running) {
    char value[16]; snprintf(value,sizeof(value),"%02d:%02d", absHour, absMin);
    print2XBlock(3, COL_CENTER_ABS, value);
    lastHr=absHour; lastMin=absMin; lastRun=running;
  }
  clearRows(5,1);
  displayChargingLine();
}


void showStatsPage() {
  oled.clear();
  
  // use smaller font (4x6)
  oled.setFont(System5x7);   // default small readable font
  oled.set1X();
  
  oled.setCursor(0, 0);
  oled.println("Stats");
  oled.setCursor(0, 2);
  oled.println("Total time saved:");

  unsigned long total = totalReclaimed;
  unsigned long days  = total / 86400UL;
  unsigned long rem   = total % 86400UL;
  unsigned int  hours = rem / 3600UL;
  unsigned int  mins  = (rem % 3600UL) / 60UL;

  // draw in smaller size (1X) — no double-size here
  oled.setCursor(0, 4);
  oled.printf("%03lu days", days);
  oled.setCursor(0, 5);
  oled.printf("%02u hours", hours);
  oled.setCursor(0, 6);
  oled.printf("%02u mins", mins);

  // Always show VIN/charging line on bottom
  displayChargingLine();

  // Restore normal font for next pages
  oled.setFont(Adafruit5x7);
}
void showAboutPage() {
  oled.clear();
  oled.set1X();
  oled.setCursor(0,0); oled.println("About");
  oled.setCursor(0,2); oled.println("Smartphone Safe");
  oled.setCursor(0,3); oled.println("V1 by LeoHD");
}
void showModePage() {
  oled.clear();
  oled.set1X();
  oled.setCursor(0,0); oled.println("Mode");
  oled.setCursor(0,2); oled.print("Current: ");
  oled.print(lockMode==MODE_RELATIVE?"Relative":"Absolute");
}
void showClockPage() {
  oled.clear();
  oled.set1X();
  oled.setCursor(0,0); oled.println("Clock");
  if (!timeIsSet) {
    oled.setCursor(0,2); oled.println("Time not set!");
    oled.setCursor(0,4); oled.println("START: +1h");
    return;
  }
  time_t now = nowEpoch();
  struct tm tm_; localtime_r(&now,&tm_);
  char clk[16]; snprintf(clk,sizeof(clk),"%02d:%02d:%02d", tm_.tm_hour, tm_.tm_min, tm_.tm_sec);
  oled.setCursor(0,2); oled.println("Current:");
  print2XBlock(2, 0, clk);

  // Last sync display (HH:MM only)
  oled.set1X();
  if (lastNtpSyncEpoch > 0) {
    struct tm tms; localtime_r(&lastNtpSyncEpoch, &tms);
    char lastbuf[24];
    snprintf(lastbuf, sizeof(lastbuf), "Last sync: %02d:%02d", tms.tm_hour, tms.tm_min);
    oled.setCursor(0,5); oled.println(lastbuf);
  } else {
    oled.setCursor(0,5); oled.println("Last sync: never");
  }

  oled.setCursor(0,6); oled.println("STAR: Sync now!");
}
void showPortalPage() {
  oled.clear();
  oled.set1X();
  oled.setCursor(0,0); oled.println("Setup via AP:");
  if (portalActive) {
    oled.setCursor(0,2); oled.println("AP: SafeSetup");
    oled.setCursor(0,3); oled.println("Pass: 12345678");
    oled.setCursor(0,4); oled.println("URL: 192.168.4.1");
  } else {
    oled.setCursor(0,2); oled.println("AP off");
    oled.setCursor(0,3); oled.println("Press START");
    oled.setCursor(0,4); oled.println("to activate!");
  }
}
void renderMenuPage() {
  fastIncrementUp = false; fastIncrementDown = false;
  if (menuIndex == 0) showStatsPage();
  else if (menuIndex == 1) showAboutPage();
  else if (menuIndex == 2) showModePage();
  else if (menuIndex == 3) showClockPage();
  else showPortalPage();
}
void showMainPage() {
  oled.clear();
  displayMainHeader(lastBattPct);
  if (lockMode==MODE_RELATIVE) {
    strcpy(lastDisplay, "");
    displayTimeRel(true);
  } else {
    absInitFromNowOrUnlock(); absClampToToday();
    displayTimeAbs(true);
  }
}

// ============================================================================
//  Power / Sleep
// ============================================================================
void noteInteraction() {
  lastInteractionMs = millis();
  if (!displayIsOn) {
    oledDisplayOn(); displayIsOn = true;
    if (!showMenu) showMainPage(); else renderMenuPage();
  }
}
void oledDisplayOn() { oled.ssd1306WriteCmd(0xAF); }
void oledDisplayOff() { oled.ssd1306WriteCmd(0xAE); }
void setupLightSleepWakePins() {
  esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_ALL);
  gpio_wakeup_enable((gpio_num_t)BTN_UP,    GPIO_INTR_LOW_LEVEL);
  gpio_wakeup_enable((gpio_num_t)BTN_DOWN,  GPIO_INTR_LOW_LEVEL);
  gpio_wakeup_enable((gpio_num_t)BTN_RESET, GPIO_INTR_LOW_LEVEL);
  gpio_wakeup_enable((gpio_num_t)BTN_START, GPIO_INTR_LOW_LEVEL);
  esp_sleep_enable_gpio_wakeup();
}
void maybeEnterLightSleep() {
  if (millis() - lastInteractionMs >= INACTIVITY_MS) {
    oledDisplayOff(); 
    displayIsOn = false;
    saveState();
    if (myServo.attached()) myServo.detach();

    // Wake up periodically to allow hourly NTP sync checks
    esp_sleep_enable_timer_wakeup(10ULL * 60ULL * 1000000ULL); // wake every 10 minutes
    // (GPIO wake is already enabled elsewhere)

    esp_light_sleep_start();

    oledDisplayOn(); 
    displayIsOn = true;
    lastInteractionMs = millis();
    if (!showMenu) showMainPage(); else renderMenuPage();
  }
}

// ============================================================================
//  Servo
// ============================================================================
void lockServo() { myServo.attach(SERVO_PIN, 500, 2500); myServo.write(90); delay(200); myServo.detach(); servoPos = 90; }
void unlockServo(){ myServo.attach(SERVO_PIN, 500, 2500); myServo.write(0);  delay(200); myServo.detach(); servoPos = 0; }

// ============================================================================
//  Measurements
// ============================================================================
float readBatteryVoltage() {
  analogSetPinAttenuation(BAT_ADC, ADC_11db);
  delay(5);
  (void)analogReadMilliVolts(BAT_ADC);
  uint32_t sum_mV = 0;
  for (int i = 0; i < BATT_SAMPLES; i++) { sum_mV += analogReadMilliVolts(BAT_ADC); delay(2); }
  float v_adc = (sum_mV / (float)BATT_SAMPLES) / 1000.0f;
  return v_adc * VOLT_DIV * CAL;
}
float readVinVoltage() {
  analogSetPinAttenuation(VIN_ADC, ADC_11db);
  delay(5);
  (void)analogReadMilliVolts(VIN_ADC);
  uint32_t sum_mV = 0;
  for (int i = 0; i < VIN_SAMPLES; i++) { sum_mV += analogReadMilliVolts(VIN_ADC); delay(2); }
  float v_adc = (sum_mV / (float)VIN_SAMPLES) / 1000.0f;
  return v_adc * VOLT_DIV_VIN * CAL_VIN;
}
int liionPercentFromVoltage(float v) {
  if (v >= 4.20f) return 100;
  if (v <= 3.40f) return 0;
  struct VP { float v; int p; };
  static const VP curve[] = {
    {4.20f,100},{4.10f,90},{4.00f,80},{3.95f,70},
    {3.90f,60},{3.85f,50},{3.80f,40},{3.75f,30},
    {3.70f,20},{3.60f,10},{3.50f,5},{3.40f,0}
  };
  for (int i = 0; i < (int)(sizeof(curve)/sizeof(curve[0]))-1; i++) {
    if (v <= curve[i].v && v >= curve[i+1].v) {
      float t = (v - curve[i+1].v) / (curve[i].v - curve[i+1].v);
      return (int)lroundf(curve[i+1].p + t * (curve[i].p - curve[i+1].p));
    }
  }
  int approx = (int)lroundf((v - 3.40f) * (100.0f / (4.20f - 3.40f)));
  if (approx < 0) approx = 0; if (approx > 100) approx = 100;
  return approx;
}

// ============================================================================
//  Setup
// ============================================================================
void setup() {
  // External 32 kHz crystal
  enableExternal32K();

  // I2C / OLED
  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(400000);
  oled.begin(&Adafruit128x64, I2C_ADDRESS);
  oled.setFont(Adafruit5x7);
  oled.clear();

  // Splash
  oled.setCursor(0, 0); oled.println("Reclaim");
  oled.setCursor(0, 3); oled.println("YOUR");
  oled.setCursor(0, 6); oled.println("time!");
  delay(800);

  if (!ext32k_ok) {
    oled.clear();
    oled.setCursor(0, 2);
    oled.println("⚠ 32kHz crystal FAIL");
    oled.setCursor(0, 4);
    oled.println("Using internal RC");
    delay(2000);
  }
  oled.clear();

  // Buttons
  pinMode(BTN_UP,    INPUT_PULLUP);
  pinMode(BTN_DOWN,  INPUT_PULLUP);
  pinMode(BTN_RESET, INPUT_PULLUP);
  pinMode(BTN_START, INPUT_PULLUP);

  // Servo default
  servoPos = 0;

  // Preferences
  prefs.begin("timer", false);

  // Factory reset if RESET held at boot
  if (digitalRead(BTN_RESET) == LOW) {
    prefs.clear();
    applyDefaultsAndSave(false);
  }

  loadState();
  uint32_t magic = prefs.getUInt("magic", 0);
  if (magic != CONFIG_MAGIC) applyDefaultsAndSave(true);

  // Wi-Fi creds & last sync
  loadWifiCreds();

  // Timezone
  setenv("TZ", TZ_STR, 1);
  tzset();

  // One-time NTP at boot if time not set and Wi-Fi known
  if (!timeIsSet && wifiSsid[0] != '\0' && !portalActive) {
    if (connectWifiSTA(8000)) {
      if (syncTimeViaNtp(8000)) saveState();
      disconnectWifi();
    }
  }

  lastInteractionMs = millis();
  displayIsOn = true;
  absInitFromNowOrUnlock();
  absClampToToday();

  // Button handlers
  btnUp.attachClick([](){
    noteInteraction();
    if (showMenu) { menuIndex = (menuIndex + 1) % MENU_COUNT; renderMenuPage(); return; }
    if (running) return;
    if (lockMode==MODE_ABSOLUTE) { absAdjustMinutes(+1); displayTimeAbs(true); return; }
    if (countdownSeconds + 60 <= MAX_SECONDS) { countdownSeconds += 60; displayTimeRel(true); saveState(); }
  });
  btnUp.attachLongPressStart([](){ if (running || showMenu) return; noteInteraction(); fastIncrementUp = true; lastIncrementUp = millis(); });
  btnUp.attachDuringLongPress([](){
    if (!fastIncrementUp || running || showMenu) return; noteInteraction();
    static unsigned long interval = 150;
    if (millis() - lastIncrementUp >= interval) {
      if (lockMode==MODE_ABSOLUTE) { absAdjustMinutes(+1); displayTimeAbs(true); }
      else { if (countdownSeconds + 60 <= MAX_SECONDS) { countdownSeconds += 60; displayTimeRel(true); saveState(); } }
      lastIncrementUp = millis(); if (interval > 50) interval -= 10;
    }
  });
  btnUp.attachLongPressStop([](){ fastIncrementUp = false; });

  btnDown.attachClick([](){
    noteInteraction();
    if (showMenu) { menuIndex = (menuIndex - 1 + MENU_COUNT) % MENU_COUNT; renderMenuPage(); return; }
    if (running) return;
    if (lockMode==MODE_ABSOLUTE) { absAdjustMinutes(-1); displayTimeAbs(true); return; }
    if (countdownSeconds >= 60) { countdownSeconds -= 60; displayTimeRel(true); saveState(); }
  });
  btnDown.attachLongPressStart([](){ if (running || showMenu) return; noteInteraction(); fastIncrementDown = true; lastIncrementDown = millis(); });
  btnDown.attachDuringLongPress([](){
    if (!fastIncrementDown || running || showMenu) return; noteInteraction();
    static unsigned long interval = 150;
    if (millis() - lastIncrementDown >= interval) {
      if (lockMode==MODE_ABSOLUTE) { absAdjustMinutes(-1); displayTimeAbs(true); }
      else { if (countdownSeconds >= 60) { countdownSeconds -= 60; displayTimeRel(true); saveState(); } }
      lastIncrementDown = millis(); if (interval > 50) interval -= 10;
    }
  });
  btnDown.attachLongPressStop([](){ fastIncrementDown = false; });

  btnReset.attachClick([](){
    noteInteraction();
    if (running) return;
    if (showMenu) { showMenu = false; showMainPage(); }
    else { showMenu = true; menuIndex = 0; renderMenuPage(); }
  });

  btnStart.attachClick([](){
    noteInteraction();
    if (showMenu) {
      if (menuIndex == 2) { // Mode toggle
        lockMode = (lockMode == MODE_RELATIVE) ? MODE_ABSOLUTE : MODE_RELATIVE;
        if (lockMode==MODE_ABSOLUTE) { absInitFromNowOrUnlock(); absClampToToday(); }
        saveState(); renderMenuPage(); return;
      }
      if (menuIndex == 3) { // Clock menu: +1h if not set OR manual NTP sync
        if (!timeIsSet) {
          time_t now = nowEpoch(); now += 3600;
          setSystemEpoch(now);
          renderMenuPage();
          return;
        }
        oled.clear();
        oled.setCursor(0,3); oled.println("Trying NTP sync...");
        oled.setCursor(0,5); oled.println("(needs Wi-Fi)");
        bool success = false;
        if (wifiSsid[0] != '\0') {
          if (connectWifiSTA(8000)) {
            success = syncTimeViaNtp(8000);
            disconnectWifi();
          }
        }
        delay(500);
        oled.clear();
        oled.setCursor(0,3);
        oled.println(success ? "NTP sync OK!" : "Sync failed.");
        if (success) saveState();
        delay(1500);
        renderMenuPage();
        return;
      }
      if (menuIndex == 4) { // Setup portal toggle
        if (!portalActive) startTimePortal(); else stopTimePortal();
        renderMenuPage(); return;
      }
      return;
    }

    if (running) return;

    if (lockMode==MODE_ABSOLUTE) {
      if (!timeIsSet) { return; }
      absClampToToday();
      time_t tgt = absMakeEpochToday();
      time_t now = nowEpoch();
      if (tgt > now) {
        unlockAt = tgt;
        lockStartedAt = now;   // record when we started locking (absolute mode)
        running = true;
        lastUpdate = millis();
        lockServo();
        saveState();
      }
      showMainPage();
      return;
    }

    // Relative mode
    if (countdownSeconds > 0) {
      running = true; lastUpdate = millis();
      if (originalDuration == 0) originalDuration = countdownSeconds;
      lockServo(); saveState();
    }
  });

  // First readings
  float vbat = readBatteryVoltage();
  int pct = liionPercentFromVoltage(vbat);
  lastBattV = vbat; lastBattPct = pct;
  float vin = readVinVoltage();
  lastVin = vin;

// Initialize charging state using interval
isChargingNow = (vin >= VIN_CHARGE_MIN && vin <= VIN_CHARGE_MAX);
lastChargingShown = !isChargingNow;  // force initial OLED update

  showMainPage();
  if (running) lockServo();
  setupLightSleepWakePins();
}

// ============================================================================
//  Loop
// ============================================================================
void loop() {
  btnUp.tick(); btnDown.tick(); btnReset.tick(); btnStart.tick();

  if (digitalRead(BTN_UP)==LOW || digitalRead(BTN_DOWN)==LOW ||
      digitalRead(BTN_RESET)==LOW || digitalRead(BTN_START)==LOW) {
    noteInteraction();
  }

  if (portalActive) {
    server.handleClient();
    if (millis() - portalStartedMs > PORTAL_TIMEOUT_MS) {
      stopTimePortal();
      if (showMenu) renderMenuPage(); else showMainPage();
    }
  }

  // Relative mode
  if (!showMenu && running && lockMode == MODE_RELATIVE) {
    if (millis() - lastUpdate >= 1000UL) {
      lastUpdate += 1000UL;
      if (countdownSeconds > 0) {
        countdownSeconds--;
        if (displayIsOn) displayTimeRel(false);
        saveState();
      } else {
        running = false;
        if (originalDuration > 0) { totalReclaimed += originalDuration; originalDuration = 0; }
        unlockServo();
        if (displayIsOn) displayTimeRel(true);
        saveState();
      }
    }
  }

  // Absolute mode
  if (!showMenu && lockMode == MODE_ABSOLUTE) {
    static unsigned long lastTick = 0;
    if (millis() - lastTick >= 250UL) {
      lastTick += 250UL;
      if (!running) absClampToToday();
      if (displayIsOn) displayTimeAbs(false);
    }
    if (running) {
      time_t now = nowEpoch();
      if (timeIsSet && unlockAt > 0 && now >= unlockAt) {
        running = false;
        unlockServo();

        // Add absolute-mode locked duration to stats
        if (lockStartedAt > 0 && unlockAt > lockStartedAt) {
          unsigned long duration = (unsigned long)(unlockAt - lockStartedAt);
          totalReclaimed += duration;
          lockStartedAt = 0;
          saveState();
        } else {
          saveState();
        }

        displayTimeAbs(true);
      }
    }
  }

  // Battery update
  if (!showMenu && displayIsOn && millis() - lastBattMs >= BATT_INTERVAL_MS) {
    lastBattMs = millis();
    float vbat = readBatteryVoltage();
    int pct = liionPercentFromVoltage(vbat);
    if (lastBattPct != pct) { lastBattV = vbat; lastBattPct = pct; displayMainHeader(pct); }
  }

  // VIN + charging hysteresis
if (!showMenu && displayIsOn && millis() - lastVinMs >= VIN_INTERVAL_MS) {
  lastVinMs = millis();
  float vin = readVinVoltage();
  lastVin = vin;
  lastVinDisplay = vin;   // ✅ store for display

  bool chargingNow = (vin >= VIN_CHARGE_MIN && vin <= VIN_CHARGE_MAX);

  if (chargingNow != isChargingNow) {
    isChargingNow = chargingNow;
    displayChargingLine();      // refresh OLED line 7
    lastChargingShown = isChargingNow;
  }
}

  // Display off after inactivity
  if (displayIsOn && (millis() - lastInteractionMs >= INACTIVITY_MS)) {
    oledDisplayOff(); displayIsOn = false;
  }

  // Light sleep when idle and no portal
  if (!running && !portalActive) {
    maybeEnterLightSleep();
  }

  // Hourly NTP sync attempt
  maybeHourlyNtpSync();
}