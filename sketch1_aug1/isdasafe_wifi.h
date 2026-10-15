// IsdaSafe WiFi setup: the WiFi network is chosen at run time and saved in
// flash (survives reboots and deep sleep) instead of being compiled in.
//
// Two ways to choose a network:
//   - Setup hotspot: the device opens its own WiFi network "IsdaSafe-XXXX"
//     (password WIFI_SETUP_AP_PASSWORD). Join it from a phone; a setup page
//     pops up (or browse to http://192.168.4.1) listing the nearby networks
//     -- tap one, type its password, Save. The hotspot opens automatically
//     when no saved network works, right after a successful PAIR, when the
//     BOOT button is held for 3 s, on `WIFI SETUP`, or after the device has
//     been offline for WIFI_OFFLINE_SETUP_MS.
//   - USB serial, so the website can offer the same choice right after
//     PAIRED OK:
//       WIFI SCAN            -> WIFI NET <rssi> <open|secured> <ssid>  (one line per network, strongest first)
//                               WIFI SCAN DONE <count>
//       WIFI SSID <name>     -> WIFI SSID OK     (rest of the line; spaces allowed)
//       WIFI PASS <password> -> WIFI PASS OK     (send just `WIFI PASS` for an open network)
//       WIFI CONNECT         -> WIFI CONNECTING, then WIFI OK <ip> | WIFI FAIL <reason>
//       WIFI STATUS          -> WIFI STATUS CONNECTED <ip> <rssi> <ssid> | WIFI STATUS DISCONNECTED <saved ssid or ->
//       WIFI SETUP           -> WIFI SETUP <hotspot name>   (opens the setup hotspot)
//       WIFI FORGET          -> WIFI FORGET OK   (erases the saved network, opens the hotspot)
//
// A network is only saved after the device has actually connected to it, so
// a mistyped password never replaces a network that works.
//
// Connecting:
//   - A failed attempt says why (wrong password, network not found, signal
//     too weak, ...) from the router's disconnect reason, and gives up as
//     soon as that's clear instead of always waiting WIFI_CONNECT_TIMEOUT_MS.
//   - While offline, the saved network is retried every WIFI_RETRY_MS --
//     except while a phone is on the setup hotspot: each attempt can move
//     the radio to another channel, which drops the phone off the hotspot
//     mid-setup. For the same reason the hotspot opens on the saved (or
//     strongest) network's channel.
//   - Once connected, the device checks in with the server right away
//     (queueCheckIn(), which also uploads readings kept while offline), so
//     the app sees it's online without waiting for the next reading.
//
// The sketch calls wifiBegin() in setup() and wifiLoop() from every wait.

#pragma once

#include <Arduino.h>
#include <DNSServer.h>
#include <Preferences.h>
#include <WebServer.h>
#include <WiFi.h>
#include "isdasafe_pairing.h"

#ifndef WIFI_SETUP_AP_PASSWORD
#define WIFI_SETUP_AP_PASSWORD  "isdasafe"   // hotspot password, 8+ characters
#endif
#ifndef WIFI_SETUP_BUTTON_PIN
#define WIFI_SETUP_BUTTON_PIN   0            // BOOT button; -1 = no button
#endif
#define WIFI_SETUP_BUTTON_MS    3000         // hold this long to open the hotspot
#define WIFI_SETUP_TIMEOUT_MS   (10UL * 60UL * 1000UL)  // hotspot closes after this, but only while connected
#define WIFI_SETUP_CLOSE_MS     60000        // after a successful connect, keep the page up this long
#define WIFI_CONNECT_TIMEOUT_MS 20000
#define WIFI_RETRY_MS           30000        // offline: retry the saved network this often
#define WIFI_CHECKIN_DELAY_MS   3000         // connected: check in with the server after this
#define WIFI_OFFLINE_SETUP_MS   (2UL * 60UL * 1000UL)   // offline this long -> hotspot opens
#define WIFI_MAX_NETS           30
#define WIFI_SCAN_MS_PER_CHANNEL 400         // active scan time per channel (library default 300)

// Defined in isdasafe_queue.h: tells the server the device is online (and on
// which network), uploading any readings kept while offline.
void queueCheckIn();

static Preferences wifiPrefs;
static String wifiSsid, wifiPass;              // saved network
static String wifiNewSsid, wifiNewPass;        // network being tried / staged over serial

enum WifiTryState { WIFI_TRY_IDLE, WIFI_TRY_RUNNING, WIFI_TRY_OK, WIFI_TRY_FAIL };
static WifiTryState wifiTry = WIFI_TRY_IDLE;
static unsigned long wifiTryStartMs = 0;
static String wifiTryReason;

// Written by the WiFi event task: the last disconnect reason and how many
// disconnects there have been (a failed attempt is one disconnect).
static volatile uint8_t wifiDiscReason = 0;
static volatile uint16_t wifiDiscCount = 0;
static uint16_t wifiTryDiscStart = 0;          // wifiDiscCount when the try started
static uint16_t wifiTryDiscSeen = 0;           // failures of the try already acted on
static uint16_t wifiSavedDiscStart = 0;        // wifiDiscCount when the saved network was last tried

static unsigned long wifiLastBeginMs = 0;      // last attempt at the saved network
static unsigned long wifiCheckInAtMs = 0;      // 0 = no check-in scheduled

static bool wifiSetupOn = false;
static unsigned long wifiSetupStartMs = 0;
static unsigned long wifiSetupCloseAtMs = 0;   // 0 = not scheduled
static String wifiApName;
static WebServer wifiServer(80);
static DNSServer wifiDns;
static bool wifiRoutesAdded = false;

struct WifiNet { String ssid; int32_t rssi; bool open; uint8_t channel; };
static WifiNet wifiNets[WIFI_MAX_NETS];
static int wifiNetCount = 0;

inline bool wifiSetupActive() { return wifiSetupOn; }
inline bool wifiTrying() { return wifiTry == WIFI_TRY_RUNNING; }
inline const String &wifiSetupApName() { return wifiApName; }
inline const String &wifiSavedSsid() { return wifiSsid; }

// A phone (or laptop) is on the setup hotspot.
static bool wifiSetupInUse() { return wifiSetupOn && WiFi.softAPgetStationNum() > 0; }

static void wifiConnectSaved() {
  if (wifiSsid.length() == 0) return;
  wifiLastBeginMs = millis();
  wifiSavedDiscStart = wifiDiscCount;
  WiFi.begin(wifiSsid.c_str(), wifiPass.length() ? wifiPass.c_str() : nullptr);
}

static void wifiOnDisconnected(arduino_event_id_t, arduino_event_info_t info) {
  uint8_t reason = info.wifi_sta_disconnected.reason;
  if (reason == WIFI_REASON_ASSOC_LEAVE) return;   // we disconnected on purpose
  wifiDiscReason = reason;
  wifiDiscCount = wifiDiscCount + 1;   // only this task writes it
}

enum WifiFailKind { WIFI_FAIL_OTHER, WIFI_FAIL_PASSWORD, WIFI_FAIL_NOT_FOUND, WIFI_FAIL_SECURITY, WIFI_FAIL_WEAK };

static WifiFailKind wifiFailKind(uint8_t reason) {
  switch (reason) {
    case WIFI_REASON_AUTH_FAIL:
    case WIFI_REASON_4WAY_HANDSHAKE_TIMEOUT:
    case WIFI_REASON_HANDSHAKE_TIMEOUT:
    case WIFI_REASON_MIC_FAILURE:
      return WIFI_FAIL_PASSWORD;
    case WIFI_REASON_NO_AP_FOUND:
      return WIFI_FAIL_NOT_FOUND;
    case WIFI_REASON_NO_AP_FOUND_W_COMPATIBLE_SECURITY:
    case WIFI_REASON_NO_AP_FOUND_IN_AUTHMODE_THRESHOLD:
      return WIFI_FAIL_SECURITY;
    case WIFI_REASON_NO_AP_FOUND_IN_RSSI_THRESHOLD:
    case WIFI_REASON_BEACON_TIMEOUT:
      return WIFI_FAIL_WEAK;
    default:
      return WIFI_FAIL_OTHER;
  }
}

// Shown to the owner (the app translates these exact texts).
static const char *wifiFailText(WifiFailKind kind) {
  switch (kind) {
    case WIFI_FAIL_PASSWORD:  return "wrong password";
    case WIFI_FAIL_NOT_FOUND: return "network not found";
    case WIFI_FAIL_SECURITY:  return "its security type isn't supported";
    case WIFI_FAIL_WEAK:      return "signal too weak";
    default:                  return "wrong password or weak signal";
  }
}

// Adds the results of the last scan to wifiNets: duplicates merged; when
// it's full, a stronger network replaces the weakest.
static void wifiMergeScan(int n) {
  for (int i = 0; i < n; i++) {
    String ssid = WiFi.SSID(i);
    if (ssid.length() == 0) continue;
    int32_t rssi = WiFi.RSSI(i);
    bool open = WiFi.encryptionType(i) == WIFI_AUTH_OPEN;
    uint8_t channel = WiFi.channel(i);
    int j = 0;
    while (j < wifiNetCount && wifiNets[j].ssid != ssid) j++;
    if (j < wifiNetCount) {
      if (rssi > wifiNets[j].rssi) {
        wifiNets[j].rssi = rssi;
        wifiNets[j].channel = channel;
      }
    } else if (wifiNetCount < WIFI_MAX_NETS) {
      wifiNets[wifiNetCount++] = { ssid, rssi, open, channel };
    } else {
      int weakest = 0;
      for (int k = 1; k < wifiNetCount; k++) if (wifiNets[k].rssi < wifiNets[weakest].rssi) weakest = k;
      if (rssi > wifiNets[weakest].rssi) wifiNets[weakest] = { ssid, rssi, open, channel };
    }
  }
  WiFi.scanDelete();
}

// Scans nearby networks into wifiNets (hidden ones skipped, duplicates
// merged, strongest first). Takes ~5-8 s.
//
// While the device is offline it keeps retrying the saved network, and a
// scan started during a connection attempt fails or comes back with only a
// few networks. So those retries are paused for the scan, and it scans
// twice, longer on each channel, because a single pass often misses
// networks that answer slowly.
static int wifiScan() {
  WiFi.enableSTA(true);
  bool paused = WiFi.status() != WL_CONNECTED && !wifiTrying();
  if (paused) {
    WiFi.disconnect();
    delay(100);
  }
  wifiNetCount = 0;
  for (int pass = 0; pass < 2; pass++) {
    int n = WiFi.scanNetworks(false, false, false, WIFI_SCAN_MS_PER_CHANNEL);
    if (n < 0) {   // radio busy: wait a moment and try once more
      delay(500);
      n = WiFi.scanNetworks(false, false, false, WIFI_SCAN_MS_PER_CHANNEL);
    }
    if (n > 0) wifiMergeScan(n);
    else WiFi.scanDelete();
  }
  if (paused && !wifiSetupInUse()) wifiConnectSaved();
  for (int i = 1; i < wifiNetCount; i++) {   // insertion sort, strongest first
    WifiNet n2 = wifiNets[i];
    int j = i - 1;
    while (j >= 0 && wifiNets[j].rssi < n2.rssi) { wifiNets[j + 1] = wifiNets[j]; j--; }
    wifiNets[j + 1] = n2;
  }
  return wifiNetCount;
}

static int wifiFindNet(const String &ssid) {
  for (int i = 0; i < wifiNetCount; i++) if (wifiNets[i].ssid == ssid) return i;
  return -1;
}

// Channel for the setup hotspot while offline: the saved network's (where
// the device will most likely connect) -- from the last scan, else the one
// it was on when last connected -- else the strongest network's. The hotspot
// has to move to whatever channel the device connects on, which drops the
// phone, so starting on the right one avoids that. No scan here: the
// hotspot should appear at once.
static uint8_t wifiSetupChannel() {
  int i = wifiFindNet(wifiSsid);
  if (i >= 0) return wifiNets[i].channel;
  uint8_t saved = wifiPrefs.getUChar("chan", 0);
  if (saved >= 1 && saved <= 13) return saved;
  return wifiNetCount > 0 ? wifiNets[0].channel : 1;
}

// Remembers the channel of the network the device is on, for
// wifiSetupChannel() after a restart. Written only when it changes.
static void wifiSaveChannel() {
  uint8_t ch = WiFi.channel();
  if (ch >= 1 && ch <= 13 && ch != wifiPrefs.getUChar("chan", 0)) wifiPrefs.putUChar("chan", ch);
}

// The saved network clearly isn't going to work right now (not in range,
// password changed, ...): the router answered the same way twice. Lets the
// boot stop waiting for it and open the setup hotspot straight away.
bool wifiSavedFailed() {
  return WiFi.status() != WL_CONNECTED && (uint16_t)(wifiDiscCount - wifiSavedDiscStart) >= 2 &&
         wifiFailKind(wifiDiscReason) != WIFI_FAIL_OTHER;
}

static void wifiFailTry(const String &reason) {
  wifiTryReason = reason;
  wifiTry = WIFI_TRY_FAIL;
  Serial.printf("WIFI FAIL %s\n", wifiTryReason.c_str());
  Serial.printf("[WiFi] couldn't connect to \"%s\": %s (reason code %u)\n", wifiNewSsid.c_str(),
                wifiTryReason.c_str(), (unsigned)wifiDiscReason);
  WiFi.disconnect();
  wifiConnectSaved();   // go back to the network that was working
}

// Tries a network without saving it; wifiLoop() saves it once connected.
static void wifiStartTry(const String &ssid, const String &pass) {
  wifiNewSsid = ssid;
  wifiNewPass = pass;
  wifiTry = WIFI_TRY_RUNNING;
  wifiTryStartMs = millis();
  wifiTryReason = "";
  wifiTryDiscSeen = 0;
  wifiDiscReason = 0;
  Serial.println("WIFI CONNECTING");
  if (pass.length() > 0 && pass.length() < 8) {
    // WPA passwords are 8-63 characters; the radio wouldn't even try.
    wifiFailTry("the password is too short (WiFi passwords have at least 8 characters)");
    return;
  }
  Serial.printf("[WiFi] trying \"%s\"...\n", ssid.c_str());
  WiFi.enableSTA(true);
  WiFi.disconnect();
  delay(100);
  wifiTryDiscStart = wifiDiscCount;
  WiFi.begin(ssid.c_str(), pass.length() ? pass.c_str() : nullptr);
}

// Follows a running try: saves the network once connected, or gives up when
// the router's answers make the reason clear (or after the timeout).
static void wifiFollowTry() {
  wl_status_t st = WiFi.status();
  if (st == WL_CONNECTED) {
    wifiSsid = wifiNewSsid;
    wifiPass = wifiNewPass;
    wifiPrefs.putString("ssid", wifiSsid);
    wifiPrefs.putString("pass", wifiPass);
    wifiTry = WIFI_TRY_OK;
    Serial.printf("WIFI OK %s\n", WiFi.localIP().toString().c_str());
    Serial.printf("[WiFi] connected to \"%s\" and saved it.\n", wifiSsid.c_str());
    if (wifiSetupOn) wifiSetupCloseAtMs = millis() + WIFI_SETUP_CLOSE_MS;
    return;
  }

  // A failed attempt is tried again, and two failures with a clear reason
  // are the answer. (The library retries by itself only once per boot, so
  // the retries are made here.)
  uint16_t failures = wifiDiscCount - wifiTryDiscStart;
  WifiFailKind kind = wifiFailKind(wifiDiscReason);
  if (millis() - wifiTryStartMs > WIFI_CONNECT_TIMEOUT_MS) {
    wifiFailTry(failures == 0 && st == WL_NO_SSID_AVAIL ? "network not found" : wifiFailText(kind));
  } else if (failures > wifiTryDiscSeen) {
    wifiTryDiscSeen = failures;
    if (failures >= 2 && kind != WIFI_FAIL_OTHER) {
      wifiFailTry(wifiFailText(kind));
    } else if (failures < 4) {
      WiFi.begin(wifiNewSsid.c_str(), wifiNewPass.length() ? wifiNewPass.c_str() : nullptr);
    }
  }
}

// ---------------------------------------------------------------------------
// Setup page
// ---------------------------------------------------------------------------
static String wifiHtmlEscape(const String &s) {
  String out;
  out.reserve(s.length() + 8);
  for (size_t i = 0; i < s.length(); i++) {
    char c = s[i];
    if (c == '&') out += "&amp;";
    else if (c == '<') out += "&lt;";
    else if (c == '>') out += "&gt;";
    else if (c == '"') out += "&quot;";
    else if (c == '\'') out += "&#39;";
    else out += c;
  }
  return out;
}

static String wifiPageStart(const char *title, int refreshSec = 0, const char *refreshTo = "/") {
  String h = F("<!doctype html><html><head><meta charset=utf-8>"
               "<meta name=viewport content='width=device-width,initial-scale=1'>");
  if (refreshSec > 0) h += "<meta http-equiv=refresh content='" + String(refreshSec) + ";url=" + refreshTo + "'>";
  h += F("<title>IsdaSafe WiFi setup</title><style>"
         "body{font-family:sans-serif;max-width:480px;margin:0 auto;padding:16px;color:#123}"
         "h1{font-size:1.4em;color:#0a6b8a}h2{font-size:1.05em;margin-top:1.4em}"
         ".muted{color:#678;font-size:.9em}"
         ".net{display:flex;align-items:center;gap:10px;padding:12px;border:1px solid #cde;border-radius:8px;margin:6px 0}"
         ".net span{margin-left:auto;color:#678;font-size:.85em;white-space:nowrap}"
         "input[type=password],input[type=text]{width:100%;box-sizing:border-box;padding:10px;font-size:1em;border:1px solid #9ab;border-radius:6px}"
         "button{width:100%;padding:14px;margin-top:16px;font-size:1.05em;background:#0a6b8a;color:#fff;border:0;border-radius:8px}"
         ".ok{color:#0a7a3a}.bad{color:#b02020}a{color:#0a6b8a}"
         "</style></head><body><h1>");
  h += title;
  h += "</h1>";
  return h;
}

static void wifiRedirect(const String &url) {
  wifiServer.sendHeader("Location", url, true);
  wifiServer.send(302, "text/plain", "");
}

static void wifiHandleRoot() {
  if (wifiTrying()) {   // e.g. the phone rejoined mid-attempt
    wifiRedirect("/status");
    return;
  }
  // Pre-select the network that just failed (to retype its password), else
  // the saved one, else the strongest.
  int selected = wifiTry == WIFI_TRY_FAIL ? wifiFindNet(wifiNewSsid) : -1;
  if (selected < 0) selected = wifiFindNet(wifiSsid);
  if (selected < 0) selected = 0;

  String h = wifiPageStart("IsdaSafe WiFi setup");
  h += "<p class=muted>Device ID " + deviceId() + " &middot; " +
       ((isPaired() || isProvisioned()) ? "paired" : "not paired yet") + "</p><p>";
  if (WiFi.status() == WL_CONNECTED) {
    h += "<span class=ok>Connected to <b>" + wifiHtmlEscape(WiFi.SSID()) + "</b></span>";
  } else if (wifiSsid.length() == 0) {
    h += "No WiFi network chosen yet.";
  }
  h += F("</p><form method=POST action=/save><h2>Choose a network</h2>");
  if (wifiNetCount == 0) h += F("<p class=muted>No networks found.</p>");
  for (int i = 0; i < wifiNetCount; i++) {
    int pct = constrain(2 * (wifiNets[i].rssi + 100), 0, 100);
    String esc = wifiHtmlEscape(wifiNets[i].ssid);
    h += "<label class=net><input type=radio name=ssid value=\"" + esc + "\"" + (i == selected ? " checked" : "") +
         ">" + esc + "<span>" + String(pct) + "% &middot; " + (wifiNets[i].open ? "open" : "secured") +
         "</span></label>";
  }
  h += F("<label class=net><input type=radio name=ssid value='' id=oth>"
         "<input type=text name=other placeholder='Other (hidden) network name' onfocus='oth.checked=true'></label>"
         "<p><a href=/scan>Scan again</a> <span class=muted>(only 2.4 GHz networks are shown)</span></p>"
         "<h2>Password</h2><input type=password name=pass id=pw placeholder='Leave empty for open networks'>"
         "<p><label><input type=checkbox onclick=\"pw.type=this.checked?'text':'password'\"> Show password</label></p>"
         "<button>Save and connect</button></form></body></html>");
  wifiServer.send(200, "text/html", h);
}

static void wifiHandleScan() {
  if (!wifiTrying()) wifiScan();
  wifiRedirect("/");
}

static void wifiHandleSave() {
  String ssid = wifiServer.arg("ssid");
  if (ssid.length() == 0) {
    ssid = wifiServer.arg("other");
    ssid.trim();
  }
  if (ssid.length() == 0 || ssid.length() > 32) {
    wifiRedirect("/");
    return;
  }
  wifiStartTry(ssid, wifiServer.arg("pass"));
  wifiRedirect("/status");
}

static void wifiHandleStatus() {
  String h;
  String name = "<b>" + wifiHtmlEscape(wifiNewSsid) + "</b>";
  if (wifiTry == WIFI_TRY_RUNNING) {
    h = wifiPageStart("Connecting...", 2, "/status");
    h += "<p>Connecting to " + name + ". This takes up to 20 seconds.</p>"
         "<p class=muted>If this page stops loading, your phone may have left the IsdaSafe hotspot. "
         "Rejoin it to see the result.</p>";
  } else if (wifiTry == WIFI_TRY_OK) {
    h = wifiPageStart("Connected");
    h += "<p class=ok>The device is now connected to " + name + " and will use it from now on.</p>"
         "<p>This setup hotspot closes in a minute. You can switch your phone back to your normal WiFi.</p>";
  } else if (wifiTry == WIFI_TRY_FAIL) {
    h = wifiPageStart("Couldn't connect");
    h += "<p class=bad>Couldn't connect to " + name + ": " + wifiHtmlEscape(wifiTryReason) + ".</p>"
         "<p><a href=/>Try again</a></p>";
  } else {
    wifiRedirect("/");
    return;
  }
  h += F("</body></html>");
  wifiServer.send(200, "text/html", h);
}

// Phones probe a fixed URL to detect hotspot login pages; redirecting every
// unknown URL to the setup page makes it pop up automatically.
static void wifiHandleNotFound() {
  wifiRedirect("http://" + WiFi.softAPIP().toString() + "/");
}

// ---------------------------------------------------------------------------
// Public API
// ---------------------------------------------------------------------------
void wifiStartSetup() {
  if (wifiSetupOn) {
    Serial.printf("WIFI SETUP %s\n", wifiApName.c_str());
    return;
  }
  // Hotspot first, so it shows up on phones at once; the network list is
  // scanned right after. (Online, the hotspot shares the network's channel
  // anyway.)
  uint8_t channel = WiFi.status() == WL_CONNECTED ? WiFi.channel() : wifiSetupChannel();
  WiFi.mode(WIFI_AP_STA);
  WiFi.softAP(wifiApName.c_str(), WIFI_SETUP_AP_PASSWORD, channel);
  delay(100);
  wifiDns.start(53, "*", WiFi.softAPIP());
  if (!wifiRoutesAdded) {
    wifiServer.on("/", HTTP_GET, wifiHandleRoot);
    wifiServer.on("/scan", HTTP_GET, wifiHandleScan);
    wifiServer.on("/save", HTTP_POST, wifiHandleSave);
    wifiServer.on("/status", HTTP_GET, wifiHandleStatus);
    wifiServer.onNotFound(wifiHandleNotFound);
    wifiRoutesAdded = true;
  }
  wifiServer.begin();
  wifiSetupOn = true;
  wifiSetupStartMs = millis();
  wifiSetupCloseAtMs = 0;
  Serial.printf("WIFI SETUP %s\n", wifiApName.c_str());
  Serial.printf("[WiFi] Setup hotspot \"%s\" is open on channel %u (password: %s). Join it from a phone, "
                "or browse to http://%s\n",
                wifiApName.c_str(), (unsigned)channel, WIFI_SETUP_AP_PASSWORD,
                WiFi.softAPIP().toString().c_str());
  if (!wifiTrying()) wifiScan();   // ~10 s; a phone joining meanwhile just waits for the page
}

void wifiStopSetup() {
  if (!wifiSetupOn) return;
  wifiServer.stop();
  wifiDns.stop();
  WiFi.softAPdisconnect(true);
  WiFi.mode(WIFI_STA);
  wifiSetupOn = false;
  Serial.println("[WiFi] Setup hotspot closed.");
  if (WiFi.status() != WL_CONNECTED) wifiConnectSaved();
}

// Call once in setup(), after pairingBegin(). Starts connecting to the
// saved network and returns true, or returns false when none is saved (the
// caller should then open the setup hotspot).
bool wifiBegin() {
  wifiPrefs.begin("isdawifi", false);
  wifiSsid = wifiPrefs.getString("ssid", "");
  wifiPass = wifiPrefs.getString("pass", "");
#ifdef WIFI_SSID
  // Compile-time network from secrets.h, used only until one is chosen.
  if (wifiSsid.length() == 0 && String(WIFI_SSID) != "YOUR_WIFI_SSID") {
    wifiSsid = WIFI_SSID;
    wifiPass = WIFI_PASSWORD;
  }
#endif
  String id = deviceId();
  wifiApName = "IsdaSafe-" + id.substring(id.length() - 4);
#if WIFI_SETUP_BUTTON_PIN >= 0
  pinMode(WIFI_SETUP_BUTTON_PIN, INPUT_PULLUP);
#endif
  WiFi.onEvent(wifiOnDisconnected, ARDUINO_EVENT_WIFI_STA_DISCONNECTED);
  WiFi.persistent(false);       // we keep the credentials ourselves
  WiFi.setAutoReconnect(false); // wifiLoop() retries, and holds off while a phone is on the hotspot
  WiFi.mode(WIFI_STA);
  if (wifiSsid.length() == 0) return false;
  wifiConnectSaved();
  return true;
}

// Call often (from every wait): runs the setup page, finishes connection
// attempts, retries the saved network and watches the BOOT button.
void wifiLoop() {
  if (wifiSetupOn) {
    wifiDns.processNextRequest();
    wifiServer.handleClient();
  }

  if (wifiTry == WIFI_TRY_RUNNING) wifiFollowTry();

  // Online again (or for the first time since a network was chosen here):
  // check in shortly, once the connection has settled. The connection made
  // at boot doesn't need it -- the reading right after it uploads anyway.
  static bool wasConnected = false, connectedBefore = false;
  bool connected = WiFi.status() == WL_CONNECTED;
  if (connected && !wasConnected) {
    wifiSaveChannel();
    if (connectedBefore || wifiTry == WIFI_TRY_OK) wifiCheckInAtMs = millis() + WIFI_CHECKIN_DELAY_MS;
    connectedBefore = true;
  } else if (!connected && wasConnected && !wifiTrying()) {
    Serial.println("[WiFi] connection lost - reconnecting.");
    wifiLastBeginMs = millis() - WIFI_RETRY_MS + 2000;   // first retry in 2 s
  }
  wasConnected = connected;
  if (wifiCheckInAtMs && (long)(millis() - wifiCheckInAtMs) >= 0) {
    wifiCheckInAtMs = 0;
    if (connected) queueCheckIn();
  }

  // Offline: keep retrying the saved network, but not while a phone is on
  // the setup hotspot (an attempt can switch channels and drop the phone).
  static bool retryHeld = false;
  if (!connected && !wifiTrying() && wifiSsid.length() > 0) {
    bool hold = wifiSetupInUse();
    if (hold && !retryHeld) Serial.println("[WiFi] phone on the setup hotspot - holding reconnect attempts.");
    retryHeld = hold;
    if (!hold && millis() - wifiLastBeginMs >= WIFI_RETRY_MS) {
      Serial.printf("[WiFi] retrying \"%s\"...\n", wifiSsid.c_str());
      wifiConnectSaved();
    }
  }

  // Lost the network mid-run (router off, password changed, moved): after a
  // while, open the hotspot so a new network can be chosen without access
  // to the board. Retrying the saved network continues meanwhile.
  static unsigned long offlineSinceMs = 0;
  if (connected || wifiTrying()) {
    offlineSinceMs = 0;
  } else if (offlineSinceMs == 0) {
    offlineSinceMs = millis();
  } else if (!wifiSetupOn && millis() - offlineSinceMs >= WIFI_OFFLINE_SETUP_MS) {
    Serial.println("[WiFi] offline for a while - opening the setup hotspot.");
    wifiStartSetup();
  }

  if (wifiSetupOn && !wifiTrying()) {
    bool done = wifiSetupCloseAtMs && (long)(millis() - wifiSetupCloseAtMs) >= 0;
    // Only give up on the hotspot while online; offline it's the only way
    // to choose a network.
    bool timedOut = WiFi.status() == WL_CONNECTED &&
                    millis() - wifiSetupStartMs > WIFI_SETUP_TIMEOUT_MS;
    if (done || timedOut) wifiStopSetup();
  }

#if WIFI_SETUP_BUTTON_PIN >= 0
  static unsigned long btnDownMs = 0;
  if (digitalRead(WIFI_SETUP_BUTTON_PIN) == LOW) {
    if (btnDownMs == 0) btnDownMs = millis();
    else if (!wifiSetupOn && millis() - btnDownMs >= WIFI_SETUP_BUTTON_MS) wifiStartSetup();
  } else {
    btnDownMs = 0;
  }
#endif
}

// After a successful PAIR: open the hotspot so the owner can choose the
// network from a phone (the website can also do it over USB with WIFI ...).
void wifiOnPaired() {
  wifiStartSetup();
}

// Handles WIFI ... serial commands; returns false if `line` isn't one.
bool wifiHandleSerialCommand(const String &line) {
  if (line != "WIFI" && !line.startsWith("WIFI ")) return false;
  String rest = line.length() > 5 ? line.substring(5) : String("");

  if (rest == "SCAN") {
    if (wifiTrying()) {
      Serial.println("WIFI SCAN DONE 0");
      return true;
    }
    int n = wifiScan();
    for (int i = 0; i < n; i++) {
      Serial.printf("WIFI NET %d %s %s\n", (int)wifiNets[i].rssi,
                    wifiNets[i].open ? "open" : "secured", wifiNets[i].ssid.c_str());
    }
    Serial.printf("WIFI SCAN DONE %d\n", n);
  } else if (rest.startsWith("SSID ")) {
    wifiNewSsid = rest.substring(5);
    wifiNewPass = "";
    Serial.println(wifiNewSsid.length() && wifiNewSsid.length() <= 32 ? "WIFI SSID OK" : "WIFI SSID ERROR");
  } else if (rest == "PASS" || rest.startsWith("PASS ")) {
    wifiNewPass = rest.length() > 5 ? rest.substring(5) : String("");
    Serial.println(wifiNewPass.length() <= 63 ? "WIFI PASS OK" : "WIFI PASS ERROR");   // never echo it
  } else if (rest == "CONNECT") {
    if (wifiNewSsid.length() == 0 || wifiNewSsid.length() > 32 || wifiNewPass.length() > 63) {
      Serial.println("WIFI FAIL send WIFI SSID first");
    } else {
      wifiStartTry(wifiNewSsid, wifiNewPass);
    }
  } else if (rest == "STATUS") {
    if (WiFi.status() == WL_CONNECTED) {
      Serial.printf("WIFI STATUS CONNECTED %s %d %s\n", WiFi.localIP().toString().c_str(),
                    (int)WiFi.RSSI(), WiFi.SSID().c_str());
    } else {
      Serial.printf("WIFI STATUS DISCONNECTED %s\n", wifiSsid.length() ? wifiSsid.c_str() : "-");
    }
  } else if (rest == "SETUP") {
    wifiStartSetup();
  } else if (rest == "FORGET") {
    wifiPrefs.remove("ssid");
    wifiPrefs.remove("pass");
    wifiSsid = "";
    wifiPass = "";
    WiFi.disconnect();
    Serial.println("WIFI FORGET OK");
    wifiStartSetup();
  } else {
    Serial.println("WIFI ERROR unknown command (SCAN, SSID, PASS, CONNECT, STATUS, SETUP, FORGET)");
  }
  return true;
}
