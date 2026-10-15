// IsdaSafe device pairing + authenticated uploads.
//
// Copy this file next to your .ino sketch (same folder), then see
// firmware/README.md for the small edits the sketch needs.
//
// How ownership works:
//   - The IsdaSafe website connects to the device over USB and pairs it to
//     the signed-in account. The database issues a secret token and the
//     website sends it here as `PAIR <64 hex chars>`; it's saved in flash
//     (survives reboots and deep sleep) and confirmed with `PAIRED OK`.
//   - Every reading is uploaded to Supabase's ingest_sensor_reading()
//     together with that token. The database looks the token up and files
//     the reading under the owning account and pond. Without a valid token
//     (never paired, or removed in the app) uploads are rejected.
//   - `UNPAIR` wipes the token (the app sends it when the device is removed
//     while plugged in).
//
// QR-code claiming (no USB needed by the customer):
//   - At build time tools/provision_device.ps1 sends `PROVISION <64 hex>`:
//     a permanent factory secret, saved in flash under its own key, plus a
//     claim code printed as a QR sticker. The database stores only hashes.
//   - Scanning the QR in the app claims the device. From then on uploads
//     with the factory secret are filed under the claiming account.
//   - When a factory secret is present it is used for uploads (the database
//     accepts it whether the device was claimed by QR or paired over USB).
//     `UNPAIR` never erases it.

#pragma once

#include <Arduino.h>
#include <HTTPClient.h>
#include <Preferences.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>

// ---------------------------------------------------------------------------
// Supabase project (same values as SUPABASE_URL / SUPABASE_ANON_KEY in the
// app's .env). The anon key is public by design; the device token is what
// proves ownership.
// (In this sketch the real key comes from secrets.h, which is included
// before this file and kept out of git.)
// ---------------------------------------------------------------------------
#ifndef SUPABASE_URL
#define SUPABASE_URL      "https://vemaphehzjnvspxmweeb.supabase.co"
#endif
#ifndef SUPABASE_ANON_KEY
#define SUPABASE_ANON_KEY "PASTE_SUPABASE_ANON_KEY_HERE"
#endif
#ifndef UPLOAD_TIMEOUT_MS
#define UPLOAD_TIMEOUT_MS 15000
#endif

static Preferences isdaPrefs;
static String isdaToken;
static String isdaSecret;   // factory secret from PROVISION (QR claiming)
static String isdaCmdBuf;

// The chip's factory-burned ID: 12 hex digits, unique per ESP32. This is
// what the website registers; it is NOT a secret (the token is).
inline String deviceId() {
  char buf[13];
  snprintf(buf, sizeof(buf), "%012llX", (unsigned long long)ESP.getEfuseMac());
  return String(buf);
}

inline bool isPaired() { return isdaToken.length() == 64; }
inline bool isProvisioned() { return isdaSecret.length() == 64; }

// Credential sent with uploads: the factory secret when provisioned,
// otherwise the USB pairing token.
static const String &uploadCredential() { return isProvisioned() ? isdaSecret : isdaToken; }

// Call once in setup(), after Serial.begin().
inline void pairingBegin() {
  isdaPrefs.begin("isdasafe", false);
  isdaToken = isdaPrefs.getString("token", "");
  isdaSecret = isdaPrefs.getString("secret", "");
  Serial.printf("ID=%s  PAIRED=%s\n", deviceId().c_str(), isPaired() ? "yes" : "no");
  Serial.printf("PROVISIONED=%s\n", isProvisioned() ? "yes" : "no");
}

static bool isHexToken(const String &s) {
  if (s.length() != 64) return false;
  for (size_t i = 0; i < s.length(); i++) {
    if (!isxdigit((unsigned char)s[i])) return false;
  }
  return true;
}

// Defined by the sketch: handles its own (non-pairing) serial commands.
void handleAppSerialCommand(const String &line);
// Defined in isdasafe_wifi.h: WIFI ... commands, and opening the WiFi setup
// hotspot right after pairing so the owner can choose the network.
bool wifiHandleSerialCommand(const String &line);
// Defined in isdasafe_queue.h: QUEUE / QUEUE CLEAR.
bool queueHandleSerialCommand(const String &line);
void wifiOnPaired();
void wifiStartSetup();

static void handleSerialCommand(String line) {
  line.trim();
  if (line.startsWith("PAIR ")) {
    String token = line.substring(5);
    token.trim();
    if (!isHexToken(token)) {
      Serial.println("PAIR ERROR invalid token");
      return;
    }
    isdaPrefs.putString("token", token);
    isdaToken = token;
    Serial.println("PAIRED OK");   // never echo the token itself
    wifiOnPaired();
  } else if (line.startsWith("PROVISION ")) {
    String secret = line.substring(10);
    secret.trim();
    secret.toLowerCase();
    if (!isHexToken(secret)) {
      Serial.println("PROVISION ERROR invalid secret");
      return;
    }
    isdaPrefs.putString("secret", secret);
    isdaSecret = secret;
    Serial.println("PROVISIONED OK");   // never echo the secret itself
  } else if (line == "UNPAIR") {
    isdaPrefs.remove("token");
    isdaToken = "";
    Serial.println("UNPAIRED OK");
  } else if (wifiHandleSerialCommand(line) || queueHandleSerialCommand(line)) {
    // handled
  } else if (line.length() > 0) {
    handleAppSerialCommand(line);
  }
}

// Reads commands from the website. Call it often: from loop() and inside
// every wait (see delayPollingDht in README.md), otherwise pairing can time
// out while the board sits in a long delay().
// A command runs on newline, or -- so the Serial Monitor's "No Line Ending"
// setting also works -- 300 ms after its last character arrives.
static unsigned long isdaLastCharMs = 0;

inline void pollSerialCommands() {
  while (Serial.available()) {
    char c = (char)Serial.read();
    isdaLastCharMs = millis();
    if (c == '\n' || c == '\r') {
      if (isdaCmdBuf.length() > 0) handleSerialCommand(isdaCmdBuf);
      isdaCmdBuf = "";
    } else if (isdaCmdBuf.length() < 128) {
      isdaCmdBuf += c;
    }
  }
  if (isdaCmdBuf.length() > 0 && millis() - isdaLastCharMs > 300) {
    handleSerialCommand(isdaCmdBuf);
    isdaCmdBuf = "";
  }
}

// JSON number, or null for a sensor that failed to read (NaN).
static String jsonNumber(float v, int decimals = 2) {
  return isnan(v) || isinf(v) ? String("null") : String(v, decimals);
}

// JSON number at full float precision (no fixed rounding), e.g. 0.00312845
// or 3.1e-05 -- for small computed values like free NH3.
static String jsonNumberFull(float v) {
  if (isnan(v) || isinf(v)) return String("null");
  char buf[24];
  snprintf(buf, sizeof(buf), "%.7g", v);
  return String(buf);
}

// Calls a Supabase upload RPC with this device's credential added as
// p_token. `params` is the rest of the JSON object's members, starting with
// a comma. Returns the HTTP status (200 = stored), or a negative value when
// the request couldn't be made at all (no WiFi, timeout, ...).
static int isdaPostRpc(const char *rpc, const String &params) {
  if (!isPaired() && !isProvisioned()) {
    Serial.println("Not paired - connect the device to the IsdaSafe website over USB to pair it. Skipping upload.");
    return 0;
  }
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("WiFi not connected - skipping upload.");
    return -1;
  }

  WiFiClientSecure client;
  // TODO: replace with client.setCACert(<Supabase root CA>) so the token
  // can't be intercepted on an untrusted network.
  client.setInsecure();
  HTTPClient http;
  http.setTimeout(UPLOAD_TIMEOUT_MS);
  if (!http.begin(client, String(SUPABASE_URL) + "/rest/v1/rpc/" + rpc)) {
    Serial.println("HTTP begin() failed.");
    return -1;
  }
  http.addHeader("Content-Type", "application/json");
  http.addHeader("apikey", SUPABASE_ANON_KEY);
  http.addHeader("Authorization", String("Bearer ") + SUPABASE_ANON_KEY);

  int status = http.POST(String("{\"p_token\":\"") + uploadCredential() + "\"" + params + "}");
  bool setupRequested = false;
  if (status == 200) {
    Serial.println("Upload OK");
    // The owner pressed "Change WiFi" on the website: the reply is
    // "ok wifi_setup" instead of "ok".
    setupRequested = http.getString().indexOf("wifi_setup") >= 0;
  } else if (status > 0) {
    // 401/403 here means the token was revoked (device removed in the app)
    // or the device was paired again elsewhere: pair it again over USB.
    // For a provisioned board it means nobody has claimed it yet: scan its
    // QR sticker in the app.
    Serial.printf("Upload rejected: HTTP %d %s\n", status, http.getString().c_str());
  } else {
    Serial.printf("Upload failed: %s\n", http.errorToString(status).c_str());
  }
  http.end();
  if (setupRequested) {   // after the connection is closed
    Serial.println("[WiFi] setup requested from the website.");
    wifiStartSetup();
  }
  return status;
}

// Uploads one reading to the account this device is paired to, stamped with
// the time it arrives. Returns true on success. Any value may be NAN (sent
// as null). ammonia = total ammonia nitrogen (TAN, as measured);
// freeAmmonia = free NH3 computed from TAN, pH and water temperature.
// (The sketch uploads through isdasafe_queue.h instead, which keeps
// readings while offline and sends the time each was taken.)
inline bool uploadReading(float dissolvedOxygen, float ph, float ammonia,
                          float freeAmmonia, float waterTemp, float airTemp,
                          float humidity) {
  return isdaPostRpc("ingest_sensor_reading",
                     ",\"p_dissolved_oxygen\":" + jsonNumber(dissolvedOxygen) +
                     ",\"p_ph\":" + jsonNumber(ph) +
                     ",\"p_ammonia\":" + jsonNumber(ammonia) +
                     ",\"p_free_ammonia\":" + jsonNumberFull(freeAmmonia) +
                     ",\"p_water_temperature\":" + jsonNumber(waterTemp, 1) +
                     ",\"p_air_temperature\":" + jsonNumber(airTemp, 1) +
                     ",\"p_humidity\":" + jsonNumber(humidity, 1)) == 200;
}
