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
  } else if (line.length() > 0) {
    handleAppSerialCommand(line);
  }
}

// Reads commands from the website. Call it often: from loop() and inside
// every wait (see delayPollingDht in README.md), otherwise pairing can time
// out while the board sits in a long delay().
inline void pollSerialCommands() {
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\n' || c == '\r') {
      if (isdaCmdBuf.length() > 0) handleSerialCommand(isdaCmdBuf);
      isdaCmdBuf = "";
    } else if (isdaCmdBuf.length() < 128) {
      isdaCmdBuf += c;
    }
  }
}

// JSON number, or null for a sensor that failed to read (NaN).
static String jsonNumber(float v, int decimals = 2) {
  return isnan(v) || isinf(v) ? String("null") : String(v, decimals);
}

// Uploads one reading to the account this device is paired to. Returns
// true on success. Any value may be NAN (sent as null).
inline bool uploadReading(float dissolvedOxygen, float ph, float ammonia,
                          float waterTemp, float airTemp, float humidity) {
  if (!isPaired() && !isProvisioned()) {
    Serial.println("Not paired - connect the device to the IsdaSafe website over USB to pair it. Skipping upload.");
    return false;
  }
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("WiFi not connected - skipping upload.");
    return false;
  }

  WiFiClientSecure client;
  // TODO: replace with client.setCACert(<Supabase root CA>) so the token
  // can't be intercepted on an untrusted network.
  client.setInsecure();
  HTTPClient http;
  http.setTimeout(UPLOAD_TIMEOUT_MS);
  if (!http.begin(client, String(SUPABASE_URL) + "/rest/v1/rpc/ingest_sensor_reading")) {
    Serial.println("HTTP begin() failed.");
    return false;
  }
  http.addHeader("Content-Type", "application/json");
  http.addHeader("apikey", SUPABASE_ANON_KEY);
  http.addHeader("Authorization", String("Bearer ") + SUPABASE_ANON_KEY);

  String body = String("{\"p_token\":\"") + uploadCredential() + "\"" +
                ",\"p_dissolved_oxygen\":" + jsonNumber(dissolvedOxygen) +
                ",\"p_ph\":" + jsonNumber(ph) +
                ",\"p_ammonia\":" + jsonNumber(ammonia) +
                ",\"p_water_temperature\":" + jsonNumber(waterTemp, 1) +
                ",\"p_air_temperature\":" + jsonNumber(airTemp, 1) +
                ",\"p_humidity\":" + jsonNumber(humidity, 1) + "}";

  int status = http.POST(body);
  if (status == 200) {
    Serial.println("Upload OK");
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
  return status == 200;
}
