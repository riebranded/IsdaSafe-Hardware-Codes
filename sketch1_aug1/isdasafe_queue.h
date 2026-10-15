// IsdaSafe offline queue: readings taken while the device can't reach the
// server are kept in flash (LittleFS, survives reboots, deep sleep and power
// cuts) and uploaded, oldest first, once it's back online.
//
// - Every reading carries the time it was taken. The clock comes from the
//   internet (NTP) and keeps running through deep sleep. A reading taken
//   before the clock was ever set (booted without WiFi) is stamped with
//   millis() and converted once the time is known -- unless the device
//   restarted in between, in which case its time is lost and it's dropped.
// - Uploads go to ingest_sensor_readings() in batches of QUEUE_BATCH. The
//   server skips readings it already has, so resending a batch whose reply
//   was lost is harmless.
// - At most QUEUE_MAX readings are kept (about a week at one a minute);
//   when full, the oldest are dropped.
// - A rejected upload (device not paired / removed in the app) isn't queued:
//   those readings have nowhere to go.
//
// Serial: `QUEUE` prints how many readings are waiting; `QUEUE CLEAR`
// deletes them.
//
// The sketch calls queueBegin() in setup() and submitReading() wherever it
// used to call uploadReading().

#pragma once

#include <Arduino.h>
#include <LittleFS.h>
#include <Preferences.h>
#include <time.h>
#include "isdasafe_pairing.h"

#define QUEUE_FILE      "/queue.bin"
#define QUEUE_TMP_FILE  "/queue.tmp"
#define QUEUE_MAX       10080   // readings kept while offline (~1 week at 1/min, ~380 KB)
#define QUEUE_BATCH     50      // readings per upload request
#define QUEUE_MAX_BATCHES_PER_CALL 10   // a long backlog drains over several reading cycles

// Values in the order the batch JSON sends them.
enum { Q_DO, Q_PH, Q_NH3, Q_FNH3, Q_WT, Q_AT, Q_HUM, Q_VALUES };
static const char *const kQueueKeys[Q_VALUES] = { "do", "ph", "nh3", "fnh3", "wt", "at", "hum" };
static const uint8_t kQueueDecimals[Q_VALUES] = { 2, 2, 2, 0 /* full */, 1, 1, 1 };

#define Q_EPOCH 0x01   // t is unix seconds; otherwise millis() during boot `boot`

struct __attribute__((packed)) QueuedReading {
  uint32_t t;
  uint16_t boot;
  uint8_t flags;
  float v[Q_VALUES];
};

static Preferences queuePrefs;
static bool queueFsOk = false;
static uint16_t queueBoot = 0;
static uint32_t queueHead = 0;   // index of the oldest unsent record in QUEUE_FILE

inline bool clockKnown() { return time(nullptr) > 1700000000; }   // after Nov 2023

static uint32_t queueCount() {
  if (!queueFsOk) return 0;
  File f = LittleFS.open(QUEUE_FILE, "r");
  if (!f) return 0;
  uint32_t n = f.size() / sizeof(QueuedReading);
  f.close();
  return n;
}

// Readings waiting to be uploaded.
inline uint32_t queueSize() {
  uint32_t n = queueCount();
  return n > queueHead ? n - queueHead : 0;
}

static void queueSetHead(uint32_t head) {
  queueHead = head;
  queuePrefs.putUInt("head", head);
}

static void queueClear() {
  LittleFS.remove(QUEUE_FILE);
  queueSetHead(0);
}

// Drops what's already been sent from the front of the file, once that's a
// good share of it.
static void queueCompact() {
  uint32_t n = queueCount();
  if (queueHead == 0) return;
  if (queueHead >= n) {
    queueClear();
    return;
  }
  if (queueHead < 500 || queueHead * 2 < n) return;
  File in = LittleFS.open(QUEUE_FILE, "r");
  File out = LittleFS.open(QUEUE_TMP_FILE, "w");
  if (!in || !out) return;
  in.seek(queueHead * sizeof(QueuedReading));
  uint8_t buf[512];
  size_t got;
  while ((got = in.read(buf, sizeof(buf))) > 0) out.write(buf, got);
  in.close();
  out.close();
  LittleFS.remove(QUEUE_FILE);
  LittleFS.rename(QUEUE_TMP_FILE, QUEUE_FILE);
  queueSetHead(0);
}

// Call once in setup(), after Serial.begin().
inline void queueBegin() {
  queuePrefs.begin("isdaqueue", false);
  queueBoot = queuePrefs.getUShort("boot", 0) + 1;
  queuePrefs.putUShort("boot", queueBoot);
  queueHead = queuePrefs.getUInt("head", 0);
  queueFsOk = LittleFS.begin(true);   // formats the partition the first time
  if (!queueFsOk) Serial.println("[Queue] LittleFS failed - offline readings won't be kept.");
  // UTC; the clock is set as soon as WiFi is up and kept through deep sleep.
  configTime(0, 0, "pool.ntp.org", "time.google.com", "time.cloudflare.com");
  if (queueHead > queueCount()) queueSetHead(0);
  Serial.printf("[Queue] %u reading(s) waiting to upload\n", (unsigned)queueSize());
}

static void queuePush(const QueuedReading &r) {
  if (!queueFsOk) return;
  if (queueSize() >= QUEUE_MAX) {
    queueSetHead(queueHead + 1);   // full: drop the oldest
    queueCompact();
  }
  File f = LittleFS.open(QUEUE_FILE, "a");
  if (!f) return;
  f.write((const uint8_t *)&r, sizeof(r));
  f.close();
}

// The reading's unix time, if it can be known now. `lost` = never (taken
// before a restart, before the clock was set).
static bool queueTimeOf(const QueuedReading &r, uint32_t &unixTime, bool &lost) {
  lost = false;
  if (r.flags & Q_EPOCH) {
    unixTime = r.t;
    return true;
  }
  if (r.boot != queueBoot) {
    lost = true;
    return false;
  }
  if (!clockKnown()) return false;
  unixTime = (uint32_t)time(nullptr) - (millis() - r.t) / 1000;
  return true;
}

static String queueJsonItem(const QueuedReading &r, uint32_t unixTime) {
  String s = "{\"t\":" + String(unixTime);
  for (int i = 0; i < Q_VALUES; i++) {
    s += ",\"";
    s += kQueueKeys[i];
    s += "\":";
    s += kQueueDecimals[i] ? jsonNumber(r.v[i], kQueueDecimals[i]) : jsonNumberFull(r.v[i]);
  }
  return s + "}";
}

// JSON string literal for `s` (quotes, backslashes and control characters
// escaped).
static String jsonString(const String &s) {
  String out = "\"";
  for (size_t i = 0; i < s.length(); i++) {
    char c = s[i];
    if (c == '"' || c == '\\') {
      out += '\\';
      out += c;
    } else if ((uint8_t)c < 0x20) {
      char buf[7];
      snprintf(buf, sizeof(buf), "\\u%04x", (unsigned)(uint8_t)c);
      out += buf;
    } else {
      out += c;
    }
  }
  return out + "\"";
}

// ingest_sensor_readings() parameters for a batch of `items` (JSON objects,
// comma-separated). The network name lets the app confirm a WiFi change.
static String queueBatchParams(const String &items) {
  return ",\"p_readings\":[" + items + "],\"p_wifi_ssid\":" + jsonString(WiFi.SSID());
}

// Uploads queued readings, oldest first, up to `maxBatches` requests.
// Returns true once nothing is left waiting.
static bool queueFlush(int maxBatches = QUEUE_MAX_BATCHES_PER_CALL) {
  if (!queueFsOk) return true;
  for (int batch = 0; batch < maxBatches; batch++) {
    uint32_t n = queueCount();
    if (queueHead >= n) {
      queueCompact();
      return true;
    }
    if (WiFi.status() != WL_CONNECTED) return false;

    File f = LittleFS.open(QUEUE_FILE, "r");
    if (!f) return false;
    f.seek(queueHead * sizeof(QueuedReading));
    String items;
    uint32_t used = 0, sent = 0, lost = 0;
    QueuedReading r;
    while (sent < QUEUE_BATCH && queueHead + used < n &&
           f.read((uint8_t *)&r, sizeof(r)) == sizeof(r)) {
      uint32_t unixTime;
      bool isLost;
      if (!queueTimeOf(r, unixTime, isLost)) {
        if (!isLost) break;   // clock not set yet: try again later
        used++;
        lost++;
        continue;
      }
      if (sent) items += ",";
      items += queueJsonItem(r, unixTime);
      used++;
      sent++;
    }
    f.close();
    if (lost) Serial.printf("[Queue] dropped %u reading(s) taken before a restart with no clock\n", (unsigned)lost);
    if (used == 0) return false;   // waiting for the clock

    if (sent) {
      Serial.printf("[Queue] uploading %u queued reading(s)...\n", (unsigned)sent);
      int status = isdaPostRpc("ingest_sensor_readings", queueBatchParams(items));
      if (status == 400) {
        // The server can't use this batch; skip it rather than block the
        // whole queue behind it.
        Serial.printf("[Queue] server refused %u reading(s); skipping them\n", (unsigned)used);
      } else if (status != 200) {
        // Offline again, server down, device not paired, or the server
        // doesn't have ingest_sensor_readings() yet (404): keep them.
        return false;
      }
    }
    queueSetHead(queueHead + used);
  }
  queueCompact();
  return queueSize() == 0;
}

// Tells the server the device is online and on which network -- the app
// waits for that after the owner chooses a network -- uploading readings
// kept while offline. isdasafe_wifi.h calls it once connected. With nothing
// (or not everything, e.g. the clock isn't set yet) uploaded, it sends an
// empty batch, which only records the check-in.
void queueCheckIn() {
  if (!isPaired() && !isProvisioned()) return;
  Serial.println("[Queue] online - checking in with the server.");
  if (queueSize() == 0 || !queueFlush()) {
    isdaPostRpc("ingest_sensor_readings", queueBatchParams(""));
  }
}

// Uploads a reading, or keeps it for later when the device is offline. Any
// value may be NAN (sent as null).
inline void submitReading(float dissolvedOxygen, float ph, float ammonia,
                          float freeAmmonia, float waterTemp, float airTemp,
                          float humidity) {
  QueuedReading r;
  r.boot = queueBoot;
  if (clockKnown()) {
    r.t = (uint32_t)time(nullptr);
    r.flags = Q_EPOCH;
  } else {
    r.t = millis();
    r.flags = 0;
  }
  r.v[Q_DO] = dissolvedOxygen;
  r.v[Q_PH] = ph;
  r.v[Q_NH3] = ammonia;
  r.v[Q_FNH3] = freeAmmonia;
  r.v[Q_WT] = waterTemp;
  r.v[Q_AT] = airTemp;
  r.v[Q_HUM] = humidity;

  if (!isPaired() && !isProvisioned()) {
    Serial.println("Not paired - connect the device to the IsdaSafe website over USB to pair it. Skipping upload.");
    return;
  }

  // Older readings go first so the history stays in order.
  if (WiFi.status() == WL_CONNECTED && queueFlush()) {
    uint32_t unixTime;
    bool lost;
    if (queueTimeOf(r, unixTime, lost)) {
      int status = isdaPostRpc("ingest_sensor_readings",
                               queueBatchParams(queueJsonItem(r, unixTime)));
      if (status == 200) return;
      if (status == 404) {
        // Server without the batch upload yet: send it the old way (stamped
        // on arrival) instead of queueing everything until it's updated.
        if (uploadReading(dissolvedOxygen, ph, ammonia, freeAmmonia, waterTemp,
                          airTemp, humidity)) return;
      } else if (status > 0 && status < 500) {
        return;   // rejected (e.g. device removed in the app): nowhere to keep it for
      }
    }
  }
  queuePush(r);
  Serial.printf("[Queue] reading saved for later (%u waiting)\n", (unsigned)queueSize());
}

// Handles QUEUE ... serial commands; returns false if `line` isn't one.
bool queueHandleSerialCommand(const String &line) {
  if (line == "QUEUE") {
    Serial.printf("[Queue] %u reading(s) waiting; clock %s\n", (unsigned)queueSize(),
                  clockKnown() ? "set" : "not set yet");
  } else if (line == "QUEUE CLEAR") {
    queueClear();
    Serial.println("[Queue] cleared");
  } else {
    return false;
  }
  return true;
}
