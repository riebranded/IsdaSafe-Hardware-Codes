// IsdaSafe LCD screens (16x2 I2C). Included by sketch1_aug1.ino right where
// the LCD code goes: it uses that file's `lcd`, `readings`, `lcdPage` and
// LCD_PAGE_DISPLAY_MS, and the WiFi/queue helpers. (A header rather than
// .ino code because Arduino's generated prototypes would come before the
// types declared here.)

#pragma once

// ---------------------------------------------------------------------------
// LCD (16x2, I2C). One reading per page, always laid out the same way:
//
//   Dissolved O2   W     <- title, WiFi icon in the corner (W = on, X = off)
//   6.12mg/L    OK       <- value + unit, and its status on the right
//
// Status uses the same limits as the app (lib/models/reading_bands.dart):
// OK / WARN / DANGER. If anything is in danger, an alert page leads the
// rotation. A status page shows the WiFi network and whether readings are
// synced or saved offline. "--" means the sensor has no reading.
// ---------------------------------------------------------------------------

// While the sensors warm up before a reading (duty cycle): when it ends, as
// millis(); 0 when not warming up. The screen then alternates between a
// countdown (lcdShowCountdown) and the live reading pages.
static unsigned long lcdWarmupUntilMs = 0;
static bool lcdShowCountdown = true;

// Custom characters (CGRAM slots 1-7; 0 can't go inside a String).
enum : uint8_t { LCD_WIFI = 1, LCD_NO_WIFI, LCD_WARN, LCD_OK, LCD_DROP, LCD_TEMP, LCD_UPLOAD };
#define LCD_DEG "\xDF"   // the degree sign in the HD44780's built-in font

void lcdCreateChars() {
  uint8_t wifi[8]   = {0b00000, 0b01110, 0b10001, 0b00100, 0b01010, 0b00000, 0b00100, 0b00000};
  uint8_t noWifi[8] = {0b10001, 0b01010, 0b00100, 0b01010, 0b10001, 0b00000, 0b00100, 0b00000};
  uint8_t warn[8]   = {0b00100, 0b01110, 0b01110, 0b01110, 0b00100, 0b00000, 0b00100, 0b00000};
  uint8_t ok[8]     = {0b00000, 0b00001, 0b00011, 0b10110, 0b11100, 0b01000, 0b00000, 0b00000};
  uint8_t drop[8]   = {0b00100, 0b00100, 0b01010, 0b01010, 0b10001, 0b10001, 0b01110, 0b00000};
  uint8_t temp[8]   = {0b00100, 0b01010, 0b01010, 0b01110, 0b01110, 0b11111, 0b11111, 0b01110};
  uint8_t upload[8] = {0b00100, 0b01110, 0b10101, 0b00100, 0b00100, 0b00000, 0b11111, 0b00000};
  lcd.createChar(LCD_WIFI, wifi);
  lcd.createChar(LCD_NO_WIFI, noWifi);
  lcd.createChar(LCD_WARN, warn);
  lcd.createChar(LCD_OK, ok);
  lcd.createChar(LCD_DROP, drop);
  lcd.createChar(LCD_TEMP, temp);
  lcd.createChar(LCD_UPLOAD, upload);
}

void printLine(uint8_t row, const String &text) {
  String padded = text;
  while (padded.length() < LCD_COLS) padded += ' ';
  lcd.setCursor(0, row);
  lcd.print(padded.substring(0, LCD_COLS)); // hard cap -- never exceeds 16 chars
}

// `left` and `right` on one line, with spaces between (left is cut short if
// both don't fit).
String lcdSpread(String left, const String &right) {
  int room = LCD_COLS - (int)right.length();
  if ((int)left.length() > room) left = left.substring(0, room > 0 ? room : 0);
  while ((int)left.length() < room) left += ' ';
  return left + right;
}

// Centred on a line.
String lcdCenter(const String &text) {
  int pad = (LCD_COLS - (int)text.length()) / 2;
  String out;
  for (int i = 0; i < pad; i++) out += ' ';
  return out + text;
}

// Boot screen: the name on top, what it's doing underneath.
void lcdBootStep(const String &step) {
  printLine(0, lcdCenter("IsdaSafe"));
  printLine(1, lcdCenter(step));
}

// 0 OK, 1 WARN, 2 DANGER, -1 no reading. Same limits as the app.
enum LcdMetric { M_WATER_T, M_PH, M_DO, M_NH3, M_FREE_NH3 };

int lcdStatus(LcdMetric metric, float v) {
  if (isnan(v)) return -1;
  switch (metric) {
    case M_WATER_T:  return (v < 22 || v > 32) ? 2 : (v < 25 || v > 29) ? 1 : 0;
    case M_PH:       return (v < 6.0f || v > 9.0f) ? 2 : (v < 6.5f || v > 8.5f) ? 1 : 0;
    case M_DO:       return v < 2 ? 2 : v < 4 ? 1 : 0;
    case M_NH3:      return v > 0.15f ? 2 : v > 0.05f ? 1 : 0;
    case M_FREE_NH3: return v > 0.05f ? 2 : v > 0.02f ? 1 : 0;
  }
  return -1;
}

String lcdStatusWord(int status) {
  switch (status) {
    case 0:  return String((char)LCD_OK) + "OK";
    case 1:  return String((char)LCD_WARN) + "WARN";
    case 2:  return String((char)LCD_WARN) + "DANGER";
    default: return "";
  }
}

String lcdValue(float v, int decimals, const char *unit) {
  return isnan(v) ? String("--") : String(v, decimals) + unit;
}

// Title line: the page's title, the WiFi icon in the last column.
void lcdTitle(const String &title) {
  char icon = WiFi.status() == WL_CONNECTED ? (char)LCD_WIFI : (char)LCD_NO_WIFI;
  printLine(0, lcdSpread(title, String(icon)));
}

// A reading page: title, then value + its status.
void lcdReadingPage(const String &title, LcdMetric metric, float v, int decimals, const char *unit) {
  lcdTitle(title);
  printLine(1, lcdSpread(lcdValue(v, decimals, unit) + " ", lcdStatusWord(lcdStatus(metric, v))));
}

// Pages, in order. Each is shown for LCD_PAGE_DISPLAY_MS.
enum LcdPage { P_ALERT, P_WATER_T, P_PH, P_DO, P_NH3, P_FREE_NH3, P_AIR, P_STATUS };

// The readings in danger/warning, worst first; for the alert page.
struct LcdIssue { const char *what; LcdMetric metric; float v; int decimals; const char *unit; int status; };

int lcdIssues(LcdIssue *out, int max) {
  LcdIssue all[] = {
#if ENABLE_RS485
    { readings.waterTemp_C > 29 ? "Hot water" : "Cold water", M_WATER_T, readings.waterTemp_C, 1, LCD_DEG "C", 0 },
    { readings.ph > 8.5f ? "High pH" : "Low pH", M_PH, readings.ph, 2, "", 0 },
    { "High NH3", M_NH3, readings.ammonia_mgL, 2, "mg/L", 0 },
    { "Toxic NH3", M_FREE_NH3, readings.freeNh3_mgL, 3, "", 0 },
#endif
    { "Low O2", M_DO, readings.dissolvedOxygen_mgL, 1, "mg/L", 0 },
  };
  int n = 0;
  for (int level = 2; level >= 1; level--) {
    for (auto &issue : all) {
      issue.status = lcdStatus(issue.metric, issue.v);
      if (issue.status == level && n < max) out[n++] = issue;
    }
  }
  return n;
}

// The pages to rotate through now (the alert page only while something
// needs attention). Returns how many.
uint8_t lcdPages(LcdPage *out) {
  uint8_t n = 0;
  LcdIssue issues[1];
  if (lcdIssues(issues, 1) > 0) out[n++] = P_ALERT;
#if ENABLE_RS485
  out[n++] = P_WATER_T;
  out[n++] = P_PH;
#endif
  out[n++] = P_DO;
#if ENABLE_RS485
  out[n++] = P_NH3;
  out[n++] = P_FREE_NH3;
#endif
  out[n++] = P_AIR;
  out[n++] = P_STATUS;
  return n;
}

uint8_t lcdPageCount() {
  LcdPage pages[8];
  return lcdPages(pages);
}

void updateLcd() {
  if (wifiSetupActive()) {   // tell whoever is standing there how to join
    // Alternates: the hotspot to join, then where the setup page is.
    if ((millis() / LCD_PAGE_DISPLAY_MS) % 2 == 0) {
      printLine(0, "WiFi setup: join");
      printLine(1, wifiSetupApName());
    } else {
      printLine(0, "Pass: " WIFI_SETUP_AP_PASSWORD);
      printLine(1, "or 192.168.4.1");
    }
    return;
  }

  // Warming up before the reading: a countdown every other page, so the
  // live (still settling) values show in between.
  if (lcdWarmupUntilMs && millis() < lcdWarmupUntilMs && lcdShowCountdown) {
    unsigned long left = (lcdWarmupUntilMs - millis()) / 1000UL;
    char clock[8];
    snprintf(clock, sizeof(clock), "%lu:%02lu", left / 60, left % 60);
    printLine(0, lcdSpread("Warming up", clock));
    printLine(1, "Sensors settling");
    return;
  }

  LcdPage pages[8];
  uint8_t count = lcdPages(pages);
  switch (pages[lcdPage % count]) {
    case P_ALERT: {
      LcdIssue issues[5];
      int n = lcdIssues(issues, 5);
      bool danger = n > 0 && issues[0].status == 2;
      String head = String((char)LCD_WARN) + (danger ? "DANGER" : "WARNING");
      printLine(0, lcdSpread(head, n == 1 ? "1 issue" : String(n) + " issues"));
      if (n > 0) {
        // The worst one; the others have their own pages.
        printLine(1, lcdSpread(String(issues[0].what) + " ",
                               lcdValue(issues[0].v, issues[0].decimals, issues[0].unit)));
      }
      break;
    }
    case P_WATER_T:
      lcdReadingPage(String((char)LCD_TEMP) + "Water Temp", M_WATER_T, readings.waterTemp_C, 1, LCD_DEG "C");
      break;
    case P_PH:
      lcdReadingPage("pH Level", M_PH, readings.ph, 2, "");
      break;
    case P_DO:
      lcdReadingPage("Dissolved O2", M_DO, readings.dissolvedOxygen_mgL, 2, "mg/L");
      break;
    case P_NH3:
      lcdReadingPage("Ammonia (TAN)", M_NH3, readings.ammonia_mgL, 2, "mg/L");
      break;
    case P_FREE_NH3:
      // The toxic part of the ammonia; "mg/L" left off so 3 decimals fit.
      lcdReadingPage("Free NH3 mg/L", M_FREE_NH3, readings.freeNh3_mgL, 3, "");
      break;
    case P_AIR:
      // Weather, not water: no status of its own (see weather_context.dart).
      lcdTitle(String((char)LCD_DROP) + "Air & Humidity");
      printLine(1, lcdSpread(lcdValue(readings.ambientTemp_C, 1, LCD_DEG "C"),
                             lcdValue(readings.humidity_pct, 0, "%RH")));
      break;
    case P_STATUS: {
      bool online = WiFi.status() == WL_CONNECTED;
      printLine(0, online ? lcdSpread(String((char)LCD_WIFI) + WiFi.SSID(), "")
                          : String((char)LCD_NO_WIFI) + "WiFi offline");
      uint32_t waiting = queueSize();
      if (waiting > 0) {
        printLine(1, String((char)LCD_UPLOAD) + "Saved: " + String(waiting));
      } else {
        printLine(1, String((char)LCD_OK) + (online ? "All synced" : "Nothing saved"));
      }
      break;
    }
  }
}

