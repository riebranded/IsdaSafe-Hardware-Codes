/*
  DO_Calibration - ESP32 + ADS1115 version of DFRobot's SEN0237-A
  calibration example (originally analogRead(A1) on an UNO).

  Also prints a live temperature next to the voltage: the WATER
  temperature from the SEN0711 (put it in the same cup as the DO probe for
  saturated-water / two-point calibration), falling back to the DHT22's
  AIR temperature if the SEN0711 doesn't answer (fine for the in-air
  method). Record it as DO_CAL1_T / DO_CAL2_T.

  Use this exactly like the documented UNO procedure:
   1. Upload this sketch, open Serial Monitor at 115200 baud.
   2. Single-point: wet the probe, expose it to air (or saturated water),
      wait for the printed voltage AND temperature to stabilize, then
      record both. Plug them into sketch1_aug1.ino as
      DO_CAL1_V / DO_CAL1_T.
   3. Two-point: repeat step 2 at a second, different temperature (e.g. one
      cup chilled, one cup warmed, both below 40C) and record that as
      DO_CAL2_V / DO_CAL2_T, then set DO_TWO_POINT_CALIBRATION to 1 in
      sketch1_aug1.ino.

  Averaged capture (recommended): once the live voltage looks stable, type
  "1" (calibration point 1) or "2" (point 2) in the Serial Monitor, line
  ending Newline. The sketch then averages voltage and temperature over
  60 s (or "1 120" for 120 s), reports mean / sd / min / max / drift, and
  prints the ready-to-paste #define lines. Keep the probe still meanwhile.

  Wiring: same I2C bus as the main sketch -- ADS1115 SDA/SCL -> ESP32
  GPIO21/GPIO22, DO probe signal -> ADS1115 A0. DHT22 data -> GPIO15
  (same pin as the main sketch).

  Libraries required: Adafruit ADS1X15 (Adafruit_ADS1X15.h), DHT sensor
  library (DHT.h) + Adafruit Unified Sensor.
*/

#include <Wire.h>
#include <Adafruit_ADS1X15.h>
#include <DHT.h>

#define I2C_SDA_PIN  21
#define I2C_SCL_PIN  22
#define ADS1115_ADDR 0x49   // same as sketch1_aug1 (ADDR pin -> VDD); 0x48 if ADDR -> GND

#define DHT_PIN      15
#define DHT_TYPE     DHT22

// SEN0711 (same RS485 wiring as sketch1_aug1): used only for its WATER
// temperature, which is what two-point / saturated-water calibration needs.
// The DHT22 measures AIR and is used only if the SEN0711 doesn't answer.
#define RS485_RX_PIN     16
#define RS485_TX_PIN     17
#define RS485_DE_RE_PIN  32
#define RS485_BAUD       4800
#define WQ_SLAVE_ID      1

Adafruit_ADS1115 ads;
DHT dht(DHT_PIN, DHT_TYPE);
HardwareSerial RS485Serial(1);

static uint16_t modbusCrc16(const uint8_t* d, size_t n) {
  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < n; i++) {
    crc ^= d[i];
    for (int b = 0; b < 8; b++) crc = (crc & 1) ? (crc >> 1) ^ 0xA001 : crc >> 1;
  }
  return crc;
}

// Reads SEN0711 register 0x0002 (water temp x10, signed). NAN if no reply.
float readWaterTemp() {
  uint8_t req[8] = { WQ_SLAVE_ID, 0x03, 0x00, 0x02, 0x00, 0x01 };
  uint16_t crc = modbusCrc16(req, 6);
  req[6] = crc & 0xFF; req[7] = crc >> 8;
  while (RS485Serial.available()) RS485Serial.read();
  digitalWrite(RS485_DE_RE_PIN, HIGH);
  delayMicroseconds(100);
  RS485Serial.write(req, sizeof(req));
  RS485Serial.flush();
  delayMicroseconds(100);
  digitalWrite(RS485_DE_RE_PIN, LOW);

  uint8_t buf[24]; size_t n = 0;
  unsigned long t0 = millis();
  while (millis() - t0 < 300 && n < sizeof(buf)) {
    if (RS485Serial.available()) buf[n++] = RS485Serial.read();
  }
  // Reply: 01 03 02 [T hi] [T lo] [CRC lo] [CRC hi]
  for (size_t i = 0; i + 7 <= n; i++) {
    if (buf[i] == WQ_SLAVE_ID && buf[i + 1] == 0x03 && buf[i + 2] == 0x02 &&
        modbusCrc16(&buf[i], 5) == (uint16_t)(buf[i + 5] | (buf[i + 6] << 8))) {
      return (int16_t)((buf[i + 3] << 8) | buf[i + 4]) / 10.0f;
    }
  }
  return NAN;
}

void setup() {
  Serial.begin(115200);
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);

  pinMode(RS485_DE_RE_PIN, OUTPUT);
  digitalWrite(RS485_DE_RE_PIN, LOW);
  RS485Serial.begin(RS485_BAUD, SERIAL_8N1, RS485_RX_PIN, RS485_TX_PIN);

  if (!ads.begin(ADS1115_ADDR)) {
    Serial.println("ADS1115 not found - check wiring/address.");
  }

  dht.begin();
  Serial.println(">>> DO_Calibration ready. When the voltage is stable, type 1 (point 1) or 2 (point 2)");
  Serial.println(">>> and press Enter to average it over 60 s (e.g. \"1 120\" for 120 s).");
  Serial.println(">>> s = stop readings (also cancels a capture), r = resume.");
}

// ---------------------------------------------------------------------------
// Averaged capture state ("1" / "2" commands)
// ---------------------------------------------------------------------------
#define CAPTURE_DEFAULT_S  60
#define CAPTURE_MAX_DRIFT_MV 10.0f   // first-10s vs last-10s mean; above = not stable

int      capPoint = 0;               // 0 = idle, 1 or 2 = capturing that point
uint16_t capTotal = 0, capN = 0, capTempN = 0, capWaterN = 0;
double   capSumMv = 0, capSumSqMv = 0, capSumT = 0, capSumWaterT = 0;
float    capMinMv = 0, capMaxMv = 0;
double   capFirstSum = 0, capLastSum = 0;
uint16_t capFirstN = 0, capLastN = 0;
String   cmdBuf;
bool     halted = false;             // true after a result: readings paused so it stays visible

void startCapture(int point, int seconds) {
  halted = false;
  capPoint = point;
  capTotal = seconds;
  capN = capTempN = capWaterN = 0;
  capSumMv = capSumSqMv = capSumT = capSumWaterT = 0;
  capFirstSum = capLastSum = 0;
  capFirstN = capLastN = 0;
  Serial.printf(">>> Capturing calibration point %d: averaging for %d s - keep the probe still...\n",
                point, seconds);
}

void finishCapture() {
  int point = capPoint;
  capPoint = 0;
  halted = true;   // stop printing so the result stays on screen (see end of this function)
  if (capN < capTotal / 2) {
    Serial.println(">>> Capture FAILED: too few readings (is the ADS1115 connected?)");
    return;
  }
  float avgMv = capSumMv / capN;
  float var = capSumSqMv / capN - (double)avgMv * avgMv;
  float sd = var > 0 ? sqrtf(var) : 0;
  float drift = (capLastN && capFirstN) ? (capLastSum / capLastN - capFirstSum / capFirstN) : 0;
  Serial.println(">>> ===== CALIBRATION POINT RESULT =====");
  Serial.printf(">>> samples=%u  mean=%.1f mV  sd=%.1f  min=%.1f  max=%.1f  drift=%+.1f mV\n",
                capN, avgMv, sd, capMinMv, capMaxMv, drift);
  if (fabsf(drift) > CAPTURE_MAX_DRIFT_MV) {
    Serial.println(">>> WARNING: voltage still drifting - wait longer and capture again.");
  }
  if (capTempN == 0) {
    Serial.println(">>> Temperature: DHT22 read failed - measure the temperature yourself.");
    Serial.printf(">>> Paste into sketch1_aug1.ino:\n#define DO_CAL%d_V   %ld\n#define DO_CAL%d_T   <temp C>\n",
                  point, lroundf(avgMv), point);
  } else {
    // Never mix air (DHT22 fallback) and water temperatures: if the SEN0711
    // answered at all, average only its water readings.
    float avgT;
    if (capWaterN > 0) {
      avgT = capSumWaterT / capWaterN;
      Serial.printf(">>> mean WATER temperature (SEN0711) = %.1f C  (%u water readings", avgT, capWaterN);
      if (capWaterN < capTempN) Serial.printf("; %u air-temp fallbacks ignored", capTempN - capWaterN);
      Serial.println(")");
    } else {
      avgT = capSumT / capTempN;
      Serial.printf(">>> mean AIR temperature (DHT22) = %.1f C - SEN0711 never answered.\n", avgT);
      Serial.println(">>> For water calibration, measure the water temperature yourself.");
    }
    Serial.printf(">>> Paste into sketch1_aug1.ino:\n#define DO_CAL%d_V   %ld\n#define DO_CAL%d_T   %ld\n",
                  point, lroundf(avgMv), point, lroundf(avgT));
  }
  Serial.println(">>> Readings PAUSED. Type r to resume live readings, or 1 / 2 for another capture.");
  Serial.flush();
}

void runCommand() {
  cmdBuf.trim();
  if (cmdBuf.length()) Serial.println(">>> Received: " + cmdBuf);
  if (cmdBuf.equalsIgnoreCase("r")) {
    halted = false;
    Serial.println(">>> Resumed live readings.");
  } else if (cmdBuf.equalsIgnoreCase("s") || cmdBuf.equalsIgnoreCase("stop")) {
    if (capPoint) Serial.printf(">>> Capture of point %d CANCELLED - nothing recorded.\n", capPoint);
    capPoint = 0;
    halted = true;
    Serial.println(">>> Readings STOPPED. Type r to resume, or 1 / 2 for a capture.");
  } else if (cmdBuf.length()) {
    int point = cmdBuf.toInt();
    int sp = cmdBuf.indexOf(' ');
    int secs = sp > 0 ? cmdBuf.substring(sp + 1).toInt() : CAPTURE_DEFAULT_S;
    if ((point == 1 || point == 2) && secs >= 10 && secs <= 600) {
      startCapture(point, secs);
    } else {
      Serial.println(">>> Commands: 1 / 2 [seconds] = capture point, s = stop, r = resume.");
    }
  }
  cmdBuf = "";
}

// Works with any Serial Monitor line-ending setting: a command runs on
// Enter (newline), or 300 ms after the last character if no newline is sent.
unsigned long lastCharMs = 0;

void pollCommands() {
  while (Serial.available()) {
    char c = (char)Serial.read();
    lastCharMs = millis();
    if (c == '\n' || c == '\r') { runCommand(); continue; }
    if (cmdBuf.length() < 32) cmdBuf += c;
  }
  if (cmdBuf.length() && millis() - lastCharMs > 300) runCommand();
}

// delay() that keeps listening for commands.
void waitPolling(unsigned long ms) {
  unsigned long t0 = millis();
  while (millis() - t0 < ms) { pollCommands(); delay(10); }
}

void loop() {
  pollCommands();
  if (halted) { delay(50); return; }   // result on screen: wait for r / 1 / 2

  int16_t raw = ads.readADC_SingleEnded(0);
  float voltage_mV = ads.computeVolts(raw) * 1000.0f;
  // Water temperature from the SEN0711; air temperature (DHT22) only as a fallback.
  float tempC = readWaterTemp();
  bool tempIsWater = !isnan(tempC);
  if (!tempIsWater) tempC = dht.readTemperature();

  Serial.print("raw:\t" + String(raw) + "\tVoltage(mv):\t" + String(voltage_mV, 1));
  if (isnan(tempC)) {
    Serial.print("\tTemp(C):\tread failed");
  } else {
    Serial.print(String(tempIsWater ? "\tWaterTemp(C):\t" : "\tAirTemp(C):\t") + String(tempC, 1));
  }
  if (capPoint && tempIsWater) { capWaterN++; capSumWaterT += tempC; }

  if (capPoint) {
    if (capN == 0 || voltage_mV < capMinMv) capMinMv = voltage_mV;
    if (capN == 0 || voltage_mV > capMaxMv) capMaxMv = voltage_mV;
    capSumMv += voltage_mV; capSumSqMv += (double)voltage_mV * voltage_mV; capN++;
    if (!isnan(tempC)) { capSumT += tempC; capTempN++; }
    if (capN <= 10) { capFirstSum += voltage_mV; capFirstN++; }
    if (capN > capTotal - 10) { capLastSum += voltage_mV; capLastN++; }
    Serial.printf("\t[capturing P%d %u/%u]", capPoint, capN, capTotal);
    Serial.println();
    if (capN >= capTotal) finishCapture();
  } else {
    Serial.println();
  }

  waitPolling(1000);
}
