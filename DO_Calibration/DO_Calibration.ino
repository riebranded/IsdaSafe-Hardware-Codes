/*
  DO_Calibration - ESP32 + ADS1115 version of DFRobot's SEN0237-A
  calibration example (originally analogRead(A1) on an UNO).

  Also reads the board's DHT22 alongside the DO probe, so you get a live
  temperature reading printed next to the voltage -- no separate
  thermometer needed. Since calibration is done in air (or saturated
  water), the DHT22's ambient temperature is exactly what you want to
  record as DO_CAL1_T / DO_CAL2_T.

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

Adafruit_ADS1115 ads;
DHT dht(DHT_PIN, DHT_TYPE);

void setup() {
  Serial.begin(115200);
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);

  if (!ads.begin(ADS1115_ADDR)) {
    Serial.println("ADS1115 not found - check wiring/address.");
  }

  dht.begin();
}

// ---------------------------------------------------------------------------
// Averaged capture state ("1" / "2" commands)
// ---------------------------------------------------------------------------
#define CAPTURE_DEFAULT_S  60
#define CAPTURE_MAX_DRIFT_MV 10.0f   // first-10s vs last-10s mean; above = not stable

int      capPoint = 0;               // 0 = idle, 1 or 2 = capturing that point
uint16_t capTotal = 0, capN = 0, capTempN = 0;
double   capSumMv = 0, capSumSqMv = 0, capSumT = 0;
float    capMinMv = 0, capMaxMv = 0;
double   capFirstSum = 0, capLastSum = 0;
uint16_t capFirstN = 0, capLastN = 0;
String   cmdBuf;
bool     halted = false;             // true after a result: readings paused so it stays visible

void startCapture(int point, int seconds) {
  halted = false;
  capPoint = point;
  capTotal = seconds;
  capN = capTempN = 0;
  capSumMv = capSumSqMv = capSumT = 0;
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
    float avgT = capSumT / capTempN;
    Serial.printf(">>> mean temperature (DHT22 = air) = %.1f C\n", avgT);
    Serial.println(">>> (Saturated-water method: use the WATER temperature instead.)");
    Serial.printf(">>> Paste into sketch1_aug1.ino:\n#define DO_CAL%d_V   %ld\n#define DO_CAL%d_T   %ld\n",
                  point, lroundf(avgMv), point, lroundf(avgT));
  }
  Serial.println(">>> Readings PAUSED. Type r to resume live readings, or 1 / 2 for another capture.");
  Serial.flush();
}

void pollCommands() {
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c != '\n' && c != '\r') { if (cmdBuf.length() < 32) cmdBuf += c; continue; }
    cmdBuf.trim();
    if (cmdBuf.equalsIgnoreCase("r")) {
      halted = false;
      Serial.println(">>> Resumed live readings.");
    } else if (cmdBuf.length()) {
      int point = cmdBuf.toInt();
      int sp = cmdBuf.indexOf(' ');
      int secs = sp > 0 ? cmdBuf.substring(sp + 1).toInt() : CAPTURE_DEFAULT_S;
      if ((point == 1 || point == 2) && secs >= 10 && secs <= 600) {
        startCapture(point, secs);
      } else {
        Serial.println(">>> Type 1 or 2 (optionally followed by seconds, e.g. \"1 120\").");
      }
    }
    cmdBuf = "";
  }
}

void loop() {
  pollCommands();
  if (halted) { delay(50); return; }   // result on screen: wait for r / 1 / 2

  int16_t raw = ads.readADC_SingleEnded(0);
  float voltage_mV = ads.computeVolts(raw) * 1000.0f;
  float tempC = dht.readTemperature();

  Serial.print("raw:\t" + String(raw) + "\tVoltage(mv):\t" + String(voltage_mV, 1));
  if (isnan(tempC)) {
    Serial.print("\tTemp(C):\tread failed");
  } else {
    Serial.print("\tTemp(C):\t" + String(tempC, 1));
  }

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

  delay(1000);
}
