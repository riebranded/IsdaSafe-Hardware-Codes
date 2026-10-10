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

void loop() {
  int16_t raw = ads.readADC_SingleEnded(0);
  float voltage_mV = ads.computeVolts(raw) * 1000.0f;
  float tempC = dht.readTemperature();

  Serial.print("raw:\t" + String(raw) + "\tVoltage(mv):\t" + String(voltage_mV, 1));
  if (isnan(tempC)) {
    Serial.println("\tTemp(C):\tread failed");
  } else {
    Serial.println("\tTemp(C):\t" + String(tempC, 1));
  }

  delay(1000);
}
