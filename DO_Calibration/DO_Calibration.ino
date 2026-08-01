/*
  DO_Calibration - ESP32 + ADS1115 version of DFRobot's SEN0237-A
  calibration example (originally analogRead(A1) on an UNO).

  Use this exactly like the documented UNO procedure:
   1. Upload this sketch, open Serial Monitor at 115200 baud.
   2. Single-point: wet the probe, expose it to air (or saturated water),
      wait for the printed voltage to stabilize, then record that
      Voltage(mv) and the water/air temperature. Plug them into
      sketch1_aug1.ino as DO_CAL1_V / DO_CAL1_T.
   3. Two-point: repeat step 2 at a second, different temperature (e.g. one
      cup chilled, one cup warmed, both below 40C) and record that as
      DO_CAL2_V / DO_CAL2_T, then set DO_TWO_POINT_CALIBRATION to 1 in
      sketch1_aug1.ino.

  Wiring: same I2C bus as the main sketch -- ADS1115 SDA/SCL -> ESP32
  GPIO21/GPIO22, DO probe signal -> ADS1115 A0.

  Library required: Adafruit ADS1X15 (Adafruit_ADS1X15.h)
*/

#include <Wire.h>
#include <Adafruit_ADS1X15.h>

#define I2C_SDA_PIN  21
#define I2C_SCL_PIN  22
#define ADS1115_ADDR 0x48

Adafruit_ADS1115 ads;

void setup() {
  Serial.begin(115200);
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);

  if (!ads.begin(ADS1115_ADDR)) {
    Serial.println("ADS1115 not found - check wiring/address.");
  }
}

void loop() {
  int16_t raw = ads.readADC_SingleEnded(0);
  float voltage_mV = ads.computeVolts(raw) * 1000.0f;

  Serial.println("raw:\t" + String(raw) + "\tVoltage(mv):\t" + String(voltage_mV, 1));
  delay(1000);
}
