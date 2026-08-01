/*
  IsdaSafe - Pond Monitoring & Cooling Controller
  MCU: ESP32

  Sensors / Modules:
   - DFRobot Gravity Analog Dissolved Oxygen sensor -> read through an ADS1115
     16-bit I2C ADC module (the sensor itself is analog; ADS1115 digitizes it
     over I2C so it can share the bus with the LCD instead of using a noisy
     ESP32 ADC pin).
   - DFRobot RS485 Water Quality Ammonia Nitrogen & pH Sensor: a single
     integrated probe (slave ID 1) that returns Ammonia, pH AND Water
     Temperature together in one Modbus-RTU read (registers 0x0000-0x0002),
     wired through a Gravity Active Isolated RS485-to-UART module (auto
     direction switching, no DE/RE control pin needed). Sensor itself needs
     its own 10-30V DC supply; only RX/TX/GND are shared with the ESP32.
   - DHT22 for ambient (enclosure/air) temperature & humidity.
   - 16x2 I2C LCD (HD44780 + PCF8574 backpack), powered through a buck
     converter, for rotating on-device readout.
   - DRV8833 dual H-bridge driving a 3V/1000RPM DC motor as a cooling fan,
     PWM-speed-controlled off the water temperature reading.

  Libraries required (install via Library Manager):
   - Adafruit ADS1X15  (Adafruit_ADS1X15.h)
   - LiquidCrystal I2C  (LiquidCrystal_I2C.h)  -- e.g. by Frank de Brabander
   - DHT sensor library  (DHT.h)  -- Adafruit, plus Adafruit Unified Sensor
   - ModbusMaster  (ModbusMaster.h)  -- by Doc Walker
   (WiFi.h / WiFiClientSecure.h / HTTPClient.h ship with the ESP32 board
   package, no separate install needed.)

  Networking:
   - Connects to WiFi and POSTs each poll's readings to the IsdaSafe server
     (Render) at POST /sensor-readings. Set WIFI_SSID/WIFI_PASSWORD below
     before flashing.
*/

#include <Wire.h>
#include <Adafruit_ADS1X15.h>
#include <LiquidCrystal_I2C.h>
#include <DHT.h>
#include <ModbusMaster.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <HTTPClient.h>

// ---------------------------------------------------------------------------
// WiFi + IsdaSafe server (Render)
// Server contract is POST /sensor-readings with a FLAT JSON body:
//   {"dissolved_oxygen":..,"ammonia":..,"temperature":..,"humidity":..,"ph":..}
// (matches app/schemas.py::SensorReading in the IsdaSafe_Server repo exactly --
// no deviceId/timestamp field and no auth header, so none are sent here.
// "temperature" is WATER temperature; the server has no ambient-temp field.)
// ---------------------------------------------------------------------------
#define WIFI_SSID        "YOUR_WIFI_SSID"
#define WIFI_PASSWORD    "YOUR_WIFI_PASSWORD"
#define SERVER_URL       "https://isdasafe-server.onrender.com/sensor-readings"

// Render free tier spins the service down when idle, so the first request
// after a while can take 30-50s to wake it back up -- give HTTP plenty of
// margin instead of timing out and dropping the reading.
#define HTTP_TIMEOUT_MS  45000

// ---------------------------------------------------------------------------
// I2C bus (shared by ADS1115 + LCD)
// ---------------------------------------------------------------------------
#define I2C_SDA_PIN     21
#define I2C_SCL_PIN     22

#define ADS1115_ADDR    0x48   // default DO ADS1115 address (ADDR pin -> GND)
#define LCD_I2C_ADDR    0x27   // common backpack address; try 0x3F if blank
#define LCD_COLS        16
#define LCD_ROWS        2

// ---------------------------------------------------------------------------
// DHT22 (ambient temp/humidity)
// ---------------------------------------------------------------------------
#define DHT_PIN         15
#define DHT_TYPE        DHT22

// ---------------------------------------------------------------------------
// RS485 bus - DFRobot Water Quality Ammonia Nitrogen & pH Sensor
// (wired through the Gravity Active Isolated RS485-to-UART module; that
// module auto-switches transmit/receive direction, so no DE/RE GPIO is
// required)
// ---------------------------------------------------------------------------
#define RS485_RX_PIN     16   // ESP32 <- module Blue/RX wire
#define RS485_TX_PIN     17   // ESP32 -> module Green/TX wire
#define RS485_BAUD       4800 // fixed baud rate for this sensor, per datasheet

#define WQ_SENSOR_SLAVE_ID 1     // sensor's fixed Modbus address
#define WQ_REG_START       0x0000
#define WQ_REG_COUNT       3     // NH3, pH, Temp read in a single transaction

#define AMMONIA_SCALE    100.0f   // raw / 100 -> mg/L
#define PH_SCALE         100.0f   // raw / 100 -> pH
#define WTEMP_SCALE      10.0f    // raw / 10   -> deg C

// ---------------------------------------------------------------------------
// DRV8833 cooling fan (3V 1000RPM DC motor, one channel)
// ---------------------------------------------------------------------------
#define MOTOR_AIN1_PIN   25
#define MOTOR_AIN2_PIN   26
#define MOTOR_STBY_PIN   27   // DRV8833 nSLEEP, tie/drive HIGH to enable

#define PWM_FREQ_HZ      5000
#define PWM_RESOLUTION   8      // 0-255 duty

#define COOLING_ON_TEMP  30.0f  // water temp (C) at which the fan starts
#define COOLING_MAX_TEMP 34.0f  // water temp (C) at which the fan hits 100%
#define COOLING_MIN_DUTY 120    // minimum duty that reliably spins the motor

// ---------------------------------------------------------------------------
// Dissolved Oxygen calibration (DFRobot SEN0237-A method).
// Use the companion "DO_Calibration" sketch to get these voltages: expose
// the wet probe to air (or saturated water), let the reading stabilize, and
// record the printed mV + water temperature.
//
// Single-point: only CAL1_V/CAL1_T are used, with a fixed 35mV/°C slope --
// fine when water temp stays essentially constant.
// Two-point: set DO_TWO_POINT_CALIBRATION to 1 and also fill in
// CAL2_V/CAL2_T (measured at a second, different temperature) -- more
// accurate here since the cooling fan means water temp will actually swing.
// ---------------------------------------------------------------------------
#define DO_TWO_POINT_CALIBRATION 1

#define DO_CAL1_V   1600   // mV reading at calibration point 1
#define DO_CAL1_T   25     // water temp (C) at calibration point 1
#define DO_CAL2_V   1300   // mV reading at calibration point 2 (two-point only)
#define DO_CAL2_T   15     // water temp (C) at calibration point 2 (two-point only)

const float DO_TABLE[41] = {
  14.46, 14.22, 14.00, 13.77, 13.56, 13.35, 13.15, 12.95, 12.75, 12.58,
  12.40, 12.23, 12.06, 11.90, 11.75, 11.59, 11.45, 11.30, 11.16, 11.03,
  10.90, 10.77, 10.65, 10.53, 10.41, 10.30, 10.19, 10.08,  9.98,  9.88,
   9.78,  9.68,  9.58,  9.49,  9.40,  9.31,  9.22,  9.14,  9.05,  8.97,
   8.90
};

// ---------------------------------------------------------------------------
// Globals
// ---------------------------------------------------------------------------
Adafruit_ADS1115 ads;
LiquidCrystal_I2C lcd(LCD_I2C_ADDR, LCD_COLS, LCD_ROWS);
DHT dht(DHT_PIN, DHT_TYPE);

HardwareSerial RS485Serial(2); // UART2
ModbusMaster node;

struct SensorReadings {
  float dissolvedOxygen_mgL = NAN;
  float ph                  = NAN;
  float ammonia_mgL         = NAN;
  float waterTemp_C         = NAN;
  float ambientTemp_C       = NAN;
  float humidity_pct        = NAN;
} readings;

unsigned long lastSensorPoll = 0;
const unsigned long SENSOR_POLL_INTERVAL_MS = 3000;

unsigned long lastLcdSwitch = 0;
const unsigned long LCD_PAGE_INTERVAL_MS = 3000;
uint8_t lcdPage = 0;
const uint8_t LCD_PAGE_COUNT = 3;

// ---------------------------------------------------------------------------
// RS485 water quality sensor (Ammonia + pH + Water Temp in one Modbus read)
// Mirrors the sensor's documented request frame:
//   01 03 00 00 00 03 05 CB  (slave 1, read 3 holding regs from 0x0000)
// and response layout: [addr][func][bytecount][NH3 hi/lo][pH hi/lo][Temp hi/lo][CRC hi/lo]
// ModbusMaster builds/verifies this frame (incl. CRC16) for us.
// ---------------------------------------------------------------------------
bool readWaterQualitySensor() {
  node.begin(WQ_SENSOR_SLAVE_ID, RS485Serial);
  uint8_t result = node.readHoldingRegisters(WQ_REG_START, WQ_REG_COUNT);
  if (result != node.ku8MBSuccess) {
    return false;
  }
  readings.ammonia_mgL = node.getResponseBuffer(0) / AMMONIA_SCALE;
  readings.ph          = node.getResponseBuffer(1) / PH_SCALE;
  readings.waterTemp_C = node.getResponseBuffer(2) / WTEMP_SCALE;
  return true;
}

// ---------------------------------------------------------------------------
// Dissolved Oxygen (ADS1115 + temperature-compensated lookup table)
// ---------------------------------------------------------------------------
float readDissolvedOxygen(float waterTempC) {
  int16_t adcCounts = ads.readADC_SingleEnded(0);
  float voltage_mV = ads.computeVolts(adcCounts) * 1000.0f;

  int tempIndex = (int)constrain(waterTempC, 0, 40);

#if DO_TWO_POINT_CALIBRATION == 1
  float vSaturation = (float)(tempIndex - DO_CAL2_T) * (DO_CAL1_V - DO_CAL2_V)
                       / (DO_CAL1_T - DO_CAL2_T) + DO_CAL2_V;
#else
  float vSaturation = DO_CAL1_V + 35.0f * (tempIndex - DO_CAL1_T);
#endif

  float doValue = (voltage_mV * DO_TABLE[tempIndex]) / vSaturation;
  return doValue;
}

// ---------------------------------------------------------------------------
// Sensor polling
// ---------------------------------------------------------------------------
void pollSensors() {
  if (!readWaterQualitySensor()) {
    Serial.println("RS485 water quality sensor read failed/timed out.");
  }

  float waterTempForCal = isnan(readings.waterTemp_C) ? 25.0f : readings.waterTemp_C;
  readings.dissolvedOxygen_mgL = readDissolvedOxygen(waterTempForCal);

  float h = dht.readHumidity();
  float t = dht.readTemperature();
  if (!isnan(h)) readings.humidity_pct = h;
  if (!isnan(t)) readings.ambientTemp_C = t;
}

// ---------------------------------------------------------------------------
// Send the current readings to the IsdaSafe server (POST /sensor-readings).
// Body field names/types must match app/schemas.py::SensorReading exactly.
// ---------------------------------------------------------------------------
void sendReadingsToServer() {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("WiFi not connected - skipping server upload.");
    return;
  }
  if (isnan(readings.dissolvedOxygen_mgL) || isnan(readings.ammonia_mgL) ||
      isnan(readings.waterTemp_C) || isnan(readings.humidity_pct) || isnan(readings.ph)) {
    Serial.println("Incomplete sensor readings - skipping server upload.");
    return;
  }

  WiFiClientSecure client;
  client.setInsecure(); // no cert store on this MCU; accept any valid TLS cert

  HTTPClient http;
  http.setTimeout(HTTP_TIMEOUT_MS);
  if (!http.begin(client, SERVER_URL)) {
    Serial.println("HTTP begin() failed.");
    return;
  }
  http.addHeader("Content-Type", "application/json");

  char body[192];
  snprintf(body, sizeof(body),
    "{\"dissolved_oxygen\":%.2f,\"ammonia\":%.2f,\"temperature\":%.2f,\"humidity\":%.2f,\"ph\":%.2f}",
    readings.dissolvedOxygen_mgL, readings.ammonia_mgL, readings.waterTemp_C,
    readings.humidity_pct, readings.ph);

  int statusCode = http.POST((uint8_t *)body, strlen(body));
  if (statusCode > 0) {
    Serial.printf("Server upload: HTTP %d\n", statusCode);
  } else {
    Serial.printf("Server upload failed: %s\n", http.errorToString(statusCode).c_str());
  }
  http.end();
}

// ---------------------------------------------------------------------------
// Cooling fan control (DRV8833, one channel, forward-only speed control)
// ---------------------------------------------------------------------------
void controlCooling(float waterTempC) {
  uint8_t duty = 0;

  if (!isnan(waterTempC) && waterTempC >= COOLING_ON_TEMP) {
    float span = COOLING_MAX_TEMP - COOLING_ON_TEMP;
    float frac = (waterTempC - COOLING_ON_TEMP) / span;
    frac = constrain(frac, 0.0f, 1.0f);
    duty = COOLING_MIN_DUTY + (uint8_t)(frac * (255 - COOLING_MIN_DUTY));
  }

  ledcWrite(MOTOR_AIN1_PIN, duty);
  ledcWrite(MOTOR_AIN2_PIN, 0);
}

// ---------------------------------------------------------------------------
// LCD (rotates through pages since a 16x2 can't fit everything at once)
// ---------------------------------------------------------------------------
void printLine(uint8_t row, const String &text) {
  String padded = text;
  while (padded.length() < LCD_COLS) padded += ' ';
  lcd.setCursor(0, row);
  lcd.print(padded.substring(0, LCD_COLS));
}

void updateLcd() {
  switch (lcdPage) {
    case 0:
      printLine(0, "DO: " + String(readings.dissolvedOxygen_mgL, 2) + " mg/L");
      printLine(1, "pH: " + String(readings.ph, 2));
      break;
    case 1:
      printLine(0, "NH3: " + String(readings.ammonia_mgL, 2) + " mg/L");
      printLine(1, "WTemp: " + String(readings.waterTemp_C, 1) + "C");
      break;
    case 2:
      printLine(0, "Air: " + String(readings.ambientTemp_C, 1) + "C");
      printLine(1, "Hum: " + String(readings.humidity_pct, 0) + "%");
      break;
  }
}

// ---------------------------------------------------------------------------
// Setup / Loop
// ---------------------------------------------------------------------------
void setup() {
  Serial.begin(115200);

  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("Connecting to WiFi");
  unsigned long wifiStart = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - wifiStart < 20000) {
    delay(500);
    Serial.print(".");
  }
  Serial.println();
  if (WiFi.status() == WL_CONNECTED) {
    Serial.print("WiFi connected, IP: ");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println("WiFi connect timed out - will keep retrying in background.");
  }

  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);

  if (!ads.begin(ADS1115_ADDR)) {
    Serial.println("ADS1115 not found - check DO sensor wiring/address.");
  }

  lcd.init();
  lcd.backlight();
  printLine(0, "IsdaSafe boot...");
  printLine(1, "");

  dht.begin();

  RS485Serial.begin(RS485_BAUD, SERIAL_8N1, RS485_RX_PIN, RS485_TX_PIN);
  // Active Isolated RS485 module auto-switches TX/RX direction, so no
  // DE/RE pin control (node.preTransmission/postTransmission) is needed.

  pinMode(MOTOR_STBY_PIN, OUTPUT);
  digitalWrite(MOTOR_STBY_PIN, HIGH); // enable DRV8833 driver stage
  ledcAttach(MOTOR_AIN1_PIN, PWM_FREQ_HZ, PWM_RESOLUTION);
  ledcAttach(MOTOR_AIN2_PIN, PWM_FREQ_HZ, PWM_RESOLUTION);

  pollSensors();
  updateLcd();
}

void loop() {
  unsigned long now = millis();

  if (now - lastSensorPoll >= SENSOR_POLL_INTERVAL_MS) {
    lastSensorPoll = now;
    pollSensors();
    controlCooling(readings.waterTemp_C);

    Serial.printf(
      "DO=%.2f mg/L  pH=%.2f  NH3=%.2f mg/L  WaterT=%.1fC  AirT=%.1fC  Hum=%.0f%%\n",
      readings.dissolvedOxygen_mgL, readings.ph, readings.ammonia_mgL,
      readings.waterTemp_C, readings.ambientTemp_C, readings.humidity_pct
    );

    sendReadingsToServer();
  }

  if (now - lastLcdSwitch >= LCD_PAGE_INTERVAL_MS) {
    lastLcdSwitch = now;
    lcdPage = (lcdPage + 1) % LCD_PAGE_COUNT;
    updateLcd();
  }
}
