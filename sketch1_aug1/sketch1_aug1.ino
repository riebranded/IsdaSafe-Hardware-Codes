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
     wired through a MAX3485 TTL-to-RS485 module (ESP32 drives its DE/RE
     pins for direction). Per the DFRobot SEN0711 wiki: sensor is 10-30V DC
     (0.3W) from its OWN supply (the MAX3485 module cannot power it); its
     wires are Brown=VCC, Black=GND, Yellow=485-A, Blue=485-B. The sensor
     supply GND must be common with the ESP32 GND.
     Serial: 4800 baud, 8N1, Modbus-RTU, slave ID 1, FC 0x03.
   - DHT22 for ambient (enclosure/air) temperature & humidity.
   - 16x2 I2C LCD (HD44780 + PCF8574 backpack), powered through a buck
     converter, for rotating on-device readout.
   - DRV8833 dual H-bridge driving a 3V/1000RPM DC motor as a cooling fan,
     PWM-speed-controlled off the water temperature reading.

  Libraries required (install via Library Manager):
   - Adafruit ADS1X15  (Adafruit_ADS1X15.h)
   - LiquidCrystal I2C  (LiquidCrystal_I2C.h)  -- e.g. by Frank de Brabander
   - DHT sensor library  (DHT.h)  -- Adafruit, plus Adafruit Unified Sensor
   (WiFi.h / WiFiClientSecure.h / HTTPClient.h ship with the ESP32 board
   package, no separate install needed.)

  Networking:
   - Connects to WiFi and uploads each reading to Supabase
     (ingest_sensor_readings RPC + this device's token or factory secret,
     see isdasafe_pairing.h). The device must first be paired to an account
     (QR claim, or PAIR over USB).
   - While offline, readings are kept in flash with the time they were
     taken and uploaded once it's back online (isdasafe_queue.h).
   - The WiFi network is chosen at run time and saved in flash
     (isdasafe_wifi.h): from a phone via the "IsdaSafe-XXXX" setup hotspot
     (opens when no saved network works, after pairing, or when BOOT is
     held 3 s), or by the website over USB (WIFI ... serial commands).
   - SUPABASE_ANON_KEY (and an optional default WIFI_SSID / WIFI_PASSWORD)
     live in secrets.h (not in git). Copy secrets.example.h to secrets.h.

  Duty cycle:
   - Readings are taken once every 15 minutes. After a reading is captured,
     displayed on the LCD, and uploaded, the board powers down its
     peripherals and drops into ESP32 deep sleep for the remainder of the
     15-minute window, then wakes itself back up via the RTC timer to take
     the next reading. This is far lower-power than polling continuously.
*/

#include <Wire.h>
#include <Adafruit_ADS1X15.h>
#include <LiquidCrystal_I2C.h>
#include <DHT.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <HTTPClient.h>
#include <esp_sleep.h>
#include "secrets.h"            // SUPABASE_ANON_KEY, optional default WIFI_SSID/WIFI_PASSWORD (git-ignored)
#include "isdasafe_pairing.h"   // deviceId(), PAIR/UNPAIR commands, uploadReading()
#include "isdasafe_wifi.h"      // WiFi setup hotspot + WIFI ... commands
#include "isdasafe_queue.h"     // submitReading(): keeps readings while offline

// ---------------------------------------------------------------------------
// I2C buses -- ADS1115 and LCD are on SEPARATE physical I2C buses (each on
// its own SDA/SCL pins), using the ESP32's two hardware I2C peripherals:
//   - ADS1115 -> Wire1 (I2C_SDA_PIN / I2C_SCL_PIN)
//   - LCD     -> Wire  (LCD_SDA_PIN / LCD_SCL_PIN) -- the LiquidCrystal_I2C
//     library only supports the default global Wire bus, so the LCD gets it
//     and the ADS1115 (which Adafruit's library lets us point at any TwoWire
//     instance) is moved to Wire1 instead.
// ---------------------------------------------------------------------------
#define I2C_SDA_PIN     21   // ADS1115 bus (Wire1)
#define I2C_SCL_PIN     22

#define LCD_SDA_PIN     25   // LCD bus (Wire)
#define LCD_SCL_PIN     27

#define ADS1115_ADDR    0x49   // I2C scan found the chip here (ADDR pin -> VDD); use 0x48 if ADDR -> GND
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
// (wired through a MAX3485 TTL-to-RS485 module; direction is controlled by
// the ESP32 via RS485_DE_RE_PIN below)
//
// DROPPED FOR NOW: set to 0 while this code path is unused, to stop the
// LCD/server from carrying stale "nan" pH/Ammonia/WaterTemp readings.
// Set back to 1 once you want RS485 active again. NOTE: the TTL module's
// wires are physically connected to GPIO16/17 whether or not this flag is
// on, so those two pins are NOT free for other uses either way.
// ---------------------------------------------------------------------------
#define ENABLE_RS485     1

// NOTE: DFRobot's wiki example for ESP32 uses RX=GPIO16 (adapter BLUE wire,
// labeled "RX") and TX=GPIO17 (adapter GREEN wire, labeled "TX") -- the wire
// labels are from the MCU's point of view, so they are NOT crossed. If your
// wires are the other way round, the boot-time auto-diagnose will detect it.
#define RS485_RX_PIN     16   // ESP32 RX pin (currently set to match your physical wiring)
#define RS485_TX_PIN     17   // ESP32 TX pin

// MAX3485 TTL-to-RS485 module: it does NOT switch direction by itself, so the
// ESP32 drives DE and RE (tie the module's DE and RE pins together and wire
// them to this GPIO): HIGH = transmit, LOW = receive.
// Set RS485_USE_DE_RE to 0 only if your module is the 4-pin auto-direction
// type (VCC/GND/TXD/RXD, no DE/RE pins).
#define RS485_USE_DE_RE  1   // MAX485 module: DE+RE bridged together and wired to RS485_DE_RE_PIN
#define RS485_DE_RE_PIN  32   // any free output-capable GPIO
#define RS485_BAUD       4800 // fixed baud rate for this sensor, per datasheet

#define WQ_SENSOR_SLAVE_ID 1     // sensor's fixed Modbus address
#define WQ_REG_START       0x0000
#define WQ_REG_COUNT       3     // NH3, pH, Temp read in a single transaction

#define AMMONIA_SCALE    100.0f   // raw / 100 -> mg/L
#define PH_SCALE         100.0f   // raw / 100 -> pH
#define WTEMP_SCALE      10.0f    // raw / 10   -> deg C

// ---------------------------------------------------------------------------
// DRV8833 cooling fan (3V 1000RPM DC motor, one channel)
// Wiring (board with only IN1/IN2 inputs -- its sleep/enable is tied on
// internally, so nothing else is needed):
//   ESP32 GPIO18 -> IN1      ESP32 GPIO2 -> IN2
//   Motor        -> OUT1 / OUT2
//   VCC/VM       -> 5 V (or 3.3 V) supply,  GND -> ESP32 GND (must be shared)
// With FAN_ALWAYS_ON=1 the fan runs constantly; with 0 it runs automatically
// only when water >= COOLING_ON_TEMP. To test it
// at any temperature type FAN ON / FAN 50 (percent) / FAN OFF / FAN AUTO in
// the Serial Monitor. MOTOR_STBY_CONNECTED is only for boards that expose a
// Sleep/STBY pin.
// ---------------------------------------------------------------------------
#define MOTOR_AIN1_PIN   18   // -> DRV8833 IN1 (moved off GPIO17 -- that pin is taken by the TTL module's TX wire)
#define MOTOR_AIN2_PIN   2   // -> DRV8833 IN2
#define MOTOR_STBY_CONNECTED 0
#define MOTOR_STBY_PIN   4    // -> DRV8833 Sleep, only used if MOTOR_STBY_CONNECTED=1

#define PWM_FREQ_HZ      5000
#define PWM_RESOLUTION   8      // 0-255 duty

#define COOLING_ON_TEMP  30.0f  // water temp (C) at which the fan starts
#define COOLING_MAX_TEMP 34.0f  // water temp (C) at which the fan hits 100%
#define COOLING_MIN_DUTY 120    // minimum duty that reliably spins the motor

// 1 = fan runs all the time at FAN_ALWAYS_ON_DUTY, from boot onward, ignoring
// water temperature. 0 = run only when water >= COOLING_ON_TEMP.
// (FAN OFF / FAN <percent> in the Serial Monitor still override it until
// FAN AUTO.)
#define FAN_ALWAYS_ON       1
#define FAN_ALWAYS_ON_DUTY  255  // 0-255, must be >= COOLING_MIN_DUTY to actually spin

// Spins the fan briefly at boot so you can confirm the motor and wiring work.
#define FAN_BOOT_TEST_MS    2000
// A DC motor often won't start from standstill at low duty; give it this
// long at full power first, then drop to the requested duty.
#define FAN_KICK_MS         150

int fanOverride = -1;      // -1 = automatic (temperature); 0-255 = FAN command
uint8_t fanDuty = 0;       // duty currently applied to IN1

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

#define DO_CAL1_V   555
#define DO_CAL1_T   26
#define DO_CAL2_V   365
#define DO_CAL2_T   19

// Catches a bad calibration before it's uploaded. In saturated water the
// probe's voltage rises with temperature, by roughly 20-35 mV per degree C
// (DFRobot uses 35), so the warmer point must read higher, and by 10-50 mV
// per degree. A point taken in water that wasn't saturated, or before the
// probe settled, fails this: redo that point with DO_Calibration.
static_assert(DO_CAL1_V >= 100 && DO_CAL1_V <= 3000,
              "DO_CAL1_V should be 100-3000 mV: redo calibration point 1");
#if DO_TWO_POINT_CALIBRATION == 1
static_assert(DO_CAL2_V >= 100 && DO_CAL2_V <= 3000,
              "DO_CAL2_V should be 100-3000 mV: redo calibration point 2");
static_assert(DO_CAL1_T - DO_CAL2_T >= 5 || DO_CAL2_T - DO_CAL1_T >= 5,
              "DO calibration points need water temperatures at least 5 C apart");
static_assert((DO_CAL1_V - DO_CAL2_V) * (DO_CAL1_T - DO_CAL2_T) > 0,
              "DO calibration: the warmer point must have the higher mV - redo the point that looks off");
// mV per degree = dV/dT; written as dV*dT vs k*dT^2 to stay in integers.
static_assert((DO_CAL1_V - DO_CAL2_V) * (DO_CAL1_T - DO_CAL2_T) >=
                  10 * (DO_CAL1_T - DO_CAL2_T) * (DO_CAL1_T - DO_CAL2_T),
              "DO calibration: the mV change between the points is too small (under 10 mV/C) - redo the point that looks off");
static_assert((DO_CAL1_V - DO_CAL2_V) * (DO_CAL1_T - DO_CAL2_T) <=
                  50 * (DO_CAL1_T - DO_CAL2_T) * (DO_CAL1_T - DO_CAL2_T),
              "DO calibration: the mV change between the points is too big (over 50 mV/C) - redo the point that looks off");
#endif

// Saturated DO (mg/L) in fresh water at 1 atm, index = temperature 0-40 C.
// Same values as DFRobot's SEN0237 example (which stores them in ug/L) and
// standard O2 solubility tables (e.g. 20 C 9.08, 25 C 8.25, 30 C 7.56).
const float DO_TABLE[41] = {
  14.46, 14.22, 13.82, 13.44, 13.09, 12.74, 12.42, 12.11, 11.81, 11.53,
  11.26, 11.01, 10.77, 10.53, 10.30, 10.08,  9.86,  9.66,  9.46,  9.27,
   9.08,  8.90,  8.73,  8.57,  8.41,  8.25,  8.11,  7.96,  7.82,  7.69,
   7.56,  7.43,  7.30,  7.18,  7.07,  6.95,  6.84,  6.73,  6.63,  6.53,
   6.41
};

// ---------------------------------------------------------------------------
// Globals
// ---------------------------------------------------------------------------
Adafruit_ADS1115 ads;
LiquidCrystal_I2C lcd(LCD_I2C_ADDR, LCD_COLS, LCD_ROWS);
DHT dht(DHT_PIN, DHT_TYPE);

HardwareSerial RS485Serial(1); // UART1 (same as DFRobot's SEN0711 ESP32 example; pins remapped to 16/17)

struct SensorReadings {
  float dissolvedOxygen_mgL = NAN;
  float ph                  = NAN;
  float ammonia_mgL         = NAN;
  float waterTemp_C         = NAN;
  float ambientTemp_C       = NAN;
  float humidity_pct        = NAN;
  float freeNh3_mgL         = NAN;   // un-ionized NH3, computed from NH3-N + pH + temp
} readings;

// Which LCD page is showing (see updateLcd(); the pages depend on what's
// connected and whether anything needs attention).
uint8_t lcdPage = 0;
const unsigned long LCD_PAGE_DISPLAY_MS = 4000; // time to show each page

// ---------------------------------------------------------------------------
// Prints every pin/address assignment straight from the #defines above, so
// it can never drift out of sync with what the firmware is actually using
// (unlike a separate reference doc). Use this to cross-check against your
// physical wiring when a sensor isn't responding.
// ---------------------------------------------------------------------------
void printPinConfig() {
  Serial.println(F("===== IsdaSafe Pin Configuration ====="));
  Serial.printf("I2C bus (Wire1): SDA=GPIO%d  SCL=GPIO%d -> ADS1115 (DO), addr 0x%02X, signal on channel A0\n",
                I2C_SDA_PIN, I2C_SCL_PIN, ADS1115_ADDR);
  Serial.printf("I2C bus (Wire):  SDA=GPIO%d  SCL=GPIO%d -> LCD 16x2, addr 0x%02X\n",
                LCD_SDA_PIN, LCD_SCL_PIN, LCD_I2C_ADDR);
  Serial.printf("DHT22 (air):     data=GPIO%d\n", DHT_PIN);
#if ENABLE_RS485
  Serial.printf("RS485 (NH3/pH/WaterTemp): RX=GPIO%d  TX=GPIO%d  baud=%d  slaveID=%d\n",
                RS485_RX_PIN, RS485_TX_PIN, RS485_BAUD, WQ_SENSOR_SLAVE_ID);
#else
  Serial.println(F("RS485 (NH3/pH/WaterTemp): DISABLED (ENABLE_RS485=0)"));
#endif
#if MOTOR_STBY_CONNECTED
  Serial.printf("Motor (DRV8833): IN1=GPIO%d  IN2=GPIO%d  Sleep=GPIO%d\n",
                MOTOR_AIN1_PIN, MOTOR_AIN2_PIN, MOTOR_STBY_PIN);
#else
  Serial.printf("Motor (DRV8833): IN1=GPIO%d  IN2=GPIO%d  Sleep=unconnected (onboard pull-up)\n",
                MOTOR_AIN1_PIN, MOTOR_AIN2_PIN);
#endif
  Serial.println(F("======================================="));
}

// ---------------------------------------------------------------------------
// Deep sleep duty cycle: one reading every READING_INTERVAL_MIN minutes.
// The ESP32 wakes via its RTC timer, which re-runs setup() from scratch
// (deep sleep resets the chip), so all the "take a reading" work lives in
// setup() and ends with goToSleep().
//
// TESTING: set ENABLE_DEEP_SLEEP to 0 to skip deep sleep entirely while
// verifying sensors/wiring -- the board stays powered and loop() re-polls
// every TEST_LOOP_INTERVAL_MS instead, so you can watch readings live in
// the Serial Monitor without waiting 15 minutes or resetting the board.
// Set it back to 1 before deploying for real.
// ---------------------------------------------------------------------------
#define ENABLE_DEEP_SLEEP       0
#define TEST_LOOP_INTERVAL_MS   2000  // continuous: one reading every 2 s (DHT22 needs >= 2 s, SEN0711 >= 200 ms)

// TESTING: set ENABLE_UPLOAD to 0 to read sensors and print to Serial/LCD
// only, without sending anything to Supabase. Set back to 1 when deploying.
#define ENABLE_UPLOAD           1
// Continuous mode (ENABLE_DEEP_SLEEP 0) reads every 2 s; uploading that often
// would flood the database, so only one reading per UPLOAD_INTERVAL_MS is
// sent (the first one right after boot). Deep-sleep mode uploads every wake.
#define UPLOAD_INTERVAL_MS      60000UL

#define READING_INTERVAL_MIN   1

// DUTY CYCLE: 1 = power on -> sensors warm up for DUTY_WARMUP_MIN (readings
// taken but NOT uploaded, LCD counts down) -> take THE reading and upload it
// -> deep sleep DUTY_SLEEP_MIN -> wake and repeat. Every wake warms up
// again, since the probes need time to stabilise. Overrides
// ENABLE_DEEP_SLEEP and READING_INTERVAL_MIN above. 0 = as configured above.
#define ENABLE_DUTY_CYCLE       0
#define DUTY_WARMUP_MIN         5   // awake, sensors stabilising, before the reading
#define DUTY_SLEEP_MIN          5   // deep sleep after the reading

#if ENABLE_DUTY_CYCLE
#undef  ENABLE_DEEP_SLEEP
#define ENABLE_DEEP_SLEEP       0
#undef  READING_INTERVAL_MIN
#define READING_INTERVAL_MIN    DUTY_SLEEP_MIN
#endif

#define uS_TO_S_FACTOR          1000000ULL
#define SLEEP_DURATION_US       ((uint64_t)READING_INTERVAL_MIN * 60 * uS_TO_S_FACTOR)

// ---------------------------------------------------------------------------
// RS485 water quality sensor (Ammonia + pH + Water Temp in one Modbus read)
// Mirrors the sensor's documented request frame:
//   01 03 00 00 00 03 05 CB  (slave 1, read 3 holding regs from 0x0000)
// and response layout: [addr][func][bytecount][NH3 hi/lo][pH hi/lo][Temp hi/lo][CRC hi/lo]
// Same approach as DFRobot's SEN0711 example: send the fixed request, flush
// stale RX bytes and RE-SEND every 300 ms for up to 1.5 s, and parse the reply
// byte-by-byte (01 03 06 + 6 data bytes + CRC) with our own CRC16 check.
// ---------------------------------------------------------------------------
#if ENABLE_RS485
// Slave ID actually used for reads. Starts at the configured value; the boot
// probe's ID scan updates it if the sensor answers at a different address.
// (For ID 1 the request is exactly 01 03 00 00 00 03 05 CB.)
static uint8_t wqSlaveId = WQ_SENSOR_SLAVE_ID;
// DFRobot FAQ: wait > 200 ms for a reply. Re-sending sooner throws away a
// slow reply (RX is flushed on each send) and can collide with it on the bus.
static const unsigned long WQ_TOTAL_TIMEOUT_MS = 1500;
static const unsigned long WQ_RESEND_MS        = 300;

static uint16_t modbusCrc16(const uint8_t* d, size_t n);   // defined below

// Transmit a frame, driving the MAX3485 DE/RE pin: HIGH while sending, then
// back to LOW (receive) as soon as the last byte has left the UART.
static void rs485Send(const uint8_t* buf, size_t len) {
#if RS485_USE_DE_RE
  digitalWrite(RS485_DE_RE_PIN, HIGH);
  delayMicroseconds(100);
#endif
  RS485Serial.write(buf, len);
  RS485Serial.flush();            // wait until the last byte is fully shifted out
#if RS485_USE_DE_RE
  delayMicroseconds(100);
  digitalWrite(RS485_DE_RE_PIN, LOW);
#endif
}

static size_t wqRxBytesSeen = 0;   // debug: any byte at all heard during a read
static uint8_t wqRxDump[24];       // debug: first bytes heard, printed on failure
static size_t wqRxDumpLen = 0;

static size_t rs485ReadN(uint8_t* buf, size_t len, unsigned long timeoutMs) {
  size_t got = 0;
  unsigned long t0 = millis();
  while (got < len && millis() - t0 < timeoutMs) {
    if (RS485Serial.available()) {
      buf[got] = RS485Serial.read();
      if (wqRxDumpLen < sizeof(wqRxDump)) wqRxDump[wqRxDumpLen++] = buf[got];
      got++;
      wqRxBytesSeen++;
    }
  }
  return got;
}

// ---------------------------------------------------------------------------
// Spike filter: NH3 and pH are reported as the MEDIAN of the last
// WQ_MEDIAN_WINDOW good readings, so a single short spike (someone touching
// the wiring/water) never shows up, while a real change comes through once
// it holds for more than half the window (~3 readings, ~6 s at 2 s/reading).
// Temperature is not filtered (it changes slowly and doesn't spike).
// The history is cleared whenever the sensor's offset/calibration changes.
// ---------------------------------------------------------------------------
#define ENABLE_SPIKE_FILTER  1
#define WQ_MEDIAN_WINDOW     5

static float  wqNh3Hist[WQ_MEDIAN_WINDOW];
static float  wqPhHist[WQ_MEDIAN_WINDOW];
static size_t wqHistCount = 0, wqHistNext = 0;

static void wqFilterReset() { wqHistCount = 0; wqHistNext = 0; }

// Last unfiltered values straight from the sensor (for frozen-value check).
static float wqLastRawNh3 = NAN, wqLastRawPh = NAN;

static void wqFilterPush(float nh3, float ph) {
  wqNh3Hist[wqHistNext] = nh3;
  wqPhHist[wqHistNext]  = ph;
  wqHistNext = (wqHistNext + 1) % WQ_MEDIAN_WINDOW;
  if (wqHistCount < WQ_MEDIAN_WINDOW) wqHistCount++;
}

static float wqMedian(const float* hist) {
  float s[WQ_MEDIAN_WINDOW];
  for (size_t i = 0; i < wqHistCount; i++) s[i] = hist[i];
  for (size_t i = 1; i < wqHistCount; i++) {          // insertion sort, n <= 5
    float v = s[i]; size_t j = i;
    while (j > 0 && s[j - 1] > v) { s[j] = s[j - 1]; j--; }
    s[j] = v;
  }
  return (wqHistCount % 2) ? s[wqHistCount / 2]
                           : (s[wqHistCount / 2 - 1] + s[wqHistCount / 2]) / 2.0f;
}

bool readWaterQualitySensor() {
  uint8_t data[11] = {0};
  uint8_t ch = 0;
  unsigned long start = millis();
  unsigned long lastTx = 0;
  bool sentOnce = false;
  int crcFails = 0;
  bool ok = false;
  wqRxBytesSeen = 0;
  wqRxDumpLen = 0;

  uint8_t request[8] = { wqSlaveId, 0x03, 0x00, 0x00, 0x00, 0x03, 0, 0 };
  uint16_t reqCrc = modbusCrc16(request, 6);
  request[6] = reqCrc & 0xFF;
  request[7] = reqCrc >> 8;

  while (millis() - start < WQ_TOTAL_TIMEOUT_MS) {
    pollSerialCommands();
    if (!sentOnce || millis() - lastTx > WQ_RESEND_MS) {
      while (RS485Serial.available() > 0) RS485Serial.read();   // drop stale bytes
      rs485Send(request, sizeof(request));
      lastTx = millis();
      sentOnce = true;
    }

    // Frame: 01 03 06 [NH3 hi lo] [pH hi lo] [Temp hi lo] [CRC lo hi]
    if (rs485ReadN(&ch, 1, 20) != 1 || ch != wqSlaveId) continue;
    data[0] = ch;
    if (rs485ReadN(&data[1], 2, 100) != 2 || data[1] != 0x03 || data[2] != 0x06) continue;
    if (rs485ReadN(&data[3], 8, 100) != 8) continue;

    uint16_t crcCalc = modbusCrc16(data, 9);
    uint16_t crcRecv = (uint16_t)(data[9] | (data[10] << 8));
    if (crcCalc != crcRecv) {
      crcFails++;
      continue;
    }

    uint16_t rawNh3 = (data[3] << 8) | data[4];
    uint16_t rawPh  = (data[5] << 8) | data[6];
    int16_t  rawTmp = (int16_t)((data[7] << 8) | data[8]);   // temperature is SIGNED
    Serial.printf("[DEBUG] SEN0711 raw regs: 0x%04X 0x%04X 0x%04X (NH3 x100, pH x100, Temp x10)\n",
                  rawNh3, rawPh, (uint16_t)rawTmp);
    readings.ammonia_mgL = rawNh3 / AMMONIA_SCALE;
    readings.ph          = rawPh  / PH_SCALE;
    readings.waterTemp_C = rawTmp / WTEMP_SCALE;
    wqLastRawNh3 = readings.ammonia_mgL;
    wqLastRawPh  = readings.ph;
    ok = true;
    break;
  }

  if (!ok) {
    // Don't keep showing/uploading the previous reading as if it were live.
    readings.ammonia_mgL = NAN;
    readings.ph          = NAN;
    readings.waterTemp_C = NAN;
    if (wqRxBytesSeen == 0) {
      Serial.printf("[DEBUG] SEN0711 TIMEOUT: 0 bytes received (RX=GPIO%d TX=GPIO%d %d 8N1) - check sensor/module power, A/B, RX/TX wires\n",
                    RS485_RX_PIN, RS485_TX_PIN, RS485_BAUD);
    } else {
      Serial.printf("[DEBUG] SEN0711 TIMEOUT: heard %u byte(s) but no valid frame (CRC failures=%d) - check A/B swap, GND, noise, slave ID\n",
                    (unsigned)wqRxBytesSeen, crcFails);
      Serial.print("[DEBUG] SEN0711 first bytes heard: ");
      for (size_t i = 0; i < wqRxDumpLen; i++) Serial.printf("%02X ", wqRxDump[i]);
      Serial.println();
      Serial.println("[DEBUG]   (01 03 00 00 00 03 05 CB repeating = echo of OUR request: the module + wiring work,");
      Serial.println("[DEBUG]    but the SENSOR is silent -> sensor power, A/B swap, common GND, slave ID/baud;");
      Serial.println("[DEBUG]    FF/00/random = floating or noisy line, wrong baud, or A/B problem)");
    }
    return false;
  }

#if ENABLE_SPIKE_FILTER
  float rawNh3mg = readings.ammonia_mgL, rawPhVal = readings.ph;
  wqFilterPush(rawNh3mg, rawPhVal);
  readings.ammonia_mgL = wqMedian(wqNh3Hist);
  readings.ph          = wqMedian(wqPhHist);
  if (fabsf(rawNh3mg - readings.ammonia_mgL) >= 0.01f || fabsf(rawPhVal - readings.ph) >= 0.01f) {
    Serial.printf("[DEBUG] spike filter: raw NH3=%.2f pH=%.2f -> median NH3=%.2f pH=%.2f (%u samples)\n",
                  rawNh3mg, rawPhVal, readings.ammonia_mgL, readings.ph, (unsigned)wqHistCount);
  }
#endif

  // Datasheet ranges: NH3 0-100 mg/L, pH 0-14, temp 0-60 C.
  if (readings.ammonia_mgL > 100.0f || readings.ph > 14.0f ||
      readings.waterTemp_C < 0.0f || readings.waterTemp_C > 60.0f) {
    Serial.printf("[DEBUG] RS485 values outside sensor range (NH3=%.2f pH=%.2f T=%.1f) - probe out of water or bad data\n",
                  readings.ammonia_mgL, readings.ph, readings.waterTemp_C);
  }

  Serial.printf("RS485 reading: Ammonia=%.2f mg/L  pH=%.2f  WaterTemp=%.1f C\n",
                readings.ammonia_mgL, readings.ph, readings.waterTemp_C);
  return true;
}

// ---------------------------------------------------------------------------
// RS485 auto-diagnose. Sends the raw Modbus request and prints every byte
// that comes back, trying swapped RX/TX and other common baud rates. On a
// valid (CRC-checked) reply it adopts that RX/TX/baud for the rest of the
// run and prints the #define values to make permanent.
// ---------------------------------------------------------------------------
static uint16_t modbusCrc16(const uint8_t* d, size_t n) {
  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < n; i++) {
    crc ^= d[i];
    for (int b = 0; b < 8; b++) crc = (crc & 1) ? (crc >> 1) ^ 0xA001 : crc >> 1;
  }
  return crc;
}

static size_t rs485Probe(int rx, int tx, unsigned long baud, uint8_t* buf, size_t cap, bool &valid,
                         uint8_t id = WQ_SENSOR_SLAVE_ID, unsigned long waitMs = 600,
                         bool reinit = true) {
  if (reinit) {
    RS485Serial.end();
    RS485Serial.begin(baud, SERIAL_8N1, rx, tx);
    delay(50);
  }
  while (RS485Serial.available()) RS485Serial.read();

  uint8_t req[8] = {id, 0x03, 0x00, 0x00, 0x00, 0x03, 0, 0};
  uint16_t crc = modbusCrc16(req, 6);
  req[6] = crc & 0xFF;
  req[7] = crc >> 8;
  rs485Send(req, 8);

  size_t n = 0;
  unsigned long t0 = millis();
  while (millis() - t0 < waitMs) {
    while (RS485Serial.available() && n < cap) buf[n++] = RS485Serial.read();
    pollSerialCommands();   // the full boot probe takes ~8 s; keep pairing responsive
    delay(2);
  }
  // A real reply is 01 03 06 + 6 data bytes + CRC (11 bytes). Our own request
  // (01 03 00 00 00 03 05 CB) has a valid CRC too, so an ECHO must not count:
  // search the buffer for the 01 03 06 header instead of trusting byte 0.
  valid = false;
  for (size_t i = 0; i + 11 <= n && !valid; i++) {
    if (buf[i] == id && buf[i + 1] == 0x03 && buf[i + 2] == 0x06 &&
        modbusCrc16(&buf[i], 9) == (uint16_t)(buf[i + 9] | (buf[i + 10] << 8))) {
      valid = true;
    }
  }
  return n;
}

bool rs485AutoDiagnose() {
  const unsigned long bauds[] = {RS485_BAUD, 9600, 2400, 19200};
  uint8_t buf[32];
  bool heardAnything = false;

  Serial.println("[DEBUG] RS485 auto-diagnose: probing sensor (slave ID " + String(WQ_SENSOR_SLAVE_ID) + ")...");
  for (int swap = 0; swap < 2; swap++) {
    int rx = swap ? RS485_TX_PIN : RS485_RX_PIN;
    int tx = swap ? RS485_RX_PIN : RS485_TX_PIN;
    for (unsigned long baud : bauds) {
      bool valid = false;
      size_t n = rs485Probe(rx, tx, baud, buf, sizeof(buf), valid);
      Serial.printf("[DEBUG]   RX=GPIO%d TX=GPIO%d baud=%lu -> %u byte(s)%s", rx, tx, baud, (unsigned)n, n ? ": " : "\n");
      for (size_t i = 0; i < n; i++) Serial.printf("%02X ", buf[i]);
      if (n) Serial.println(valid ? " [VALID]" : " [invalid/garbled]");
      if (n) heardAnything = true;
      if (valid) {
        Serial.printf("[DEBUG] RS485 WORKS with RX=GPIO%d TX=GPIO%d baud=%lu\n", rx, tx, baud);
        if (swap) Serial.printf("[DEBUG]   >> RX/TX are SWAPPED vs. the code: set RS485_RX_PIN=%d, RS485_TX_PIN=%d (or rewire).\n", rx, tx);
        if (baud != RS485_BAUD) Serial.printf("[DEBUG]   >> set RS485_BAUD to %lu\n", baud);
        return true;   // port left configured on the working settings
      }
    }
  }
  // Slave ID scan (IDs 1-10) on the configured pins/baud: the sensor's address
  // is user-changeable (register 0x07D0), so it may not be the default 1.
  Serial.printf("[DEBUG] RS485 slave-ID scan (IDs 2-10, RX=GPIO%d TX=GPIO%d %d baud)...\n",
                RS485_RX_PIN, RS485_TX_PIN, RS485_BAUD);
  RS485Serial.end();
  RS485Serial.begin(RS485_BAUD, SERIAL_8N1, RS485_RX_PIN, RS485_TX_PIN);
  delay(50);
  for (uint8_t id = 2; id <= 10; id++) {
    bool valid = false;
    size_t n = rs485Probe(RS485_RX_PIN, RS485_TX_PIN, RS485_BAUD, buf, sizeof(buf), valid, id, 300, false);
    // Anything beyond our own 8-byte echo is interesting.
    if (n > 8 || valid) {
      Serial.printf("[DEBUG]   ID %u -> %u byte(s): ", id, (unsigned)n);
      for (size_t i = 0; i < n; i++) Serial.printf("%02X ", buf[i]);
      Serial.println(valid ? " [VALID]" : " [invalid/garbled]");
      heardAnything = true;
    }
    if (valid) {
      wqSlaveId = id;
      Serial.printf("[DEBUG] RS485 WORKS: sensor answers at slave ID %u. Set WQ_SENSOR_SLAVE_ID to %u.\n", id, id);
      return true;
    }
  }
  Serial.println("[DEBUG]   no reply from IDs 2-10 (only echo/silence).");
  RS485Serial.end();
  RS485Serial.begin(RS485_BAUD, SERIAL_8N1, RS485_RX_PIN, RS485_TX_PIN);
  if (heardAnything) {
    Serial.println("[DEBUG] RS485: bytes heard but never valid -> wrong slave ID, A/B swapped, noise, or no common GND.");
  } else {
    Serial.println("[DEBUG] RS485: ZERO bytes on any setting -> sensor has no 10-30V power, A/B not connected,");
    Serial.println("[DEBUG]   module not powered (VCC/GND), or the module's RX/TX wires aren't reaching GPIO16/17.");
  }
  return false;
}

// ---------------------------------------------------------------------------
// SEN0711 offset + calibration commands (typed in the Serial Monitor).
// Register map per DFRobot wiki (wiki.dfrobot.com/sen0711/docs/23361):
//   0x0050 temp offset (x10, signed)   0x0051 pH offset (x100, signed)
//   0x0052 NH3 offset (x100, signed)   -- added by the sensor to every reading
//   0x1200/0x1201 NH3 2-point cal: [point 1|2][standard x100]  (FC 0x10)
//   0x1100/0x1101 pH  2-point cal: [point 1|2][standard x100]  (FC 0x10)
// ---------------------------------------------------------------------------
static void appendCrc(uint8_t* buf, size_t len) {
  uint16_t crc = modbusCrc16(buf, len);
  buf[len]     = crc & 0xFF;
  buf[len + 1] = crc >> 8;
}

// Sends a request and looks for a CRC-valid reply of respLen bytes with the
// same slave ID + function code (skipping any echo of our own request).
static bool wqTransact(const uint8_t* req, size_t reqLen, uint8_t* resp, size_t respLen,
                       unsigned long waitMs = 500) {
  uint8_t buf[48];
  size_t n = 0;
  while (RS485Serial.available()) RS485Serial.read();
  rs485Send(req, reqLen);
  unsigned long t0 = millis();
  while (millis() - t0 < waitMs) {
    while (RS485Serial.available() && n < sizeof(buf)) buf[n++] = RS485Serial.read();
    delay(2);
  }
  for (size_t i = 0; i + respLen <= n; i++) {
    if (buf[i] == req[0] && buf[i + 1] == req[1] &&
        modbusCrc16(&buf[i], respLen - 2) == (uint16_t)(buf[i + respLen - 2] | (buf[i + respLen - 1] << 8))) {
      memcpy(resp, &buf[i], respLen);
      return true;
    }
  }
  for (size_t i = 0; i + 5 <= n; i++) {
    if (buf[i] == req[0] && buf[i + 1] == (req[1] | 0x80) &&
        modbusCrc16(&buf[i], 3) == (uint16_t)(buf[i + 3] | (buf[i + 4] << 8))) {
      Serial.printf("[WQ] sensor rejected the command (Modbus exception 0x%02X)\n", buf[i + 2]);
      return false;
    }
  }
  // The SEN0711 also answers a refused calibration write with a non-standard
  // 8-byte frame carrying the error flag, e.g. 01 90 11 00 00 01 05 2B.
  for (size_t i = 0; i + 2 <= n; i++) {
    if (buf[i] == req[0] && buf[i + 1] == (req[1] | 0x80)) {
      Serial.print("[WQ] sensor REJECTED the command (error reply 0x");
      Serial.printf("%02X): ", buf[i + 1]);
      for (size_t j = i; j < n; j++) Serial.printf("%02X ", buf[j]);
      Serial.println();
      return false;
    }
  }
  Serial.printf("[WQ] no valid reply (%u byte(s) heard)", (unsigned)n);
  if (n) {
    Serial.print(": ");
    for (size_t i = 0; i < n; i++) Serial.printf("%02X ", buf[i]);
    Serial.print(" | sent: ");
    for (size_t i = 0; i < reqLen; i++) Serial.printf("%02X ", req[i]);
  }
  Serial.println();
  return false;
}

// Reads the stored NH3 offset (0x0052, x100 signed). Returns false on failure.
static bool wqReadNh3OffsetRaw(int16_t &raw) {
  uint8_t req[8] = { wqSlaveId, 0x03, 0x00, 0x52, 0x00, 0x01 };
  appendCrc(req, 6);
  uint8_t r[7];
  if (!wqTransact(req, sizeof(req), r, sizeof(r)) || r[2] != 0x02) return false;
  raw = (int16_t)((r[3] << 8) | r[4]);
  return true;
}

static void wqReadOffsets() {
  uint8_t req[8] = { wqSlaveId, 0x03, 0x00, 0x50, 0x00, 0x03 };
  appendCrc(req, 6);
  uint8_t r[11];
  if (!wqTransact(req, sizeof(req), r, sizeof(r)) || r[2] != 0x06) {
    Serial.println("[WQ] could not read offset registers 0x0050-0x0052");
    return;
  }
  int16_t tOff  = (int16_t)((r[3] << 8) | r[4]);
  int16_t phOff = (int16_t)((r[5] << 8) | r[6]);
  int16_t nhOff = (int16_t)((r[7] << 8) | r[8]);
  Serial.printf("[WQ] Stored offsets: Temp=%+.1f C  pH=%+.2f  NH3=%+.2f mg/L (raw %d / %d / %d)\n",
                tOff / 10.0f, phOff / 100.0f, nhOff / 100.0f, tOff, phOff, nhOff);
  if (nhOff != 0) Serial.println("[WQ]   NH3 offset is NOT zero -- the sensor adds it to every ammonia reading.");
}

// FC 0x06: write a single register.
static bool wqWriteReg(uint16_t reg, uint16_t value) {
  uint8_t req[8] = { wqSlaveId, 0x06, (uint8_t)(reg >> 8), (uint8_t)reg,
                     (uint8_t)(value >> 8), (uint8_t)value };
  appendCrc(req, 6);
  uint8_t r[8];
  wqFilterReset();   // readings before the change no longer apply
  return wqTransact(req, sizeof(req), r, sizeof(r));
}

// FC 0x10: two-point calibration write [point][standard x100] at baseReg.
static bool wqCalibrate(uint16_t baseReg, uint16_t point, float standard) {
  uint16_t val = (uint16_t)lroundf(standard * 100.0f);
  uint8_t req[13] = { wqSlaveId, 0x10, (uint8_t)(baseReg >> 8), (uint8_t)baseReg, 0x00, 0x02, 0x04,
                      (uint8_t)(point >> 8), (uint8_t)point, (uint8_t)(val >> 8), (uint8_t)val };
  appendCrc(req, 11);
  uint8_t r[8];
  wqFilterReset();   // readings before the change no longer apply
  return wqTransact(req, sizeof(req), r, sizeof(r), 1000);
}

static void wqPrintHelp() {
  Serial.println(F("[WQ] Serial Monitor commands (set line ending to Newline):"));
  Serial.println(F("[WQ]   WQ OFFSETS          read stored temp/pH/NH3 offsets"));
  Serial.println(F("[WQ]   WQ CLEARNH3         set the NH3 offset (0x0052) to 0"));
  Serial.println(F("[WQ]   WQ SETNH3 <mg/L>    set the NH3 offset by hand, e.g. WQ SETNH3 -1.50"));
  Serial.println(F("[WQ]   WQ SETPH <pH>       set the pH offset by hand, e.g. WQ SETPH 0.66 (0 clears)"));
  Serial.println(F("[WQ]   WQ ZERONH3          probe in DISTILLED water -> make current NH3 read 0.00"));
  Serial.println(F("[WQ]   WQ BLANK [minutes]  same, but averages every reading for 15 min (default)"));
  Serial.println(F("[WQ]   WQ CANCEL           stop a running WQ BLANK without changing anything"));
  Serial.println(F("[WQ]   WQ CALNH3 <1|2> <mg/L>  NH3 cal point (1=low std, 2=high std)"));
  Serial.println(F("[WQ]   WQ CALPH  <1|2> <pH>    pH cal point (e.g. 1 4.01, 2 7.00)"));
  Serial.println(F("[WQ]   Soak the probe in the standard until stable BEFORE each CAL command."));
}

// Folds a blank (ammonia-free water) average into the sensor's own NH3 offset
// so that water reads 0.00. The reading already includes the old offset, so
// newOffset = oldOffset - blankAverage.
static bool wqApplyBlank(float avg) {
  int16_t oldRaw;
  if (!wqReadNh3OffsetRaw(oldRaw)) {
    Serial.println("[WQ] ZERO FAILED: can't read current offset");
    return false;
  }
  long newRaw = (long)oldRaw - lroundf(avg * 100.0f);
  newRaw = constrain(newRaw, -32768L, 32767L);
  bool ok = wqWriteReg(0x0052, (uint16_t)(int16_t)newRaw);
  Serial.printf("[WQ] Blank avg=%.2f mg/L  offset %+.2f -> %+.2f mg/L: %s\n",
                avg, oldRaw / 100.0f, newRaw / 100.0f, ok ? "OK" : "FAILED");
  wqReadOffsets();
  return ok;
}

// ---------------------------------------------------------------------------
// WQ BLANK: long blank average. Runs in the background -- every normal
// reading cycle (TEST_LOOP_INTERVAL_MS) adds its NH3 value -- and when the
// time is up the mean becomes the sensor's zero via wqApplyBlank().
// Drift = last-minute mean minus first-minute mean; a large drift means the
// probe was still settling, so the mean is biased and the run should be
// repeated after a longer soak.
// ---------------------------------------------------------------------------
#define WQ_BLANK_DEFAULT_MIN   15
#define WQ_BLANK_MAX_DRIFT     0.20f   // mg/L between first and last minute

static bool          blankActive = false;
static unsigned long blankStartMs = 0, blankDurMs = 0, blankMinuteStartMs = 0;
static double        blankSum = 0, blankSumSq = 0, blankMinuteSum = 0;
static uint32_t      blankN = 0, blankMinuteN = 0, blankMinuteIdx = 0;
static float         blankMin = 0, blankMax = 0, blankFirstMinAvg = NAN, blankLastMinAvg = NAN;

static void wqBlankStart(int minutes) {
  blankActive = true;
  blankStartMs = blankMinuteStartMs = millis();
  blankDurMs = (unsigned long)minutes * 60000UL;
  blankSum = blankSumSq = blankMinuteSum = 0;
  blankN = blankMinuteN = blankMinuteIdx = 0;
  blankFirstMinAvg = blankLastMinAvg = NAN;
  Serial.printf("[WQ] BLANK started: averaging NH3 for %d min. Keep the probe tip in DISTILLED water,\n", minutes);
  Serial.println("[WQ]   still and bubble-free. Type WQ CANCEL to abort (nothing is changed).");
}

static void wqBlankFinish() {
  blankActive = false;
  uint32_t expected = blankDurMs / TEST_LOOP_INTERVAL_MS;
  if (blankN < expected / 2) {
    Serial.printf("[WQ] BLANK FAILED: only %lu good readings of ~%lu expected - check the sensor.\n",
                  (unsigned long)blankN, (unsigned long)expected);
    return;
  }
  float avg = blankSum / blankN;
  float var = blankSumSq / blankN - (double)avg * avg;
  float sd = var > 0 ? sqrtf(var) : 0;
  float drift = blankLastMinAvg - blankFirstMinAvg;
  Serial.println("[WQ] ===== BLANK RESULT =====");
  Serial.printf("[WQ]   samples=%lu  mean=%.3f  sd=%.3f  min=%.2f  max=%.2f mg/L\n",
                (unsigned long)blankN, avg, sd, blankMin, blankMax);
  Serial.printf("[WQ]   first-minute mean=%.3f  last-minute mean=%.3f  drift=%+.3f mg/L\n",
                blankFirstMinAvg, blankLastMinAvg, drift);
  if (!isnan(drift) && fabsf(drift) > WQ_BLANK_MAX_DRIFT) {
    Serial.printf("[WQ]   WARNING: drift > %.2f mg/L - probe was still settling; mean may be biased.\n",
                  WQ_BLANK_MAX_DRIFT);
    Serial.println("[WQ]   Consider soaking longer and running WQ BLANK again.");
  }
  wqApplyBlank(avg);
}

// Called once per reading cycle with the fresh NH3 value (NaN = failed read).
static void wqBlankSample(float nh3) {
  if (!blankActive) return;
  if (!isnan(nh3)) {
    if (blankN == 0 || nh3 < blankMin) blankMin = nh3;
    if (blankN == 0 || nh3 > blankMax) blankMax = nh3;
    blankSum += nh3; blankSumSq += (double)nh3 * nh3; blankN++;
    blankMinuteSum += nh3; blankMinuteN++;
  }
  unsigned long now = millis();
  bool done = now - blankStartMs >= blankDurMs;
  if (now - blankMinuteStartMs >= 60000UL || done) {
    if (blankMinuteN > 0) {
      float m = blankMinuteSum / blankMinuteN;
      if (blankMinuteIdx == 0) blankFirstMinAvg = m;
      blankLastMinAvg = m;
      blankMinuteIdx++;
      Serial.printf("[WQ] BLANK minute %lu/%lu: mean=%.3f mg/L (%lu readings)\n",
                    (unsigned long)blankMinuteIdx, blankDurMs / 60000UL, m, (unsigned long)blankMinuteN);
    }
    blankMinuteSum = 0; blankMinuteN = 0; blankMinuteStartMs = now;
  }
  if (done) wqBlankFinish();
}

// Commands arrive inside pollSerialCommands(), which also runs in the middle
// of sensor reads -- so only queue them here and run them from loop().
static String wqPendingCmd = "";

static void runPendingWqCommand() {
  if (wqPendingCmd.length() == 0) return;
  String cmd = wqPendingCmd;
  wqPendingCmd = "";

  if (cmd == "WQ OFFSETS") {
    wqReadOffsets();
  } else if (cmd == "WQ CLEARNH3") {
    Serial.println(wqWriteReg(0x0052, 0) ? "[WQ] NH3 offset cleared." : "[WQ] clearing NH3 offset FAILED");
    wqReadOffsets();
  } else if (cmd.startsWith("WQ SETNH3 ")) {
    // Manual offset: the sensor ADDS this to every ammonia reading
    // (negative lowers the readings).
    String arg = cmd.substring(10);
    arg.trim();
    float mgL = arg.toFloat();
    if (arg.length() == 0 || (mgL == 0.0f && arg[0] != '0' && arg[0] != '-' && arg[0] != '+' && arg[0] != '.') ||
        mgL < -100.0f || mgL > 100.0f) {
      Serial.println("[WQ] usage: WQ SETNH3 <mg/L>   (-100 to 100, e.g. WQ SETNH3 -1.50)");
      return;
    }
    long raw = lroundf(mgL * 100.0f);
    bool ok = wqWriteReg(0x0052, (uint16_t)(int16_t)raw);
    Serial.printf("[WQ] Set NH3 offset to %+.2f mg/L: %s\n", raw / 100.0f, ok ? "OK" : "FAILED");
    wqReadOffsets();
  } else if (cmd.startsWith("WQ SETPH ")) {
    // Manual pH offset (register 0x0051, x100): the sensor ADDS it to every
    // pH reading. E.g. a 9.18 buffer reading 8.52 -> WQ SETPH 0.66.
    String arg = cmd.substring(9);
    arg.trim();
    float ph = arg.toFloat();
    if (arg.length() == 0 || (ph == 0.0f && arg[0] != '0' && arg[0] != '-' && arg[0] != '+' && arg[0] != '.') ||
        ph < -5.0f || ph > 5.0f) {
      Serial.println("[WQ] usage: WQ SETPH <pH>   (-5 to 5, e.g. WQ SETPH 0.66; WQ SETPH 0 clears it)");
      return;
    }
    long raw = lroundf(ph * 100.0f);
    bool ok = wqWriteReg(0x0051, (uint16_t)(int16_t)raw);
    Serial.printf("[WQ] Set pH offset to %+.2f: %s\n", raw / 100.0f, ok ? "OK" : "FAILED");
    wqReadOffsets();
  } else if (cmd == "WQ ZERONH3") {
    // Quick blank: average 10 readings (~5 s) in ammonia-free water.
    float sum = 0; int n = 0;
    Serial.println("[WQ] Zeroing: averaging 10 readings (~5 s), keep the probe still...");
    for (int i = 0; i < 10; i++) {
      if (readWaterQualitySensor()) { sum += readings.ammonia_mgL; n++; }
      delayPollingDht(300);
    }
    if (n < 5) { Serial.println("[WQ] ZERO FAILED: too few good readings"); return; }
    wqApplyBlank(sum / n);
  } else if (cmd == "WQ BLANK" || cmd.startsWith("WQ BLANK ")) {
    if (blankActive) { Serial.println("[WQ] BLANK already running (WQ CANCEL to stop it)."); return; }
    int minutes = cmd.length() > 8 ? cmd.substring(9).toInt() : WQ_BLANK_DEFAULT_MIN;
    if (minutes < 1 || minutes > 120) { Serial.println("[WQ] usage: WQ BLANK [1-120 minutes]"); return; }
    wqBlankStart(minutes);
  } else if (cmd == "WQ CANCEL") {
    Serial.println(blankActive ? "[WQ] BLANK cancelled - offset NOT changed." : "[WQ] nothing to cancel.");
    blankActive = false;
  } else if (cmd.startsWith("WQ CALNH3 ") || cmd.startsWith("WQ CALPH ")) {
    bool isNh3 = cmd.startsWith("WQ CALNH3 ");
    String args = cmd.substring(isNh3 ? 10 : 9);
    args.trim();
    int sp = args.indexOf(' ');
    int point = sp > 0 ? args.substring(0, sp).toInt() : 0;
    float standard = sp > 0 ? args.substring(sp + 1).toFloat() : -1.0f;
    float maxStd = isNh3 ? 100.0f : 14.0f;
    if ((point != 1 && point != 2) || standard <= 0.0f || standard > maxStd) {
      Serial.println("[WQ] usage: WQ CALNH3 <1|2> <mg/L>   or   WQ CALPH <1|2> <pH>");
      return;
    }
    bool ok = wqCalibrate(isNh3 ? 0x1200 : 0x1100, point, standard);
    Serial.printf("[WQ] %s calibration point %d at %.2f %s: %s\n", isNh3 ? "NH3" : "pH",
                  point, standard, isNh3 ? "mg/L" : "pH", ok ? "OK" : "FAILED");
  } else {
    wqPrintHelp();
  }
}
#endif

// Called by isdasafe_pairing.h for any serial line that isn't a pairing command.
void handleAppSerialCommand(const String &line) {
#if ENABLE_RS485
  String cmd = line;
  cmd.trim();
  cmd.toUpperCase();
  if (cmd == "FAN" || cmd.startsWith("FAN ")) {
    handleFanCommand(cmd);
  } else if (cmd.startsWith("WQ")) {
    wqPendingCmd = cmd;
    Serial.println("[WQ] received: " + cmd + " (runs after the current reading)");
  } else {
    Serial.println("[WQ] unknown command: " + cmd + " (type WQ for the list; FAN ON|OFF|AUTO|<0-100> for the fan)");
  }
#endif
}

// ---------------------------------------------------------------------------
// Dissolved Oxygen (ADS1115 + temperature-compensated lookup table)
// ---------------------------------------------------------------------------
bool adsOk = false;   // set in setup() from ads.begin(); retried while false

// Below this the DO probe is treated as disconnected. With the pull-down on
// A0 an unplugged input sits at ~0-20 mV of noise; a working probe gives
// hundreds of mV (see DO_CAL1_V / DO_CAL2_V), and 50 mV is well under
// 1 mg/L, so real readings above that are never blanked.
#define DO_MIN_VALID_MV  50.0f

// Above this fraction of saturation a DO value is treated as impossible.
#define DO_MAX_SATURATION  2.0f

// Each DO reading is the average of the middle DO_SAMPLES_KEPT of
// DO_SAMPLES conversions: dropping the highest and lowest means one bad I2C
// read can't skew it (the library doesn't report a failed read, it returns
// ~0-48 mV), and averaging the rest smooths the probe's noise.
#define DO_SAMPLES             15
#define DO_SAMPLES_KEPT        9
#define DO_SAMPLE_GAP_MS       10
#define DO_CONVERSION_TIMEOUT_MS 50   // one conversion takes ~8 ms at 128 SPS

// ADS1115 range: +/-4.096 V (0.125 mV per count). The probe board outputs
// 0-3 V, so this covers it without clipping and is finer than the library's
// default +/-6.144 V. computeVolts() follows the gain, so the DO_CAL mV
// values stay valid whatever gain DO_Calibration used.
#define DO_ADS_GAIN            GAIN_ONE

// DO depends strongly on temperature (~2% per degree C), so it's only
// computed with a real water temperature. If the SEN0711's temperature is
// missing for a reading, the last one is used for up to this long (pond
// water changes temperature slowly); after that there's no DO reading.
#define DO_WATER_TEMP_MAX_AGE_MS (5UL * 60UL * 1000UL)

// Whether the ADS1115 acknowledges its address on Wire1.
static bool adsAnswers() {
  Wire1.beginTransmission(ADS1115_ADDR);
  return Wire1.endTransmission() == 0;
}

// One conversion on A0 in mV, or NAN if the ADS1115 didn't answer before
// and after it or didn't finish in time. Used instead of
// ads.readADC_SingleEnded(), which waits forever if the chip stops answering.
static float adsReadA0mV() {
  if (!adsAnswers()) return NAN;
  ads.startADCReading(MUX_BY_CHANNEL[0], /*continuous=*/false);
  unsigned long start = millis();
  while (!ads.conversionComplete()) {
    if (millis() - start > DO_CONVERSION_TIMEOUT_MS) return NAN;
    delay(1);
  }
  int16_t counts = ads.getLastConversionResults();
  if (!adsAnswers()) return NAN;
  return ads.computeVolts(counts) * 1000.0f;
}

// Starts the ADS1115 on Wire1 at DO_ADS_GAIN; false if it doesn't answer.
bool adsBegin() {
  ads.setGain(DO_ADS_GAIN);
  return ads.begin(ADS1115_ADDR, &Wire1);
}

// Saturated DO (mg/L) at waterTempC, interpolated between the whole degrees
// of DO_TABLE (0-40 C).
static float doSaturation(float waterTempC) {
  float t = constrain(waterTempC, 0.0f, 40.0f);
  int i = (int)t;
  if (i >= 40) return DO_TABLE[40];
  return DO_TABLE[i] + (DO_TABLE[i + 1] - DO_TABLE[i]) * (t - i);
}

float readDissolvedOxygen(float waterTempC) {
  if (isnan(waterTempC)) {
    Serial.println("[DEBUG] DO: no water temperature from the SEN0711 - no DO reading (it can't be temperature-compensated).");
    return NAN;
  }
  if (!adsOk) {
    // Not found at boot (or stopped answering): try again every reading.
    adsOk = adsBegin();
    if (!adsOk) {
      Serial.println("[DEBUG] DO: ADS1115 not answering on I2C - no DO reading (check SDA/SCL/power).");
      return NAN;
    }
    Serial.println("[DEBUG] DO: ADS1115 found again.");
  }

  float samples[DO_SAMPLES];
  int n = 0;
  for (int i = 0; i < DO_SAMPLES; i++) {
    float mV = adsReadA0mV();
    if (!isnan(mV)) {   // insertion sort as they come in
      int j = n++;
      while (j > 0 && samples[j - 1] > mV) { samples[j] = samples[j - 1]; j--; }
      samples[j] = mV;
    }
    if (i < DO_SAMPLES - 1) delay(DO_SAMPLE_GAP_MS);
  }
  if (n == 0) {
    Serial.println("[DEBUG] DO: ADS1115 stopped answering on I2C - no DO reading; will re-init next time.");
    adsOk = false;
    return NAN;
  }
  // Average of the middle DO_SAMPLES_KEPT (or as many as there are).
  int keep = n < DO_SAMPLES_KEPT ? n : DO_SAMPLES_KEPT;
  int first = (n - keep) / 2;
  float sum = 0;
  for (int i = first; i < first + keep; i++) sum += samples[i];
  float voltage_mV = sum / keep;
  Serial.printf("[DEBUG] DO probe voltage: %.1f mV (mean of middle %d of %d/%d, range %.1f-%.1f)\n",
                voltage_mV, keep, n, DO_SAMPLES, samples[0], samples[n - 1]);
  if (voltage_mV < DO_MIN_VALID_MV) {
    Serial.printf("[DEBUG] DO: %.1f mV < %.0f mV -> probe disconnected (or no oxygen).\n",
                  voltage_mV, DO_MIN_VALID_MV);
    return NAN;
  }

  // Probe voltage in saturated water at this temperature (from the
  // calibration) and the DO that saturation means. Both use the exact
  // temperature, not whole degrees: 1 C off is ~2% off.
  float t = constrain(waterTempC, 0.0f, 40.0f);
#if DO_TWO_POINT_CALIBRATION == 1
  float vSaturation = (t - DO_CAL2_T) * (DO_CAL1_V - DO_CAL2_V)
                       / (float)(DO_CAL1_T - DO_CAL2_T) + DO_CAL2_V;
#else
  float vSaturation = DO_CAL1_V + 35.0f * (t - DO_CAL1_T);
#endif
  float saturation = doSaturation(t);

  float doValue = voltage_mV * saturation / vSaturation;

  // Water can't hold more than about twice its saturation level (algae on a
  // sunny afternoon can push a pond to ~150-200%). Anything above that is a
  // bad calibration or a bubble on the membrane, not a reading: send it as
  // missing rather than a value that looks healthy.
  float maxPlausible = saturation * DO_MAX_SATURATION;
  if (doValue > maxPlausible) {
    Serial.printf("[DEBUG] DO: %.2f mg/L is over %.0f%% saturation (%.2f mg/L at %.1f C) -> not plausible, "
                  "no DO reading. Check the calibration (DO_CAL...) and for bubbles on the membrane.\n",
                  doValue, DO_MAX_SATURATION * 100.0f, maxPlausible, t);
    return NAN;
  }
  Serial.printf("[DEBUG] DO: %.2f mg/L = %.0f%% saturation at %.1f C (saturated: %.2f mg/L at %.0f mV)\n",
                doValue, doValue / saturation * 100.0f, t, saturation, vSaturation);
  return doValue;
}

// ---------------------------------------------------------------------------
// DHT22 continuous polling. The DHT22 can only be read about every 2 s, so
// this is rate-limited by millis() and is called from delayPollingDht() while
// the sketch would otherwise be idle (LCD page display, loop interval).
// Returns true when a fresh, valid reading was taken.
// ---------------------------------------------------------------------------
// Set to 1 to re-enable continuous DHT22 reads between reading cycles.
// With 0, the DHT22 is read only once per cycle (in pollSensors()).
#define ENABLE_DHT_CONTINUOUS  0
const unsigned long DHT_INTERVAL_MS = 2000;
unsigned long lastDhtMs = 0;
bool dhtEverRead = false;

bool pollDht(bool force = false) {
  if (!force && dhtEverRead && millis() - lastDhtMs < DHT_INTERVAL_MS) return false;
  lastDhtMs = millis();
  dhtEverRead = true;

  float h = dht.readHumidity();
  float t = dht.readTemperature();
  if (!isnan(h)) readings.humidity_pct = h;
  if (!isnan(t)) readings.ambientTemp_C = t;
  return !isnan(h) && !isnan(t);
}

// ---------------------------------------------------------------------------
// Sensor polling
// ---------------------------------------------------------------------------
#if ENABLE_RS485
// RS485 auto-recovery: after WQ_REINIT_AFTER failed reads in a row the UART
// and DE/RE line are restarted; after WQ_DIAGNOSE_AFTER the full boot probe
// (pins/baud/slave-ID scan) runs again. Fixes "only works after re-plugging".
#define WQ_REINIT_AFTER    5
#define WQ_DIAGNOSE_AFTER  20
static uint16_t wqFailStreak = 0;

static void wqRecoverIfNeeded(bool readOk) {
  if (readOk) {
    if (wqFailStreak >= WQ_REINIT_AFTER) Serial.println("[WQ] RS485 recovered.");
    wqFailStreak = 0;
    return;
  }
  wqFailStreak++;
  if (wqFailStreak % WQ_DIAGNOSE_AFTER == 0) {
    Serial.printf("[WQ] %u failed reads in a row - re-running RS485 auto-diagnose...\n", wqFailStreak);
    rs485AutoDiagnose();
  } else if (wqFailStreak % WQ_REINIT_AFTER == 0) {
    Serial.printf("[WQ] %u failed reads in a row - restarting RS485 port.\n", wqFailStreak);
    RS485Serial.end();
#if RS485_USE_DE_RE
    digitalWrite(RS485_DE_RE_PIN, LOW);
#endif
    delay(50);
    RS485Serial.begin(RS485_BAUD, SERIAL_8N1, RS485_RX_PIN, RS485_TX_PIN);
  }
}
#endif

// ---------------------------------------------------------------------------
// Water-quality analysis, run after every reading:
//  * Free (un-ionized) NH3 from total ammonia-N, pH and temperature
//    (Emerson et al. 1975):  pKa = 0.09018 + 2729.92 / T(K)
//    fraction = 1 / (1 + 10^(pKa - pH));  NH3 = NH3-N * fraction * 17/14
//  * Alarm with hysteresis: ON after ALARM_CONFIRM readings above
//    FREE_NH3_ALARM_ON, OFF after ALARM_CONFIRM readings below
//    FREE_NH3_ALARM_OFF -- no flickering when hovering near the limit.
//  * Warm-up: for WQ_WARMUP_MS after power-on the probe is still settling,
//    so alarms are suppressed and nothing is uploaded.
//  * Frozen-value check: if the sensor's raw NH3 AND pH haven't changed at
//    all for WQ_STUCK_MS, warn (probe out of water or sensor hung).
// ---------------------------------------------------------------------------
#define FREE_NH3_ALARM_ON   0.05f    // mg/L un-ionized NH3
#define FREE_NH3_ALARM_OFF  0.04f
#define ALARM_CONFIRM       3        // consecutive readings to change state
// TESTING: warm-up off (0). For deployment use (5UL * 60UL * 1000UL) = 5 min.
#define WQ_WARMUP_MS        0UL
#define WQ_STUCK_MS         (30UL * 60UL * 1000UL)

bool freeNh3Alarm = false;
bool wqStuck = false;
static uint8_t alarmStreak = 0;
static float stuckNh3 = NAN, stuckPh = NAN;
static unsigned long stuckSinceMs = 0;

float computeFreeNh3(float nh3N, float ph, float tempC) {
  if (isnan(nh3N) || isnan(ph) || isnan(tempC)) return NAN;
  float pKa = 0.09018f + 2729.92f / (tempC + 273.15f);
  float fraction = 1.0f / (1.0f + powf(10.0f, pKa - ph));
  return nh3N * fraction * (17.031f / 14.007f);
}

// Deep sleep restarts millis() on every wake, so warm-up would never end and
// nothing would upload -- it only applies in continuous (no-sleep) mode.
bool wqWarmingUp() { return !ENABLE_DEEP_SLEEP && millis() < WQ_WARMUP_MS; }

void analyzeWaterQuality() {
  readings.freeNh3_mgL = computeFreeNh3(readings.ammonia_mgL, readings.ph, readings.waterTemp_C);

  // Alarm with hysteresis (suppressed during warm-up / missing data).
  if (wqWarmingUp() || isnan(readings.freeNh3_mgL)) {
    alarmStreak = 0;
  } else {
    bool wantChange = freeNh3Alarm ? (readings.freeNh3_mgL < FREE_NH3_ALARM_OFF)
                                   : (readings.freeNh3_mgL > FREE_NH3_ALARM_ON);
    alarmStreak = wantChange ? alarmStreak + 1 : 0;
    if (alarmStreak >= ALARM_CONFIRM) {
      freeNh3Alarm = !freeNh3Alarm;
      alarmStreak = 0;
      Serial.printf(freeNh3Alarm ? "[ALARM] Free NH3 HIGH: %.3f mg/L (> %.2f)\n"
                                 : "[ALARM] Free NH3 back to normal: %.3f mg/L (< %.2f)\n",
                    readings.freeNh3_mgL, freeNh3Alarm ? FREE_NH3_ALARM_ON : FREE_NH3_ALARM_OFF);
    }
  }

#if ENABLE_RS485
  // Frozen-value check on the unfiltered sensor values.
  if (!isnan(wqLastRawNh3) && wqLastRawNh3 == stuckNh3 && wqLastRawPh == stuckPh) {
    if (!wqStuck && millis() - stuckSinceMs >= WQ_STUCK_MS) {
      wqStuck = true;
      Serial.println("[WARN] SEN0711 NH3 and pH unchanged for 30 min - probe out of water or sensor hung?");
    }
  } else {
    if (wqStuck) Serial.println("[WARN] SEN0711 values changing again.");
    wqStuck = false;
    stuckNh3 = wqLastRawNh3;
    stuckPh  = wqLastRawPh;
    stuckSinceMs = millis();
  }
#endif

  if (wqWarmingUp()) {
    unsigned long left = (WQ_WARMUP_MS - millis()) / 1000UL;
    Serial.printf("[WARMUP] probe settling - %lu:%02lu left (alarms/uploads paused)\n", left / 60, left % 60);
  }
}

void pollSensors() {
#if ENABLE_RS485
  bool wqOk = readWaterQualitySensor();
  if (!wqOk) {
    Serial.println("RS485 water quality sensor read failed/timed out.");
  }
  wqRecoverIfNeeded(wqOk);
#else
  Serial.println("RS485 disabled (ENABLE_RS485=0) - skipping pH/Ammonia/WaterTemp.");
#endif

  // DO is compensated with the real water temperature (or the last one, if
  // this read missed it and it's recent), never a guessed value.
  static float lastWaterTemp = NAN;
  static unsigned long lastWaterTempMs = 0;
  float waterTempForDo = readings.waterTemp_C;
  if (!isnan(waterTempForDo)) {
    lastWaterTemp = waterTempForDo;
    lastWaterTempMs = millis();
  } else if (!isnan(lastWaterTemp) && millis() - lastWaterTempMs <= DO_WATER_TEMP_MAX_AGE_MS) {
    waterTempForDo = lastWaterTemp;
    Serial.printf("[DEBUG] DO: using the last water temperature (%.1f C, %lu s old).\n",
                  lastWaterTemp, (millis() - lastWaterTempMs) / 1000UL);
  }
  readings.dissolvedOxygen_mgL = readDissolvedOxygen(waterTempForDo);

  pollDht(true);
}

// ---------------------------------------------------------------------------
// Cooling fan control (DRV8833, one channel, forward-only speed control)
// ---------------------------------------------------------------------------
void setFanDuty(uint8_t duty) {
  if (duty > 0 && fanDuty == 0) {
    ledcWrite(MOTOR_AIN1_PIN, 255);   // kick-start
    ledcWrite(MOTOR_AIN2_PIN, 0);
    delay(FAN_KICK_MS);
  }
  ledcWrite(MOTOR_AIN1_PIN, duty);    // IN1 = PWM, IN2 = low -> forward
  ledcWrite(MOTOR_AIN2_PIN, 0);
  if (duty != fanDuty) {
    Serial.printf("[FAN] %s (duty %u/255 = %u%%)\n", duty ? "ON" : "OFF",
                  duty, (unsigned)(duty * 100U / 255U));
  }
  fanDuty = duty;
}

void controlCooling(float waterTempC) {
  uint8_t duty = 0;

  if (fanOverride >= 0) {
    setFanDuty((uint8_t)fanOverride);   // manual FAN command
    return;
  }

#if FAN_ALWAYS_ON
  duty = FAN_ALWAYS_ON_DUTY;  // ignore temperature entirely
#else
  if (!isnan(waterTempC) && waterTempC >= COOLING_ON_TEMP) {
    float span = COOLING_MAX_TEMP - COOLING_ON_TEMP;
    float frac = (waterTempC - COOLING_ON_TEMP) / span;
    frac = constrain(frac, 0.0f, 1.0f);
    duty = COOLING_MIN_DUTY + (uint8_t)(frac * (255 - COOLING_MIN_DUTY));
  }
#endif

  setFanDuty(duty);
}

// FAN ON | FAN OFF | FAN AUTO | FAN <0-100 percent>
void handleFanCommand(String cmd) {
  String arg = cmd.substring(3);
  arg.trim();
  if (arg == "ON") {
    fanOverride = 255;
  } else if (arg == "OFF") {
    fanOverride = 0;
  } else if (arg == "AUTO" || arg.length() == 0) {
    fanOverride = -1;
  } else {
    int pct = constrain(arg.toInt(), 0, 100);
    fanOverride = pct * 255 / 100;
  }
  if (fanOverride < 0) {
#if FAN_ALWAYS_ON
    Serial.println("[FAN] AUTO: FAN_ALWAYS_ON=1, fan stays on");
#else
    Serial.printf("[FAN] AUTO: runs when water >= %.0f C (now %.1f C)\n",
                  COOLING_ON_TEMP, readings.waterTemp_C);
#endif
    controlCooling(readings.waterTemp_C);
  } else {
    Serial.println("[FAN] manual override (type FAN AUTO to return to temperature control)");
    setFanDuty((uint8_t)fanOverride);
  }
}

// LCD screens: one reading per page with its status, an alert page, a
// WiFi/sync status page and the boot screen -- see isdasafe_lcd.h.
#include "isdasafe_lcd.h"

// delay() replacement used for every wait: keeps listening for the website's
// PAIR/UNPAIR commands over USB serial (and, if enabled, keeps the DHT22 and
// LCD page current) instead of blocking for the whole wait.
void delayPollingDht(unsigned long ms) {
  unsigned long start = millis();
  while (millis() - start < ms) {
    pollSerialCommands();
    wifiLoop();
#if ENABLE_DHT_CONTINUOUS
    if (pollDht()) updateLcd();
#endif
    delay(20);
  }
}

#if ENABLE_DUTY_CYCLE
// ---------------------------------------------------------------------------
// Duty cycle warm-up: stay awake until DUTY_WARMUP_MIN after power-on (or
// wake) so the probes stabilise before the one reading that counts. The
// sensors are still read every DUTY_WARMUP_POLL_MS -- that keeps the SEN0711
// median filter full of settled values and shows live values on the LCD --
// but nothing is uploaded. WiFi, the setup hotspot and serial (WQ ...)
// commands keep working meanwhile.
// ---------------------------------------------------------------------------
#define DUTY_WARMUP_POLL_MS 10000UL

void dutyWarmUp() {
  // Counted from power-on: WiFi connecting at boot is part of the wait.
  lcdWarmupUntilMs = (unsigned long)DUTY_WARMUP_MIN * 60000UL;
  if (millis() >= lcdWarmupUntilMs) {
    lcdWarmupUntilMs = 0;
    return;
  }
  Serial.printf("[DUTY] warming up the sensors for %d min before the reading (nothing uploaded yet)...\n",
                DUTY_WARMUP_MIN);
  unsigned long lastPoll = 0, lastPage = millis(), lastLog = millis();
  updateLcd();
  while (millis() < lcdWarmupUntilMs) {
#if ENABLE_RS485
    runPendingWqCommand();   // WQ ... commands typed in the Serial Monitor
#endif
    if (lastPoll == 0 || millis() - lastPoll >= DUTY_WARMUP_POLL_MS) {
      lastPoll = millis();
      pollSensors();
      controlCooling(readings.waterTemp_C);
    }
    if (millis() - lastPage >= LCD_PAGE_DISPLAY_MS) {
      lastPage = millis();
      // Countdown, reading page, countdown, next reading page, ...
      lcdShowCountdown = !lcdShowCountdown;
      if (!lcdShowCountdown) lcdPage = (lcdPage + 1) % lcdPageCount();
      updateLcd();
    }
    if (millis() - lastLog >= 60000UL) {
      lastLog = millis();
      unsigned long left = (lcdWarmupUntilMs - millis()) / 1000UL;
      Serial.printf("[DUTY] warm-up: %lu:%02lu left\n", left / 60, left % 60);
    }
    delayPollingDht(50);
  }
  lcdWarmupUntilMs = 0;
  Serial.println("[DUTY] warm-up done - taking the reading.");
}
#endif

// ---------------------------------------------------------------------------
// Power down peripherals and drop into deep sleep until the next reading.
// ---------------------------------------------------------------------------
void goToSleep() {
  ledcWrite(MOTOR_AIN1_PIN, 0);
  ledcWrite(MOTOR_AIN2_PIN, 0);
#if MOTOR_STBY_CONNECTED
  digitalWrite(MOTOR_STBY_PIN, LOW); // DRV8833 nSLEEP low -> driver stage off
#endif
  // Sleep unconnected: driver stage stays enabled through deep sleep (relies
  // on onboard pull-up), but PWM outputs above are already zeroed, so the
  // motor itself stays off regardless.

  lcd.noBacklight();
  lcd.noDisplay();

  WiFi.disconnect(true);
  WiFi.mode(WIFI_OFF);

  Serial.printf("Sleeping for %d minutes...\n", READING_INTERVAL_MIN);
  Serial.flush();

  esp_sleep_enable_timer_wakeup(SLEEP_DURATION_US);
  esp_deep_sleep_start();
}

// ---------------------------------------------------------------------------
// Setup / Loop
// ---------------------------------------------------------------------------
void setup() {
  Serial.begin(115200);
  pairingBegin();   // prints ID=<chip id>  PAIRED=yes|no
  queueBegin();     // readings saved while offline, still to upload
  printPinConfig();

  // LCD first, so the boot shows progress. Its bus (global Wire) must
  // begin() before lcd.init(), since the LCD library also calls
  // Wire.begin() internally with no args; whichever begin() call runs first
  // wins the pin assignment, later calls just warn "already started" and
  // keep the existing pins.
  Wire.begin(LCD_SDA_PIN, LCD_SCL_PIN);
  lcd.init();
  lcd.backlight();
  lcdCreateChars();
  lcdBootStep("Starting...");

  // The network comes from flash (see isdasafe_wifi.h). With none saved, or
  // if the saved one can't be reached, the setup hotspot opens so it can be
  // chosen from a phone.
  if (wifiBegin()) {
    Serial.print("Connecting to WiFi \"" + wifiSavedSsid() + "\"");
    lcdBootStep("Connecting WiFi");
    unsigned long wifiStart = millis();
    // Stops early once the router has clearly said no (not in range, wrong
    // password), so the hotspot opens in seconds rather than after 20 s.
    while (WiFi.status() != WL_CONNECTED && !wifiSavedFailed() && millis() - wifiStart < 20000) {
      delayPollingDht(500);
      Serial.print(".");
    }
    Serial.println();
    if (WiFi.status() == WL_CONNECTED) {
      Serial.print("WiFi connected, IP: ");
      Serial.println(WiFi.localIP());
      lcdBootStep(String((char)LCD_WIFI) + WiFi.SSID());
    } else {
      Serial.println(wifiSavedFailed()
                       ? "Saved WiFi not reachable - opening the setup hotspot (still retrying in background)."
                       : "WiFi connect timed out - opening the setup hotspot (still retrying in background).");
      wifiStartSetup();
    }
  } else {
    Serial.println("No WiFi network saved - opening the setup hotspot.");
    wifiStartSetup();
  }


  // ADS1115 bus (separate, Wire1) -- kept independent from the LCD's bus.
  Wire1.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  adsOk = adsBegin();
  if (!adsOk) {
    Serial.println("ADS1115 not found - check DO sensor wiring/address.");
  }

  dht.begin();

#if ENABLE_RS485
#if RS485_USE_DE_RE
  pinMode(RS485_DE_RE_PIN, OUTPUT);
  digitalWrite(RS485_DE_RE_PIN, LOW);   // idle in receive mode
#endif
  RS485Serial.begin(RS485_BAUD, SERIAL_8N1, RS485_RX_PIN, RS485_TX_PIN);
  rs485AutoDiagnose();  // finds working RX/TX/baud (or prints why nothing answers)
  wqReadOffsets();      // a non-zero stored NH3 offset shifts every ammonia reading
  wqPrintHelp();
  // MAX3485: DE/RE direction is handled in rs485Send().
#endif

#if MOTOR_STBY_CONNECTED
  pinMode(MOTOR_STBY_PIN, OUTPUT);
  digitalWrite(MOTOR_STBY_PIN, HIGH); // enable DRV8833 driver stage
#endif
  // Sleep pin left unconnected (MOTOR_STBY_CONNECTED=0) -- relies on the
  // DRV8833 board's onboard pull-up jumper to stay enabled.
  ledcAttach(MOTOR_AIN1_PIN, PWM_FREQ_HZ, PWM_RESOLUTION);
  ledcAttach(MOTOR_AIN2_PIN, PWM_FREQ_HZ, PWM_RESOLUTION);
  ledcWrite(MOTOR_AIN2_PIN, 0);
#if FAN_ALWAYS_ON
  Serial.println("[FAN] FAN_ALWAYS_ON=1: fan on from boot.");
  setFanDuty(FAN_ALWAYS_ON_DUTY);
#elif FAN_BOOT_TEST_MS > 0
  Serial.println("[FAN] boot test: motor should spin now...");
  setFanDuty(255);
  delayPollingDht(FAN_BOOT_TEST_MS);
  setFanDuty(0);
  Serial.println("[FAN] boot test done. Didn't spin? Check VCC/VM power, shared GND, OUT1/OUT2.");
#endif

#if ENABLE_DUTY_CYCLE
  dutyWarmUp();   // DUTY_WARMUP_MIN from power-on before the reading that counts
#endif

  runReadingCycle();

#if ENABLE_DEEP_SLEEP
  // Don't sleep while someone may be choosing a network on the setup page.
  while (wifiSetupActive()) delayPollingDht(50);
  goToSleep();
#else
  Serial.println("ENABLE_DEEP_SLEEP=0 (testing mode) - looping instead of sleeping.");
#endif
}

// ---------------------------------------------------------------------------
// One full "take a reading, show it, upload it" pass. Called once from
// setup() always, and repeatedly from loop() when deep sleep is disabled
// for testing.
// ---------------------------------------------------------------------------
unsigned long lastCycleStartMs = 0;

void runReadingCycle() {
  lastCycleStartMs = millis();
  pollSensors();
#if ENABLE_RS485
  wqBlankSample(readings.ammonia_mgL);   // no-op unless WQ BLANK is running
#endif
  analyzeWaterQuality();
  controlCooling(readings.waterTemp_C);

  // ID is the chip's factory ID the IsdaSafe website registers over USB.
  Serial.printf(
    "DO=%.2f mg/L  pH=%.2f  NH3=%.2f mg/L  FreeNH3=%.3f mg/L%s  WaterT=%.1fC  AirT=%.1fC  Hum=%.0f%%  ID=%s\n",
    readings.dissolvedOxygen_mgL, readings.ph, readings.ammonia_mgL,
    readings.freeNh3_mgL, freeNh3Alarm ? " [ALARM]" : "",
    readings.waterTemp_C, readings.ambientTemp_C, readings.humidity_pct,
    deviceId().c_str()
  );

  // Only upload path: Supabase ingest_sensor_reading() with the pairing
  // token. Missing values (NaN) are sent as null.
#if ENABLE_UPLOAD
  if (wqWarmingUp()) {
    Serial.println("Warm-up: reading NOT uploaded (probe still settling).");
  } else {
#if !ENABLE_DEEP_SLEEP
    static bool uploadedOnce = false;
    static unsigned long lastUploadMs = 0;
    if (!uploadedOnce || millis() - lastUploadMs >= UPLOAD_INTERVAL_MS) {
#endif
      if (WiFi.status() != WL_CONNECTED && !wifiTrying()) WiFi.reconnect();
      // Uploads it (plus anything saved while offline), or saves it for later.
      submitReading(readings.dissolvedOxygen_mgL, readings.ph, readings.ammonia_mgL,
                    readings.freeNh3_mgL, readings.waterTemp_C, readings.ambientTemp_C, readings.humidity_pct);
#if !ENABLE_DEEP_SLEEP
      uploadedOnce = true;
      lastUploadMs = millis();
    }
#endif
  }
#else
  Serial.println("ENABLE_UPLOAD=0 (testing mode) - reading NOT sent to Supabase.");
#endif

#if ENABLE_DEEP_SLEEP
  // Briefly rotate through the LCD pages (3 readings each) so the reading
  // is visible on-device before powering down.
  for (lcdPage = 0; lcdPage < lcdPageCount(); lcdPage++) {
    updateLcd();
    delayPollingDht(LCD_PAGE_DISPLAY_MS);
  }
#else
  // Continuous mode: just refresh the current page; loop() rotates pages on
  // its own timer so the LCD never blocks the next reading.
  updateLcd();
#endif
}

void loop() {
#if ENABLE_DEEP_SLEEP
  // Never reached: goToSleep() at the end of setup() puts the ESP32 into
  // deep sleep, which restarts execution from setup() on wake.
#else
  // Testing mode: deep sleep is disabled, so read continuously -- a new
  // reading every TEST_LOOP_INTERVAL_MS (start to start) -- while the LCD
  // rotates to the next page every LCD_PAGE_DISPLAY_MS independently.
#if ENABLE_RS485
  runPendingWqCommand();   // WQ ... commands typed in the Serial Monitor
#endif
  static unsigned long lastPageMs = millis();
  if (millis() - lastPageMs >= LCD_PAGE_DISPLAY_MS) {
    lastPageMs = millis();
    lcdPage = (lcdPage + 1) % lcdPageCount();
    updateLcd();
  }
  if (millis() - lastCycleStartMs >= TEST_LOOP_INTERVAL_MS) {
    runReadingCycle();
  }
  delayPollingDht(20);
#endif
}
