
#include <HardwareSerial.h>

HardwareSerial rs485(2);

const uint32_t bauds[] = {4800};
const byte addresses[] = {0x01, 0x02, 0x00, 0xFF};

uint16_t modbusCrc(const byte *data, int len) {
  uint16_t crc = 0xFFFF;
  for (int i = 0; i < len; i++) {
    crc ^= data[i];
    for (int j = 0; j < 8; j++) {
      crc = (crc & 1) ? (crc >> 1) ^ 0xA001 : crc >> 1;
    }
  }
  return crc;
}

void setup() {
  Serial.begin(115200);
  Serial.println("SEN0711 Modbus scan");
}

void loop() {
  for (uint32_t baud : bauds) {
    rs485.begin(baud, SERIAL_8N1, 16, 17);
    delay(50);

    for (byte addr : addresses) {
      // Read 3 registers starting at 0x0000
      byte request[8] = {addr, 0x03, 0x00, 0x00, 0x00, 0x03, 0, 0};
      uint16_t crc = modbusCrc(request, 6);
      request[6] = crc & 0xFF;
      request[7] = crc >> 8;

      // Clear old data
      while (rs485.available()) {
        rs485.read();
      }

      rs485.write(request, sizeof(request));
      rs485.flush();

      // Read incoming data, skipping the 8-byte echo of our own request
      unsigned long start = millis();
      int count = 0;
      String extra = "";

      while (millis() - start < 500) {
        while (rs485.available()) {
          byte b = rs485.read();
          count++;
          if (count > (int)sizeof(request)) {
            if (b < 0x10) extra += '0';
            extra += String(b, HEX);
            extra += ' ';
          }
        }
      }

      int extraCount = count > (int)sizeof(request) ? count - (int)sizeof(request) : 0;
      Serial.printf("baud %u addr %02X: total %d, after echo %d", baud, addr, count, extraCount);
      if (extraCount > 0) {
        Serial.print(" -> ");
        Serial.print(extra);
      }
      Serial.println();
    }

    rs485.end();
  }

  Serial.println("--- scan done, repeating in 5 s ---\n");
  delay(5000);
}
