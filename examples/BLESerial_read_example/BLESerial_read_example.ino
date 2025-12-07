// Simple BLESerial read() example
// Demonstrates non-blocking reads using available(), read(), and read(buffer, n)
// Prints incoming data to USB Serial, echoes it back over BLE, and handles simple commands
// Use Polling mode for portability across boards.

#include <Arduino.h>
#include "BLESerial.h"

BLESerial ble;

// Small helper to trim CR/LF
static void rstrip(char* s) {
  size_t n = strlen(s);
  while (n && (s[n-1] == '\r' || s[n-1] == '\n')) { s[--n] = '\0'; }
}

void setup() {
  Serial.begin(115200);
  delay(100);

  // Start BLE in Balanced mode for this demo
  ble.begin(BLESerial::Mode::Balanced, "BLEReadDemo", BLESerial::Security::None);

  // Polling pump for portability
  ble.setPumpMode(BLESerial::PumpMode::Polling);

  Serial.println("BLESerial read() demo ready. Connect with a NUS client and type.");
  Serial.println("Examples: type 'echo hello', 'stats', 'ping' or any text.\n");
}

void loop() {
  // Required in Polling mode to service BLE and TX
  ble.update();

  // 1) Non-blocking single-byte read using read() when available() > 0
  while (ble.available() > 0) {
    int c = ble.read(); // returns -1 if none, otherwise 0..255
    if (c < 0) break;
    Serial.write((char)c); // mirror to USB Serial
    // Echo single byte back over BLE
    uint8_t b = (uint8_t)c;
    ble.write(&b, 1);
  }

  // 2) Read into a small buffer using read(buffer, n) when multiple bytes are available
  uint8_t buf[64];
  int toRead = min<int>(ble.readAvailable(), (int)sizeof(buf));
  if (toRead > 0) {
    int n = ble.read(buf, toRead); // returns number of bytes read (<= toRead)
    if (n > 0) {
      // Simple parser: if input starts with 'echo ' then echo back over BLE
      if (n >= 5 && memcmp(buf, "echo ", 5) == 0) {
        ble.write(buf + 5, (size_t)(n - 5));
        const char crlf[] = "\r\n";
        ble.write((const uint8_t*)crlf, sizeof(crlf) - 1);
      }
      // If input starts with 'stats' print stats on Serial
      else if (n >= 5 && memcmp(buf, "stats", 5) == 0) {
        ble.printf("OK: stats on Serial\r\n");
        ble.printStats(Serial);
      }
      // Otherwise, just show what we received on Serial
      else {
        Serial.print("[RX] ");
        for (int i = 0; i < n; ++i) Serial.write((char)buf[i]);
        Serial.println();
      }
    }
  }

  // 3) Read a line (basic) using read() into a small char buffer
  static char line[128];
  static size_t pos = 0;
  while (ble.available() > 0) {
    int ch = ble.read();
    if (ch < 0) break;

    if (pos + 1 < sizeof(line)) {
      line[pos++] = (char)ch;
      // newline terminators handling
      if (ch == '\n') {
        line[pos] = '\0';
        rstrip(line);

        // Handle the line
        if (strcasecmp(line, "ping") == 0) {
          ble.printf("pong\r\n");
        } else if (strcasecmp(line, "help") == 0) {
          ble.printf("Commands: echo <text>, stats, ping\r\n");
        } else {
          ble.printf("You said: %s\r\n", line);
        }

        // reset for next line
        pos = 0;
      }
    } else {
      // buffer full: flush the line
      line[sizeof(line)-1] = '\0';
      rstrip(line);
      ble.printf("(truncated) %s\r\n", line);
      pos = 0;
    }
  }

  // Cooperative delay to keep loop friendly
  delay(1);
}
