/*
BLE Serial Demo Program

This program demonstrates the BLESerial library with minimalistic example.
It sets up a BLE UART service and echoes commands received from a connected client.

Urs Utzinger
November/December 2025
*/

#include <Arduino.h>
#include "BLESerial.h"
#include "Linereader.h"

BLESerial        ble;
LineReader<128>  lr;

char line[128];
const char helpmsg[] = "Commands: ?=help, stats, echo <text>";

void setup() {
  Serial.begin(115200);
  while (!Serial) { /* wait for USB serial */ }

  // SecurityMode::None | JustWorks | PasskeyDisplay
  // Mode::Fast | LowPower | LongRange | Balanced
  ble.begin(BLESerial::Mode::Fast, "BLESerialDevice", BLESerial::Security::None);

  #ifdef ARDUINO_ARCH_ESP32
    ble.setPumpMode(BLESerial::PumpMode::Task); // background TX pump
  #endif

  Serial.println("BLESerial demo started.");
}

void loop() {
  #ifndef ARDUINO_ARCH_ESP32
    ble.update(); // required in Polling mode
  #endif

  // Parse incoming lines from BLE
  if (lr.poll(ble, line, sizeof(line))) {
    if (strcasecmp(line, "?") == 0) {
      ble.println(helpmsg);
      Serial.println(helpmsg);
    } else if (strcasecmp(line, "stats") == 0) {
      ble.printStats(); // shows on Serial port
    } else if (strncasecmp(line, "echo ", 5) == 0) {
      ble.println(line + 5);
    } else {
      ble.println("Unknown command. Type ? for help.");
    }
  }
}