/*
BLE Serial Minimal Program

This program demonstrates the BLESerial library by echoing received data back to the sender.

Urs Utzinger
November/December 2025
*/
#include <Arduino.h>
#include <BLESerial.h>

BLESerial ble;

void setup() {
 
  Serial.begin(115200);
  while (!Serial && millis() < 3000) { delay(10); }

  Serial.println(F("BLE Serial Minimal Program"));

  
  if (!ble.begin(
        BLESerial::Mode::Fast,        /*Fast, LowPower, LongRange, Balanced*/
        "BLESerialDevice",            /*Name*/
        BLESerial::Security::None /*None, JustWorks, PasskeyDisplay*/
       )
     )  {
    Serial.println(F("BLESerial begin() failed"));
    while (true) delay(1000);
  }
  // ESP32 can transmit data in a background task. 
  // To force polling mode set Polling pump.
  // Non ESP32 boards only support polling mode.
  ble.setPumpMode(BLESerial::PumpMode::Polling); /*Polling or Task (Esp32)*/
  Serial.println(F("Ready. Connect with a BLE UART client."));
}


void loop() {

  // BLE Serial Update required in main loop if using PumpMode::Polling
  //   otherwise comment this out
  ble.update(); // polling pump

  // Echo any received bytes back out
  while (ble.available() > 0) {
    int c = ble.read();
    if (c >= 0) {
      ble.write((uint8_t)c); // echo
    }
  }
  // Small delay to reduce busy loop CPU usage
  delay(1);
}


