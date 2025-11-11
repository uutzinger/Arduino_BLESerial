#include <Arduino.h>
#include <BLESerial.h>

BLESerial ble;

void setup() {
 
  Serial.begin(115200);
  while (!Serial && millis() < 3000) { delay(10); }

  Serial.println(F("BLE Serial Minimal Program"));

  if (!ble.begin(
        BLESerial::Mode::Fast, 
        "BLESerialDevice", 
        false
       )
     )  {
    Serial.println(F("BLESerial begin() failed"));
    while (true) delay(1000);
  }
  // For polling mode demonstration explicitly set pump mode
  // otherwise set PumpMode::Task on ESP32s
  ble.setPumpMode(BLESerial::PumpMode::Polling);
  Serial.println(F("Ready. Connect with a BLE UART client."));
}


void loop() {

  // BLE Serial Update if using BLESerial::PumpMode::Polling
  // otherwise comment this out
  ble.update(); // polling pump

  // Echo any received bytes back out
  while (ble.available() > 0) {
    int c = ble.read();
    if (c >= 0) {
      ble.write((uint8_t)c); // echo
    }
  }
  // Optional: small delay to reduce busy loop CPU usage
  delay(1);
}


