/*
BLE Serial Test Program

This program toggles the LED and reports the LED state over BLE and Serial.
Recording the terminal output and the LED with a camera allows measuring
the the latency of the BLE and Serial link. 

Urs Utzinger
December 2025
*/
#include <Arduino.h>
#include <BLESerial.h>

constexpr unsigned long BAUDRATE        = 2'000'000UL;
constexpr uint8_t       LED_PIN         = LED_BUILTIN;
constexpr bool          useTaskPump     = false;       // set to 'false' to use polling mode

BLESerial               ble;
unsigned long           lastBlinkUs     = 0;       // last LED blink time (microseconds)
unsigned long           blinkIntervalUs = 100'000; // LED blink interval (microseconds)
bool                    ledState        = LOW;     // current LED state
unsigned long           currentTime;
uint8_t                 counter = 0;

void setup() {
  // Initialize LED
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  Serial.begin(BAUDRATE);
  while (!Serial && millis() < 5000) { delay(10); }

  Serial.println(F("=================================================================="));
  Serial.println(F("BLE Serial Latency Test Program"));
  Serial.println(F("=================================================================="));

  // Initialize PSRAM (optional check)
  #ifdef ARDUINO_ARCH_ESP32
    if (psramInit()) {
      Serial.printf("PSRAM: total=%d free=%d\r\n", ESP.getPsramSize(), ESP.getFreePsram());
    } else {
      Serial.println("No PSRAM available.");
    }
  #endif

  ble.setPumpMode(useTaskPump ? BLESerial::PumpMode::Task : BLESerial::PumpMode::Polling);

  if (!ble.begin(
        BLESerial::Mode::Fast,        /*Fast, LowPower, LongRange, Balanced*/
        "BLESerialDevice",            /*Name*/
        BLESerial::Security::None     /*None, JustWorks, PasskeyDisplay*/
       )
     )  {
    Serial.println(F("BLESerial begin() failed"));
    while (true) delay(1000);
  }

  // Increase verbosity on serial logging
  // DEBUG   shows all events; 
  // INFO    shows key events, warnings and errors, 
  // WARNING shows problems and errors
  // ERROR   shows errors only
  // NONE    turns off logging
  ble.setLogLevel(INFO);

  // Optionally configure parameters
  // ble.setPower(BLE_TX_DBP9, PWR_ALL); // set transmit power, check src/BLESerial.h for options
  // ble.requestMTU(517);                // set desired MTU, max 517, default is 247, minimum 23

}

void loop() {
  currentTime = micros();

  // BLE Serial Update
  // =======================================================
  if (!useTaskPump) {
    ble.update(); // polling pump
  }

  // Blink LED and send blink status over BLE and Serial
  // =======================================================
  if ((currentTime - lastBlinkUs) >= blinkIntervalUs) {
    lastBlinkUs = currentTime;
    ledState = !ledState;
    digitalWrite(LED_PIN, ledState);
    blinkIntervalUs = ledState ? 100'000 : 900'000;
    ble.printf("%d",counter);
    ble.println(ledState ? "B+" : "B-");
    Serial.printf("%d",counter);
    Serial.println(ledState ? "U+" : "U-");
    counter++;
  } // end blink

} // end loop
