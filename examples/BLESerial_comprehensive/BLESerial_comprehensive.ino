#include <Arduino.h>
#include <BLESerial.h>
#include <Linereader.h>

constexpr unsigned long BAUDRATE     = 2'000'000UL;
constexpr uint8_t       LED_PIN      = LED_BUILTIN;
constexpr bool          useTaskPump  = true;       // set to 'false' to use polling mode

BLESerial               ble;
LineReader<128>         lr;

char                    line[128];                 // command line buffer
char                    data[256];                 // data output buffer
bool                    paused          = true;    // do not generated data until user requests it
unsigned long           lastBlinkUs     = 0;       // last LED blink time (microseconds)
unsigned long           blinkIntervalUs = 100'000; // LED blink interval (microseconds)
bool                    ledState        = LOW;     // current LED state
unsigned long           lastDataUs      = 0;       // last data rate calc time (microseconds)
unsigned long           dataCount       = 0;
unsigned long           dataCountPrev   = 0;
unsigned long           currentTime;
unsigned long           rate            = 0;

void setup() {
  // Initialize LED
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  Serial.begin(BAUDRATE);
  while (!Serial && millis() < 5000) { delay(10); }

  Serial.println(F("=================================================================="));
  Serial.println(F("BLE Serial Test Program"));
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
        BLESerial::Mode::Fast, 
        "BLESerialDevice", 
        false
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

  // Command Receiver
  // =======================================================
  if (lr.poll(ble, line, sizeof(line))) { 
    auto reply = [&](const char* msg){
      Serial.println(msg);
      ble.write(reinterpret_cast<const uint8_t*>(msg), strlen(msg));
      ble.write(reinterpret_cast<const uint8_t*>("\r\n"), 2);
    };

    if (strcasecmp(line, "pause") == 0) {
      paused = true;
      reply("TX paused");
    } else if (strcasecmp(line, "resume") == 0) {
      paused = false;
      reply("TX resumed");
    } else if (strcasecmp(line, ".") == 0) {
      const char* modeStr = "Balanced";
      switch (ble.getMode()) {
        case BLESerial::Mode::Fast:      modeStr = "Fast";      break;
        case BLESerial::Mode::LowPower:  modeStr = "LowPower";  break;
        case BLESerial::Mode::LongRange: modeStr = "LongRange"; break;
        default: break;
      }
      snprintf(data, sizeof(data),
            "Status: connected=%d mode=%s rssi=%ddB",
            ble.connected(), modeStr, ble.getRssi());
      reply(data);
      snprintf(data, sizeof(data),
            "TX: buffered=%u free=%u bytesTx=%lu txDrops=%lu interval=%luµs chunk=%lu",
            (unsigned)ble.getTxBuffered(), (unsigned)ble.getTxFree(),
            ble.getBytesTx(), ble.getTxDrops(), ble.getInterval(), ble.getChunkSize());
      reply(data);
      snprintf(data, sizeof(data),
            "RX: buffered=%u free=%u bytesRx=%lu rxDrops=%lu",
            (unsigned)ble.getRxBuffered(), (unsigned)ble.getRxFree(),
            ble.getBytesRx(), ble.getRxDrops());
      reply(data);
      snprintf(data, sizeof(data),
            "Link: llOctets=%lu llTime=%luµs mtu=%u phy=%s",
            ble.getLlOctets(), ble.getLlTimeUs(), ble.getMtu(), ble.getPhy());
      reply(data);
    } else if (strcasecmp(line, "?") == 0) {
      reply("help:\r\n"
            "  pause  - pause data transmission\r\n"
            "  resume - resume data transmission\r\n"
            "  .      - BLESerial status report\r\n"
            "  ?      - this help message");
    } else {
      snprintf(data, sizeof(data),
            "Unknown command: %s", line);
      reply(data);
    }
  } // end command receiver

  // Data Generator
  // =======================================================
  if (!paused && ble.connected() && ble.writeReady()) {
    if (currentTime - lastDataUs >= 1'000'000UL) {
      rate          = dataCount - dataCountPrev;
      lastDataUs    = currentTime;
      dataCountPrev = dataCount;
    }

    // Emit a fixed-length line of exactly 36 bytes (on 32-bit unsigned long):
    //   "count="(6) + %10lu (10) + " rate="(6) + %10lu (10) + "/s\r\n"(4) = 36
    // %10lu right-aligns with leading spaces (no zeros)
    int dataLen = snprintf(data, sizeof(data),
            "count=%10lu rate=%10lu/s\r\n", dataCount++, rate);
    ble.write(reinterpret_cast<const uint8_t*>(data), (size_t)dataLen);
  } // end data generator

  // Blink LED
  // =======================================================
  if ((currentTime - lastBlinkUs) >= blinkIntervalUs) {
    lastBlinkUs = currentTime;
    ledState = !ledState;
    digitalWrite(LED_PIN, ledState);
    blinkIntervalUs = ledState ? 100'000 : 900'000;
  } // end blink

} // end loop
