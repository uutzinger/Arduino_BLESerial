# BLE Serial Library

## Introduction

BLESerial allows serial communication over a BLE connection <img src="./assets/Bluetooth_Logo.svg" height="20"/>.

It implements the [**Nordic UART Service (NUS)**](https://docs.nordicsemi.com/bundle/ncs-latest/page/nrf/libraries/bluetooth/services/nus.html) as a server on a micro controller, providing commands like:

```
BLESerial ble;
ble.println("Hello");
n=ble.available();
ble.read(buffer,n);
```

It attempts to adapt for maximum throughput, low power usage or long distance communication.

The library provides a server implementation as it is designed to work with programs such as:
- [SerialUI](https://github.com/uutzinger/SerialUI) 
- [nRF connect for mobile](https://www.nordicsemi.com/Products/Development-tools/nRF-Connect-for-mobile)

There are similar implementations from other authors ([senseshift](https://github.com/senseshift/arduino-ble-serial), [afpineda](https://github.com/afpineda/NuS-NimBLE-Serial)).

A throughput of more than **100k bytes/s** and a latency of **10..20 ms** was achieved.

## Installation

Installation occurs through the Arduino library manager.

## Dependencies

- [NimBLE-Arduino](https://github.com/h2zero/NimBLE-Arduino)
- RingBuffer (provided)

## Quick Start

Minimal example demonstrating setup, polling versus task (ESP32) mode, command parsing, and date transmission and receiving:

```cpp
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

  // Security::None | JustWorks | PasskeyDisplay
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
```

## Documentation
- [API Documentation](./API.md)
- [Operation Modes](./Operation_Modes.md)
- [To Do](./TODO.md)
- [Change Log](./CHANGELOG.md)

## Example Programs
- BLESerial_minimal (simple echo program)
- BLESerial_demo (simple program listed above)
- BLESerial_comprehensive (generates data for performance measurements)

## Contributing

Urs Utzinger 2025

ChatGPT (OpenAI)

## License

See [LICENSE](License.txt).
