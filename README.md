# BLE Serial Library

## Introduction

BLESerial is a library that allows serial communication over a BLE connection. It works in a similar fashion as a Serial USB connection. For example, we use `ble.write()` instead of `Serial.print()`. It implements the Nordic UART Service (NUS).

Implemented functions (Stream-compatible unless noted):

* begin(mode, deviceName, secure) – Initialize the driver
* end() – Remove the driver
* readAvailable() – amount of bytes in the receiver buffer
* read() / read(*dst, n) – read from receiver buffer
* peek() / peek(*dst, n) – peak from receiver buffer
* write(b) / write(*b, n) – write to transmission buffer
* writeTimeout(*p, n, timeoutMs)
* writeReady() – returns whether transmitter will accept writes
* writeAvailable() – returns number of bytes left in the tramission buffer
* flush() – clears transmission buffer
* update() – call this in loop when using Polling pump mode

Implemented setters:

* setLogLevel(level) – Verbosity level of logging on serial port
* requestMTU(mtu) – Modify  MTU
* getPumpMode() – current pump mode
* setPumpMode(Polling|Task) – Polling, or ESP32 task mode which runs a background FreeRTOS TX pump
* setPower() – sets power level for Advertising, Scanning or Connection

Implemented status queries:

* connected() –  true if client is connected
* getMtu() – current MTU in use
* getMode() – Fast, Longrange, Lowpower or Balanced
* getBytesRx() / getBytesTx() – total bytes sent or received
* getRxDrops() / getTxDrops() –  bytes dropped
* getInterval() – current send interval in µs
* getRssi() – current signal strength
* getMac() – current MAC address
* getTxBuffered() / getRxBuffered() – bytes in the transmission or receiver buffer
* getTxCapacity() / getRxCapacity() – size of transmission or receiver buffer
* getTxFree() / getRxFree() - remaining space in transmission or receiver buffer
* isEncrypted() – connection encryption status
* getLlOctets() / getLlTimeUs() – link-layer negotiated data length/time
* getChunkSize() – current notify payload chunk size
* getPhy() – "1M", "2M", or "Coded"

Event hooks (additive):

* setOnClientConnect(cb(addr)) – your custom hook
* setOnClientDisconnect(cb(addr, reason)) –  your custom hook
* setOnMtuChanged(cb(mtu)) –  your custom hook
* setOnSubscribeChanged(cb(subscribed)) –  your custom hook
* setOnDataReceived(cb(data, len)) –  your custom hook
* setOnRxOverflow(cb(lost)) – called when RX ring overwrites oldest data

Notes:

* Hooks are additive: your callbacks do not replace internal logic; they run after built-in handling. Keep them fast or defer heavy work to your loop/task.
* Pump mode: On ESP32, Task mode uses a FreeRTOS background TX pump; in Polling mode, call update() regularly from loop().


## Installation

Installation occurs through the Arduino library manager.

## Dependencies

* [NimBLE-Arduino](https://github.com/h2zero/NimBLE-Arduino)
* RingBuffer (provided)

You will need a client program like [SerialUI](https://github.com/uutzinger/SerialUI) to communicate with your micro controller as Arduino IDE Serial Monitor does not yet have NUS/BLESerial support.

## Quick Start

```cpp
#include <BLESerial.h>
#include <Linereader.h>

BLESerial               ble;
LineReader<128>         lr;

void setup() {
  ble.begin(BLESerial::Mode::Fast, "BLESerialDevice", false);
  #ifdef ARDUINO_ARCH_ESP32
    ble.setPumpMode(BLESerial::PumpMode::Polling);
  #endif
}

void loop(){

  ble.update();

  // read commands
  if (lr.poll(ble, line, sizeof(line))) { 
      auto reply = [&](const char* msg){
        Serial.println(msg);
        ble.write(reinterpret_cast<const uint8_t*>(msg), strlen(msg));
        ble.write(reinterpret_cast<const uint8_t*>("\r\n"), 2);
      };

    if (strcasecmp(line, "?") == 0) {
      reply(helpmsg);
    } else if (...) {
        ...
    }
  }
  
  ... generate data
  ble.write(reinterpret_cast<const uint8_t*>(data), (size_t)dataLen);
}

```

## Contributing

Urs Utzinger 2025

ChatGPT (OpenAI)

## License

See [LICENSE](License.txt).
