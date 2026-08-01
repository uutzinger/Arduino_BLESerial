/*
BLE Serial Text Stress Test

This sketch emits a deterministic text block over USB Serial and/or BLESerial.
Use it to check whether large Print-style output is truncated before it reaches
the BLE client.

Commands from USB Serial or BLE:
  ?     show commands
  usb   send the stress block over USB Serial only
  ble   send the stress block over BLE only
  both  send the stress block over USB Serial and BLE
  stats print BLESerial counters to USB Serial

Capture the output and compare:
  - total byte count
  - line count
  - checksum
  - final END_OF_TEST sentinel

  2026 OpenAI Codex
*/

#include <Arduino.h>
#include <BLESerial.h>

BLESerial ble;

constexpr unsigned long BAUDRATE = 115200;
constexpr uint16_t TEST_LINES = 160;
constexpr bool USE_TASK_PUMP = false;

char usbLine[64];
char bleLine[64];

uint32_t fnv1aUpdate(uint32_t hash, const char *text) {
  while (*text) {
    hash ^= static_cast<uint8_t>(*text++);
    hash *= 16777619UL;
  }
  return hash;
}

size_t textLen(const char *text) {
  return strlen(text);
}

void serviceBlePump() {
  if (ble.getPumpMode() == BLESerial::PumpMode::Polling) {
    ble.update();
  }
}

size_t writeLine(Print &out, const char *text) {
  const size_t requested = textLen(text) + 2;
  const size_t written = out.println(text);
  serviceBlePump();
  return (written == requested) ? 0 : (requested - written);
}

void accountRaw(const char *line, uint32_t &checksum, size_t &bytes) {
  checksum = fnv1aUpdate(checksum, line);
  bytes += textLen(line);
}

void accountLine(const char *line, uint32_t &checksum, size_t &bytes) {
  accountRaw(line, checksum, bytes);
  accountRaw("\r\n", checksum, bytes);
}

void emitLine(Print *usbOut, Print *bleOut, const char *line,
              uint32_t &checksum, size_t &bytes,
              size_t &usbShort, size_t &bleShort) {
  accountLine(line, checksum, bytes);
  if (usbOut) usbShort += writeLine(*usbOut, line);
  if (bleOut) bleShort += writeLine(*bleOut, line);
}

void emitStressBlock(bool toUsb, bool toBle) {
  Print *usbOut = toUsb ? static_cast<Print *>(&Serial) : nullptr;
  Print *bleOut = toBle ? static_cast<Print *>(&ble) : nullptr;

  uint32_t checksum = 2166136261UL;
  size_t bytes = 0;
  size_t usbShort = 0;
  size_t bleShort = 0;
  char line[160];

  snprintf(line, sizeof(line),
           "BEGIN_TEXT_STRESS lines=%u payload=deterministic line_endings=CRLF",
           TEST_LINES);
  emitLine(usbOut, bleOut, line, checksum, bytes, usbShort, bleShort);

  for (uint16_t i = 0; i < TEST_LINES; ++i) {
    snprintf(line, sizeof(line),
             "LINE %04u | idx=%04u | hex=%04X | table=%04u,%04u,%04u,%04u | "
             "payload=ABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789abcdefghijklmnopqrstuvwxyz",
             i, i, i, i, static_cast<uint16_t>(i + 1),
             static_cast<uint16_t>(i + 2), static_cast<uint16_t>(i + 3));
    emitLine(usbOut, bleOut, line, checksum, bytes, usbShort, bleShort);
  }

  char sentinel[96];
  snprintf(sentinel, sizeof(sentinel),
           "END_OF_TEST bytes=%lu lines=%u checksum=%08lX",
           static_cast<unsigned long>(bytes), TEST_LINES,
           static_cast<unsigned long>(checksum));

  if (usbOut) usbShort += writeLine(*usbOut, sentinel);
  if (bleOut) bleShort += writeLine(*bleOut, sentinel);

  if (toUsb) {
    Serial.printf("USB_WRITE_SUMMARY requested=%lu short=%lu\r\n",
                  static_cast<unsigned long>(bytes + textLen(sentinel) + 2),
                  static_cast<unsigned long>(usbShort));
  }
  if (toBle) {
    ble.printf("BLE_WRITE_SUMMARY requested=%lu short=%lu\r\n",
               static_cast<unsigned long>(bytes + textLen(sentinel) + 2),
               static_cast<unsigned long>(bleShort));
  }

  if (toBle) {
    ble.flush();
  }
}

void printHelp(Print &out) {
  out.println(F("Commands: ? usb ble both stats"));
  out.println(F("Capture BEGIN_TEXT_STRESS through END_OF_TEST and compare bytes, lines, checksum."));
}

void handleCommand(const char *cmd, bool fromBle) {
  if (strcasecmp(cmd, "?") == 0 || strcasecmp(cmd, "help") == 0) {
    printHelp(fromBle ? static_cast<Print &>(ble) : static_cast<Print &>(Serial));
  } else if (strcasecmp(cmd, "usb") == 0) {
    emitStressBlock(true, false);
  } else if (strcasecmp(cmd, "ble") == 0) {
    if (!ble.isSubscribed()) {
      Serial.println(F("BLE client is not subscribed; cannot send BLE stress block."));
      return;
    }
    emitStressBlock(false, true);
  } else if (strcasecmp(cmd, "both") == 0) {
    if (!ble.isSubscribed()) {
      Serial.println(F("BLE client is not subscribed; sending USB baseline only."));
      emitStressBlock(true, false);
      return;
    }
    emitStressBlock(true, true);
  } else if (strcasecmp(cmd, "stats") == 0) {
    ble.printStats(Serial);
  } else if (cmd[0] != '\0') {
    Print &out = fromBle ? static_cast<Print &>(ble) : static_cast<Print &>(Serial);
    out.println(F("Unknown command. Type ? for help."));
  }
}

bool readLine(Stream &in, char *line, size_t maxLen) {
  static size_t usbPos = 0;
  static size_t blePos = 0;
  size_t &pos = (&in == &Serial) ? usbPos : blePos;

  while (in.available() > 0) {
    const int c = in.read();
    if (c < 0) break;
    if (c == '\r') continue;
    if (c == '\n') {
      line[pos] = '\0';
      pos = 0;
      return true;
    }
    if (pos + 1 < maxLen) {
      line[pos++] = static_cast<char>(c);
    }
  }
  return false;
}

void setup() {
  Serial.begin(BAUDRATE);
  while (!Serial && millis() < 3000) { delay(10); }

  Serial.println(F("BLE Serial Text Stress Test"));
  Serial.println(F("Type ? for commands."));

  ble.setPumpMode(USE_TASK_PUMP ? BLESerial::PumpMode::Task : BLESerial::PumpMode::Polling);

  if (!ble.begin(BLESerial::Mode::Fast,
                 "BLESerialTextStress",
                 BLESerial::Security::None)) {
    Serial.println(F("BLESerial begin() failed"));
    while (true) delay(1000);
  }

  ble.setWriteTimeoutMs(2000);

  Serial.println(F("Ready. Connect with a BLE UART client and subscribe to TX."));
}

void loop() {
  serviceBlePump();

  if (readLine(Serial, usbLine, sizeof(usbLine))) {
    handleCommand(usbLine, false);
  }

  if (readLine(ble, bleLine, sizeof(bleLine))) {
    handleCommand(bleLine, true);
  }

  serviceBlePump();
  delay(1);
}
