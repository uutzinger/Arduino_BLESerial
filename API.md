## Implemented API

### Lifecycle

* begin(mode, deviceName, securityMode) – Initialize BLE stack, create service & characteristics, start advertising.
* end() – Stop advertising, disconnect client (if any), free resources, reset internal state.

Supported mode is Fast, Long Range, Low Power and Balanced.
Supported security is None, JustWorks and PasskeyDisplay.

### Stream / I/O (Arduino Stream-compatible where applicable)

* available() / readAvailable() – Bytes currently buffered in RX ring.
* read() / read(dst, n) – Pop one / up to n bytes from RX ring.
* peek() / peek(dst, n) – Inspect one / up to n bytes without consuming.
* write(b) / write(buf, n) – Non-blocking enqueue to TX ring (may be partial if flow control blocks). Returns actual number of bytes written.
* write(const char*) / print(str) / println(str) – Convenience text output (println adds CRLF).
* printf(fmt, ...) – Formatted print into TX ring (truncates at local buffer size).
* writeTimeout(buf, n, timeoutMs) – Attempt to enqueue up to n bytes within timeout; cooperatively waits for buffer space/unlock; decoupled from in-flight notifies (pop-based staging).
* writeReady() – True if a client is subscribed and producer lock is not asserted.
* writeAvailable() – Remaining free space in TX ring (capacity - used).

### Pumping / Scheduling

* update() – Call periodically when in Polling pump mode (non-ESP32 or when PumpMode::Polling selected) to advance TX/RSSI logic.
* flush() – Drain TX ring by pumping until empty (non-blocking relative to BLE airtime; may return early if pacing prevents immediate sends).
* setPumpMode(Polling | Task) – Select manual loop pumping or ESP32 FreeRTOS background task.
* getPumpMode() – Current mode.

### Link / Radio

* requestMTU(newMtu) – Request peer to negotiate a higher MTU (subject to controller/peer limits).
* setPower(dBm, scope) – Adjust transmit power for Advertising, Scanning, Connection, or All.

### Diagnostics / Logging

* setLogLevel(level) / getLogLevel() – Control verbosity (NONE, ERROR, WARNING, INFO, DEBUG).
* printStats([stream]) – Emit current link, buffer, and error counters.

### Status / Introspection

* isConnected() – True if a client connection is active.
* isSubscribed() – True if client subscribed to TX characteristic (notify/indicate enabled).
* getMode() – Configured operating Mode (Fast, LowPower, LongRange, Balanced).
* getMtu() – Negotiated ATT MTU.
* isEncrypted() – True if connection has active encryption.
* getPhy() – "1M", "2M", or "Coded" (current PHY).
* getChunkSize() – Current notify payload chunk size (bytes per notification attempt).
* getInterval() – Current pacing interval in microseconds between notifications.
* getLkgInterval() – Current last-known-good pacing interval (floor for probing/backoff).
* getMinInterval() – Current computed minimum viable send interval based on negotiated link parameters.
* getRSSI() – Smoothed RSSI average.
* getMac() – Device MAC address string.
* getllTxOctets() / getllTxTimeUs() – Negotiated Link Layer TX octets/time.
* getllRxOctets() / getllRxTimeUs() – Negotiated Link Layer RX octets/time.
* getBytesTx() / getBytesRx() – Cumulative application payload bytes transmitted / received.
* getTxUsed() / getRxUsed() – Bytes currently queued in TX / RX rings.
* getTxCapacity() / getRxCapacity() – Total ring capacities.
* getTxFree() / getRxFree() – Remaining free space in TX / RX rings.
* getTxDrops() / getRxDrops() – Dropped bytes due to buffer saturation.
* getLowWaterMark() / getHighWaterMark() – Current low/high water marks used for TX flow control.

### Flow Control Concepts

* Tx state machine – Internal: transmission advances through Waiting → Staging → Pending → Recovering/Discarding based on notify outcomes
* txLocked – Internal: producer lock engaged (high-water limit reached; unlocks at low-water).
  Use writeReady() before writing and check if all data was successfully written.

### Implemented Setters

* setLogLevel(level) – Verbosity level of logging (occurs on serial port)
* requestMTU(mtu) – Modify  MTU, will result in renegotiation with client
* getPumpMode() – Current pump mode
* setPumpMode(Polling|Task) – Polling, or ESP32 task mode which runs a background FreeRTOS TX pump
* setPower() – sets power level for Advertising, Scanning or Connection
* setInterval(interval) – sets transmission interval in microseconds, auto adjusted during runtime
* getInterval() – current pacing interval (µs)

### Implemented Status Queries

* isConnected() – true if client is connected
* isSubscribed() – true if client has TX notify/indicate enabled
* getMtu() – current negotiated MTU
* getMode() – Fast, LongRange, LowPower, or Balanced
* getBytesRx() / getBytesTx() – total bytes received/sent
* getRxDrops() / getTxDrops() – dropped bytes
* getLkgInterval() / getMinInterval() – current LKG and minimum intervals (µs)
* getRSSI() – smoothed RSSI
* getMac() – device MAC address
* getTxUsed() / getRxUsed() – bytes currently buffered in TX/RX rings
* getTxCapacity() / getRxCapacity() – ring capacities
* getTxFree() / getRxFree() – free space in rings
* isEncrypted() – connection encryption status
* getllTxOctets() / getllTxTimeUs() / getllRxOctets() / getllRxTimeUs() – link-layer negotiated octets/time
* getChunkSize() – current notify payload chunk size
* getPhy() – "1M", "2M", or "Coded"

### Event Hooks (additive)

* setOnClientConnect(cb(addr)) – your custom hook
* setOnClientDisconnect(cb(addr, reason)) –  your custom hook
* setOnMtuChanged(cb(mtu)) –  your custom hook
* setOnSubscribeChanged(cb(subscribed)) –  your custom hook
* setOnDataReceived(cb(data, len)) –  your custom hook
* setOnRxOverflow(cb(lost)) – called when RX ring overwrites oldest data

### Notes

* Hooks are additive: your callbacks do not replace internal logic; they run after built-in handling. Keep them fast or defer heavy work to your loop/task.
* Pump mode: On ESP32, Task mode uses a FreeRTOS background TX pump; in Polling mode, call update() regularly from loop().
