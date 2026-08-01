## Implemented API

### Lifecycle

* begin(mode, deviceName, securityMode) – Initialize BLE stack, create service & characteristics, start advertising.
* begin(mode, deviceName, securityMode, pairingPolicy) – Same as above, with explicit onboarding policy for new peers.
* end() – Stop advertising, disconnect client (if any), free resources, reset internal state.

Supported modes are Fast, LongRange, LowPower, and Balanced. Fast adapts between
2M and 1M PHY; LongRange remains coded and adapts between S=2 and S=8;
LowPower and Balanced remain on 1M. See `Operation_Modes.md` for connection,
MTU, power, and pacing defaults. PHY changes require consecutive filtered-RSSI
decisions and use separate downgrade and upgrade confirmation counts.
Supported security is None, JustWorks and PasskeyDisplay.
Supported pairing policies are AlwaysOpen, Windowed and BondedOnly.

### Stream / I/O (Arduino Stream-compatible where applicable)

* available() / readAvailable() – Bytes currently buffered in RX ring.
* read() / read(dst, n) – Pop one / up to n bytes from RX ring.
* peek() / peek(dst, n) – Inspect one / up to n bytes without consuming.
* write(b) / write(buf, n) – Reliable bounded enqueue to TX ring using the
  configured write timeout. Returns the actual number of bytes queued.
* write(const char*) / print(str) / println(str) – Reliable bounded text output.
  `println()` adds CRLF only after the payload is accepted.
* printf(fmt, ...) – Reliable bounded formatted print into TX ring; truncates at
  the local formatting buffer size.
* writeNonBlocking(...) / printfNonBlocking(fmt, ...) – Explicit non-blocking TX
  enqueue for high-rate streaming; may be partial if flow control blocks.
* writeTimeout(buf, n, timeoutMs) – Attempt to enqueue up to n bytes within
  timeout; cooperatively waits for buffer space/unlock; decoupled from in-flight
  notifies (pop-based staging).
* setWriteTimeoutMs(timeoutMs) / getWriteTimeoutMs() – Configure/query the
  default timeout used by `write()`, `print()`, `println()`, and `printf()`. The
  default is 250 ms.
* writeReady() – True if a client is subscribed and producer lock is not asserted.
* writeAvailable() – Remaining free space in TX ring (capacity - used).

### TX Write Behavior

* Normal Arduino `Print` paths are reliable and bounded by default. `write()`,
  `print()`, `println()`, and `printf()` wait up to `getWriteTimeoutMs()` for
  TX ring space before returning a short count.
* The timeout applies only to enqueueing bytes into BLESerial's TX ring. Actual
  BLE notification delivery remains asynchronous and is handled by the TX state
  machine and pump mode.
* High-rate streaming code should use `writeReady()` plus
  `writeNonBlocking()` or `printfNonBlocking()` to preserve the high-water /
  low-water producer lock behavior without blocking the producer loop.
* Always check the returned byte count. A short count means only that many bytes
  were queued; the unqueued remainder is counted in `TxDrops` when the write
  started while subscribed.

### Pumping / Scheduling

* update() – Call periodically when in Polling pump mode (non-ESP32 or when PumpMode::Polling selected) to advance TX/RSSI logic.
* flush() – Drain queued and in-flight TX within its bounded wait. In task mode it wakes the TX worker; in polling mode it pumps locally.
* setPumpMode(Polling | Task) – Select manual loop pumping or ESP32 FreeRTOS background task. If task creation fails, the library falls back to Polling; use `getPumpMode()` in the loop.
* getPumpMode() – Current mode.

### Adaptive TX Pacing

* Successful notifications establish the last-known-good (LKG) pacing interval.
  After a clean success window, bounded probes may reduce the interval toward the
  negotiated connection-event share.
* An isolated congestion result such as `ENOMEM` cancels an active probe and
  returns to the exact confirmed LKG. The connection-event share is used only
  when no confirmed LKG exists.
* General congestion must continue for at least one second before increasing the
  LKG. High-water congestion may react sooner, but requires at least eight
  consecutive high-water congestion events spanning 250 ms.
* Congestion escalation is gradual and capped at four times the computed pacing
  floor. After 64 successful notifications, probing may resume.
* `getBytesTx()` is a cumulative count of successfully transmitted application
  payload bytes. The library does not calculate a bytes-per-second rate.

### Link / Radio

* setPreferredMTU(newMtu) – Set the local preferred ATT MTU, clamped to 23–517. Call before `begin()` when possible. This does not initiate MTU exchange; a connected central negotiates the active MTU. Before `begin()` it returns configuration success; after initialization it returns the underlying NimBLE result.
* getPreferredMTU() – Local ATT MTU preference used for future peer exchanges.
* requestMTU(newMtu) – Deprecated compatibility alias for `setPreferredMTU()`.
* setPower(dBm, scope) – Adjust transmit power for Advertising, Scanning, Connection, or All.

### Security

* `Security::None` – No BLE pairing or link encryption required.
* `Security::JustWorks` – Encrypted bonded LE Secure Connections link without passkey entry.
* `Security::PasskeyDisplay` – Encrypted bonded connection with displayed 6-digit passkey and MITM protection.
* `PairingPolicy::AlwaysOpen` – Current/default behavior. New peers may pair whenever they connect.
* `PairingPolicy::Windowed` – New peers may pair only while a temporary pairing window is open. Bonded peers may always reconnect.
* `PairingPolicy::BondedOnly` – New peers are rejected by default. Bonded peers may reconnect. A temporary pairing window can still be opened manually.
* `setPairingPolicy(policy)` – Change onboarding policy at runtime.
* `getPairingPolicy()` – Return current onboarding policy.
* `openPairingWindow(durationMs)` – Temporarily allow new peers to pair, typically after a physical button press.
* `closePairingWindow()` – Close the temporary pairing window immediately.
* `isPairingWindowOpen()` – True while the temporary pairing window is still active.

Notes:
* Pairing policies matter only for secure modes (`JustWorks`, `PasskeyDisplay`). With `Security::None`, the library does not gate clients by bond state.
* Existing users do not need to change code. `begin(mode, deviceName, securityMode)` still behaves as before because the default pairing policy is `AlwaysOpen`.
* Typical appliance flow: initialize with `BondedOnly`, then call `openPairingWindow(60000)` when a hardware button is pressed.

### Diagnostics / Logging

* setLogLevel(level) / getLogLevel() – Control the shared `UUtzinger_logger` verbosity (`LOG_LEVEL_NONE`, `LOG_LEVEL_ERROR`, `LOG_LEVEL_WARN`, `LOG_LEVEL_INFO`, `LOG_LEVEL_DEBUG`). A sketch may instead call `logSetLevel()` directly; `begin()` preserves that global setting. Debug output is compiled only when `DEBUG` is defined before including the logger.
* setDiagnosticOutput(output) – Select the dedicated `Print` destination for
  BLESerial-owned diagnostics. It defaults to `Serial`, accepts UARTs and custom
  `Print` implementations such as a flash-file adapter, and returns `false`
  without changing the destination when passed the BLESerial object itself.
  Configure it before `begin()` and keep the supplied object alive while
  BLESerial may emit diagnostics.
* BLESerial diagnostics do not follow the global `logSetOutput()` destination.
  Application `LOG()` / `LOGln()` output is unaffected and may target BLE.
* printStats([stream]) – Emit current link, buffer, and error counters.

### Status / Introspection

* isConnected() – True if a client connection is active.
* isSubscribed() – True if client subscribed to TX characteristic (notify/indicate enabled).
* getMode() – Configured operating Mode (Fast, LowPower, LongRange, Balanced).
* getMtu() – Active negotiated ATT MTU; returns 23 while disconnected.
* isEncrypted() – True if connection has active encryption.
* getPhy() – "1M", "2M", or "Coded" (current PHY).
* getChunkSize() – Current notify payload chunk size (bytes per notification attempt).
* getInterval() – Current pacing interval in microseconds between notifications.
* getLkgInterval() – Current last-known-good pacing interval (floor for probing/backoff).
* getMinInterval() – Current computed minimum viable send interval based on negotiated link parameters.
* getRSSI() – Smoothed RSSI average.
* getMac() – Device MAC address string.
* getllTxOctets() / getllTxTimeUs() – Negotiated Link Layer TX octets and DLE
  maximum TX time. The maximum time is a controller capability, not actual PDU
  airtime; `printStats()` reports estimated TX airtime separately.
* getllRxOctets() / getllRxTimeUs() – Negotiated Link Layer RX octets and DLE
  maximum RX time.
* getBytesTx() / getBytesRx() – Cumulative application payload bytes transmitted / received.
* getTxUsed() / getRxUsed() – Bytes currently queued in TX / RX rings.
* getTxCapacity() / getRxCapacity() – Total ring capacities.
* getTxFree() / getRxFree() – Remaining free space in TX / RX rings.
* getTxDrops() / getRxDrops() – Dropped bytes. TX drops include producer-side
  write timeout remainders and discarded staged TX bytes. RX drops include
  evicted queued bytes and received bytes beyond ring capacity.
* getLowWaterMark() / getHighWaterMark() – Current low/high water marks used for TX flow control.
* getPairingPolicy() – Current onboarding policy for new peers.
* isPairingWindowOpen() – True while new peer pairing is temporarily permitted.

### Flow Control Concepts

* Tx state machine – Internal: transmission advances through Waiting → Staging → Pending → Recovering/Discarding based on notify outcomes
* txLocked – Internal: producer lock engaged (high-water limit reached; unlocks at low-water).
  Use writeReady() before writeNonBlocking()/printfNonBlocking() and check if all data was successfully written.

### Implemented Setters

* setLogLevel(level) – Shared logger verbosity (defaults to Serial unless redirected with `logSetOutput()`); `begin()` does not change a sketch-selected level.
* setDiagnosticOutput(output) – Dedicated BLESerial diagnostic `Print`; defaults
  to `Serial` and rejects the BLESerial object itself.
* setPreferredMTU(mtu) – Set the local ATT MTU preference; a central initiates any MTU exchange.
* getPreferredMTU() – Local ATT MTU preference.
* requestMTU(mtu) – Deprecated compatibility alias for `setPreferredMTU()`.
* getPumpMode() – Current pump mode
* setPumpMode(Polling|Task) – Polling, or ESP32 task mode which runs a background FreeRTOS TX pump; task creation failure falls back to Polling
* setPower() – sets power level for Advertising, Scanning or Connection
* setInterval(interval) – sets transmission interval in microseconds, auto adjusted during runtime
* getInterval() – current pacing interval (µs)
* setWriteTimeoutMs(timeoutMs) / getWriteTimeoutMs() – default bounded wait used by `write()`, `print()`, `println()`, and `printf()`

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
* setOnDataReceived(cb(data, len)) – your custom hook; `data` is valid only during the callback, so copy it if it must be retained
* setOnRxOverflow(cb(lost)) – called with the number of received or queued bytes the RX ring could not retain

### Notes

* Hooks are additive: your callbacks do not replace internal logic; they run after built-in handling. Keep them fast or defer heavy work to your loop/task.
* Pump mode: On ESP32, Task mode uses a FreeRTOS background TX pump; in Polling mode, call update() regularly from loop().
