# TODO

Priorities:

- **P0:** Correctness or data-integrity issue; address before the next release.
- **P1:** Reliability, lifecycle, or API-contract issue; address in the next development cycle.
- **P2:** Improvement or cleanup with lower immediate risk.

## Implement Error Fixes

### P0 - Correctness and Data Integrity

- [x] Preserve RX callback data lifetime. Invoke `onDataReceived()` before clearing the
  NimBLE characteristic value, or copy the received value into stable storage before
  passing it to user code.
- [x] Make `flush()` safe in `PumpMode::Task`. Do not run `pumpTx()` concurrently from
  the caller and the TX worker; wake the worker and wait for the queue/in-flight frame
  to drain within the existing timeout.
- [x] Correct RX overflow accounting. Calculate how many existing bytes will be
  overwritten before calling `RingBuffer::push(..., true)`, update `rxDrops`, and call
  `onRxOverflow()` with the actual number of discarded bytes.

### P1 - Reliability and API Contracts

- [x] Make TX task creation failure switch `pumpMode` to `Polling` so `update()` can
  continue transmitting. Keep the warning consistent with the resulting state.
- [x] Correct the MTU API contract. `setPreferredMTU()` sets the preferred local
  MTU; a server cannot initiate renegotiation for an existing connection.
- [x] Keep preferred and negotiated MTU state separate. `getMtu()` reports the
  negotiated value, while `getPreferredMTU()` reports the local preference.
- [x] Add centralized `begin()` failure cleanup. Clear the custom GAP handler and
  active instance, deinitialize partial NimBLE state, release callbacks/resources,
  and leave the object reusable after any failed initialization step.
- [x] Own the dynamically allocated RX and TX characteristic callbacks and release
  them during `end()` and failed initialization. NimBLE-Arduino does not delete
  characteristic callback objects.
- [x] Avoid overriding the application's global logger level in `begin()`. A level set
  through `logSetLevel()` by the sketch or another library must remain effective;
  document that `setLogLevel()` controls the shared logger.
- [x] Record `lastTxUs` at the actual task-mode transmission attempt, after any pacing
  delay, so consecutive notifications respect `sendIntervalUs`.

### P2 - Improvements and Cleanup

- [ ] Revisit two-PDU transmission in `Mode::Fast` when NimBLE-Arduino supports it.
  Re-enable the two-PDU limit in `computeTxChunkSize()` and keep ATT/L2CAP header,
  MIC, `computeSendIntervalUs()`, and `PDUS_PER_WINDOW` calculations consistent.
- [x] Remove or replace the deprecated no-op `NimBLEService::start()` call.
- [x] Resolve warning-enabled build findings: unused variables and deprecated
  increment operations on volatile-qualified fields. Use explicit synchronization
  rather than treating `volatile` as thread safety.
- [x] Review lifecycle resets so `end()` and `clearStats()` consistently reset all
  documented counters, retry state, pending state, locks, and buffers.

## Test and Validate the Library

### P0 - Regression Tests for Correctness Fixes

- [ ] Verify `onDataReceived()` receives byte-for-byte correct data after the
  characteristic value is cleared, including maximum-size writes.
- [ ] Fill the RX ring, force overwrite, and verify retained data ordering,
  `rxDrops`, and `onRxOverflow(lost)` exactly match the discarded byte count.
- [ ] In task mode, write continuously while calling `flush()` and verify there are
  no duplicate, missing, reordered, or corrupted notifications.
- [ ] Force TX task creation failure and verify the library enters polling mode and
  continues transmitting when `update()` is called.

### P1 - Build and Automated Validation

- [x] Compile every example for `esp32:esp32:nano_nora` with explicit local
  `UUtzinger_RingBuffer` and `UUtzinger_logger` dependencies and distinct build paths.
- [x] Compile with warnings enabled and treat new library warnings as regressions.
- [x] Compile at least one example with `DEBUG` defined and one without it.
- [ ] Verify all logger levels (`DEBUG`, `INFO`, `WARN`, `ERROR`, `NONE`) and confirm
  application-level `logSetLevel()` and `logSetOutput()` survive `begin()`.
- [ ] Add host-testable coverage for chunk sizing, pacing calculations, watermarks,
  retry transitions, timeout rollover, and lifecycle state resets.
- [ ] Add repeated `begin()`/`end()` and failed-`begin()` tests, checking that the
  object can be reused and that callbacks/tasks/resources do not leak.
- [ ] Validate preferred versus negotiated MTU behavior before connection, during a
  connection, after peer negotiation, and after reconnect.

### P1 - Hardware and Fault-Injection Validation

- [ ] Exercise `Polling` and `Task` pump modes in Fast, Balanced, LowPower, and
  LongRange modes with sustained bidirectional traffic.
- [ ] Validate `Security::None`, `JustWorks`, and `PasskeyDisplay`, including
  AlwaysOpen, Windowed, and BondedOnly pairing policies.
- [ ] Force `EMSGSIZE` with a negotiated MTU of 23 and confirm `txChunkSize` decreases
  or clamps to `MIN_CHUNKSIZE`, timing is recomputed, probing stops, and cooldown is set.
- [ ] Force isolated and sustained congestion, then confirm bounded retries,
  cooldown, controlled last-known-good escalation, recovery toward the pacing
  floor, and eventual disconnect behavior for non-recoverable failures.
- [ ] Disconnect during queued and in-flight TX. Confirm `txState` returns to Waiting,
  `pendingLen` is cleared, locks are reconciled, and advertising restarts correctly.
- [ ] Measure notification spacing in task mode and confirm no interval is shorter
  than the configured/computed pacing floor after scheduler delays.
- [ ] Run long-duration throughput and reconnect tests while monitoring heap usage,
  task handles, callback allocation, RX/TX drops, and error counters.

## Diagnose BLE Text Truncation

### Problem Statement

Large terminal-style output over BLE can be truncated or misaligned when a sketch
prints long help menus or logs through `Print::print()` / `Print::println()`.
This was observed with `MAX30001G_BLESerial`: a help-menu line can be followed
immediately by `Mode changed to...` without the expected line ending.

The original library-level failure was that `BLESerial::write()` could return a
partial count or `0` when `txLocked` was set or the TX buffer was congested.
Arduino `Print` callers did not retry those bytes. Reliable bounded writes now
address that path.

Current test evidence:

- USB baseline is complete: `END_OF_TEST bytes=21509 lines=160 checksum=64C22A3F`
  and `USB_WRITE_SUMMARY requested=21562 short=0`.
- After the reliable-write fix, the independent Android BLE Serial terminal
  reaches `END_OF_TEST` with `short=0`; the original mixed-capture interleaving
  is therefore most likely in the desktop terminal capture path.

### P0 - Reproduce and Isolate

- [x] Add a deterministic BLE text-stress example or test command that emits at
  least 8 KB, exceeding the current 4 KB TX buffer.
- [x] Include numbered lines, fixed-width fields, explicit line endings, total
  byte count, line count, checksum, and a final sentinel such as
  `END_OF_TEST bytes=<n> checksum=<hex>`.
- [x] Send the identical text over USB `Serial` as a baseline to prove the source
  text and formatting are complete before BLE transport.
- [x] Capture BLE output with the normal terminal client.
- [x] Compare USB and BLE captures for byte count, line count, checksum, sentinel,
  missing line endings, and reordered or merged lines.
- [x] Capture with a second independent BLE UART client after the
  library fix if the normal terminal still shows anomalies.

### P0 - Targeted Library Fix

- [x] Fix `println(const char*)` so it returns the actual byte count and does not
  report `+2` unless both CR and LF were accepted.
- [x] Confirm `println()` never appends CR/LF after a partial payload write unless
  the remaining payload bytes have been accepted or the bounded write timeout
  has expired.
- [x] Confirm `flush()` drains queued bytes and pending in-flight notifications in
  both task and polling modes, remains bounded, and returns after disconnect.

### P1 - Reliability Fix

- [x] Make default `Print::write()` behavior reliable and bounded for terminal/log
  output, likely by routing `write(uint8_t)` and `write(const uint8_t*, size_t)`
  through `writeTimeout()` with a default timeout.
- [x] Update `writeTimeout()` to wake/pump while waiting for TX space and to
  return only the bytes actually queued before timeout or disconnect.
- [x] Ensure short writes increment TX drop/timeout diagnostics consistently
  instead of silently losing bytes.
- [x] Add configurable timeout accessors, such as
  `setWriteTimeoutMs(uint32_t)` and `getWriteTimeoutMs()`, or reuse an existing
  timeout setting if one is appropriate.
- [x] Preserve an explicit non-blocking API for high-rate streaming, such as
  `writeNonBlocking(...)`, `tryWrite(...)`, or documented `writeReady()` usage.
- [x] Document the difference between reliable bounded `Print` writes and the
  explicit non-blocking streaming path.

### Omitted

- Broad TX enqueue tracing is omitted because the stress test and BLE stats
  already identify library-level TX drops.
- Extra diagnostic counters are omitted for the first fix because existing stats
  already report `TxDrops`, congestion, and software errors.
- MAX30001G-specific reproduction is omitted before the library fix; the
  deterministic BLESerial stress test isolates the same failure mode without
  sketch-specific buffering or command routing.

### P1 - Verification After Fix

- [x] Repeat the deterministic BLE text-stress capture and verify byte count,
  line count, checksum, and final sentinel match USB.
- [ ] Verify MAX30001G help-menu output is complete over BLE and is not truncated
  before follow-up command responses.
- [x] Verify high-rate streams still work when using the explicit non-blocking API.

### P1 - Compile Validation

- [x] Compile all BLESerial examples for `esp32:esp32:nano_nora` with explicit
  local `UUtzinger_RingBuffer` and `UUtzinger_logger` dependencies.
- [ ] Compile `MAX30001G_BLESerial` against the local BLESerial,
  `UUtzinger_logger`, `UUtzinger_RingBuffer`, and `UUtzinger_MAX30001G`
  checkouts.
- [ ] Confirm no `LOG_LEVEL_* redefined` warnings return.

### Acceptance Criteria

- [x] Large BLE text output preserves line endings, table text, byte count,
  checksum, and final sentinel.
- [x] `Print::print()` and `Print::println()` over BLE are reliable for normal
  terminal/menu/log use.
- [x] Non-blocking high-rate streaming remains available through an explicit API.
- [ ] Diagnostics distinguish delayed writes, timeout writes, dropped bytes,
  high-water locks, notify failures, and flush timeouts.
- [ ] `MAX30001G_BLESerial` help menus and command responses are complete over BLE.

## Improve Link Adaptation Reliability

Observed comprehensive-test evidence:

- The example reached an enqueue rate of `732` fixed 36-byte lines/s, or 26,352
  bytes/s. This is accepted-to-buffer throughput, not confirmed over-the-air
  throughput; use the `Bytes TX` delta for delivered throughput.
- Isolated raw RSSI excursions (`-108`, `-54`, `-105` dBm) drove repeated 2M/1M
  changes despite the filtered average.
- Repeated `ENOMEM` responses escalated the last-known-good pacing interval from
  about 10.4 ms to more than 40 ms while the TX buffer still had free space.

Post-fix hardware evidence:

- Fast mode negotiated 2M PHY, MTU 517, and a 7.5 ms connection interval. The
  comprehensive stream sustained an enqueue rate of 1,751 fixed 36-byte lines/s.
- Stats reported 13,849,675 transmitted bytes with `TxDrops=0`, `Timeouts=0`, and
  final `send=min=lkg=1968 us` despite 21,398 congestion status events. Persistent
  congestion produced bounded backoff and later recovered to the pacing floor.

### P1 - Implementation

- [x] Require sustained RSSI evidence before changing PHY. Use consecutive
  filtered-RSSI decisions so one raw RSSI excursion cannot trigger a 2M/1M or
  coded-S2/S8 transition.
- [x] Apply separate upgrade and downgrade confirmation criteria and a post-change
  holdoff. Clear pending decisions on reconnect, mode change, disconnect, and a
  failed or externally initiated PHY update.
- [x] Keep mode boundaries explicit: Fast may adapt only between 2M and 1M,
  LongRange only between coded S=2 and S=8, and Balanced/LowPower remain on 1M.
- [x] Treat isolated `ENOMEM` responses as transient congestion. Return to the
  exact confirmed last-known-good interval, using the connection-event share
  only as a hard fallback when no confirmed interval exists.
- [x] Escalate the last-known-good interval only when congestion persists across
  a defined observation window or coincides with repeated high-water TX-buffer
  pressure. Require one second for general persistence or at least eight
  high-water events over 250 ms, and cap congestion backoff at four times the
  computed pacing floor.
- [x] Define recovery after congestion: require a stable success window, probe in
  bounded steps, and return to the mode/link pacing floor after a confirmed PHY,
  MTU, DLE, or connection-interval improvement.
- [x] Distinguish the negotiated DLE maximum TX time from estimated packet airtime
  in status and log output so values such as 17,040 us are not reported as actual
  2M packet airtime.
- [x] Rate-limit repeated congestion warnings while retaining counters and state
  transition logs needed to diagnose backoff and recovery.

### P1 - Hardware Verification

- [ ] Measure sustained throughput before and after the adaptation changes using
  the `Bytes TX` delta; report enqueue throughput separately.
- [ ] Exercise Fast, Balanced, LowPower, and LongRange modes when supported.
- [ ] Verify Fast changes only between 2M and 1M, ignores isolated RSSI outliers,
  does not oscillate near a threshold, and recovers its pacing floor after a
  sustained RSSI improvement.
- [ ] Verify LongRange remains coded and changes between S=2 and S=8 without
  constraining DLE to the initial coding scheme or reacting to isolated outliers.
- [ ] Verify Balanced and LowPower remain on 1M with their requested connection
  intervals, latency, MTU, and power settings.
- [x] At a stable PHY and connection interval, observe intermittent and persistent
  `ENOMEM`; verify isolated events hold the confirmed LKG, persistent congestion
  produces controlled escalation, and successful traffic recovers to the floor.
