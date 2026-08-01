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


## Prevent Recursive BLESerial Diagnostics

### Problem Statement

When `MAX30001G_BLESerial` routes the shared logger to `ble`, BLESerial's own TX
adaptation messages can be written back into the same TX queue that is currently
being drained. In the malformed `?` help capture, an internal
`Probe 1948 accepted` message appears inside a help row, two complete rows are
missing, and the following row is joined without its expected CRLF.

The capture strongly suggests self-directed BLESerial diagnostics, but does not
yet prove that recursion is the byte-loss mechanism. SerialUI normally retains
partial BLE lines until EOL and has a 64 KiB input buffer. Manually stopping its
receiver is a separate case because unsubscribing can lose upstream output.

### P0 - Reproduce and Confirm

- [x] Record `ble.printStats(Serial)` immediately before and after each test and
  compare `TxDrops`, `Timeouts`, congestion events, and software errors.
- [x] At the default INFO level (`l 3`), issue `?` repeatedly and capture the raw
  bytes, including pauses, injected BLESerial diagnostics, missing rows, merged
  rows, total byte count, and final CRLF.
- [x] Repeat the same sequence at `l 2`. This suppresses BLESerial INFO diagnostics
  while leaving the sketch's unconditional `LOGln()` help output enabled. The
  compact command documented by the sketch as `l2` currently sets level `0`
  because its parser reads the value starting at character three.
- [x] Repeat with an independent BLE UART client to separate BLESerial behavior
  from SerialUI rendering and notification handling.
- [x] Confirm whether each reported stop was a passive output pause or a manual
  SerialUI receiver stop; do not combine those cases in the result.
- [x] Add a deterministic BLESerial regression case that routes
  `logSetOutput(ble)`, triggers TX adaptation diagnostics while sending a payload
  larger than the TX ring, and validates byte count, checksum, line endings, and
  final sentinel.

### Confirmed P0 Evidence (2026-08-01)

- A raw Linux BlueZ/D-Bus notification capture reproduced the same missing rows
  and merged CRLFs without SerialUI and without manually stopping reception.
- A complete help response is `6043` bytes with `79` CRLFs and SHA-256
  `2008900b170658850fe422b9ee968ac3ec20ba1ed5ab78ed57b47fb0f583d1b5`.
- At INFO, three stats-correlated responses changed `Bytes TX` by `5711`, `6092`,
  and `5877` bytes. The first added `431` `TxDrops` and `20` timeouts; the second
  added no drops but transmitted `49` bytes beyond the help payload; the third
  added `216` drops and `10` timeouts.
- At WARN, the response changed `Bytes TX` by exactly `6043` while `TxDrops`
  remained `647` and `Timeouts` remained `30`.
- These controls confirm the loss occurs in the BLESerial/logger path, not in
  SerialUI. Suppressing INFO is a valid temporary workaround, but the library
  still needs to prevent any level of internal diagnostic from targeting its
  own BLE transport.
- The `BLESerial_text_stress` `selflog` command reproduces the recursive path
  deterministically with a 21,605-byte payload. Before the fix it added `84`
  `TxDrops` and `6` timeouts. With dedicated diagnostics it sent the same
  `21,605` bytes with zero drops/timeouts; its
  transient `ENOMEM` warning appeared only on USB Serial. The BlueZ capture
  contained the final checksum sentinel and `BLE_WRITE_SUMMARY ... short=0`.
- With dedicated diagnostics, five MAX30001G `?` responses at INFO increased
  `Bytes TX` by exactly `6043` bytes each (`30215` total). `TxDrops=0` and
  `Timeouts=0` remained unchanged. Probe start and
  acceptance messages appeared on USB Serial but did not change the BLE payload
  count. A persistent BlueZ subscriber received the help notifications from
  beginning through the final CRLF.

### P0 - Dedicated Internal Diagnostics

- [x] Add one library-wide diagnostic helper that writes to a dedicated `Print`
  destination, defaulting to `Serial`, without reading or changing the
  application's global logger destination.
- [x] Apply the dedicated route to all BLESerial diagnostics, including GAP/characteristic
  callbacks, TX/RSSI tasks, probe success/start messages, congestion recovery,
  timeout handling, and disconnect/error paths; fixing only the observed probe
  message is insufficient.
- [x] Keep application writes through `Print`, `LOG()`, and `LOGln()` unchanged.
  Global `logSetOutput(ble)` remains valid for application output.
- [x] Allow `Serial1`, `Serial2`, and application-provided `Print`
  implementations as BLESerial diagnostic destinations, but reject the
  BLESerial object itself without changing the existing destination.
- [x] Remove the translation-unit `#undef LOG*` wrappers and route every
  BLESerial-owned diagnostic explicitly through the dedicated helper.
- [x] Avoid temporarily swapping the shared logger output because callbacks and
  application tasks may log concurrently.
- [x] Ensure dedicated diagnostics do not recurse, block the TX worker, mutate
  TX queue contents, increment application `TxDrops`/`Timeouts`, or alter link
  adaptation state.
- [x] Document the dedicated-routing behavior and new diagnostic API in `API.md`,
  `README.md`, and `CHANGELOG.md`.

### P1 - Automated and Compile Validation

- [x] Add focused coverage for global application logging to BLE while BLESerial
  diagnostics remain on their dedicated non-BLE `Print`; verify self-selection
  is rejected.
- [ ] Force an ERROR path and verify it cannot recursively fill the queue during
  timeout or disconnect handling. WARN routing was hardware-verified with a
  transient `ENOMEM` event during `selflog`.
- [ ] Re-run the deterministic text-stress test and all examples in polling and
  task modes, with `DEBUG` both enabled and disabled.
- [x] Compile all BLESerial examples for `esp32:esp32:nano_nora` with warnings
  enabled and explicit local logger and RingBuffer dependencies.
- [x] Compile `MAX30001G_BLESerial` against the local BLESerial,
  `UUtzinger_logger`, `UUtzinger_RingBuffer`, and `UUtzinger_MAX30001G`
  checkouts; confirm no new warnings or `LOG_LEVEL_*` redefinitions.
- [ ] Publish `UUtzinger_logger` 2.2.1 before BLESerial 1.3.2. The BLESerial
  metadata already declares `UUtzinger_logger (>=2.2.1)`.

### P0 - Hardware Acceptance Criteria

- [x] Repeated `?` output is byte-for-byte complete over BLE at `l 3`: no missing
  `Er`/`En` rows, merged `Eq` row, injected BLESerial diagnostic, or missing CRLF.
- [ ] The same capture passes in SerialUI and an independent BLE UART client.
- [x] `TxDrops` and `Timeouts` do not increase during a completed help response.
- [x] BLESerial diagnostics still reach a separately configured non-BLE sink,
  while self-directed diagnostic configuration is rejected.
- [ ] High-rate non-blocking streaming, adaptation, pacing, and congestion
  recovery behavior remain unchanged.

### Implementation Order

1. Run the `l 3`/`l 2` control and preserve before-fix stats and raw captures.
2. Introduce the centralized self-routing guard and migrate every internal log
   call to it.
3. Add regression coverage for self-directed and separate logger outputs.
4. Run warning-enabled BLESerial and MAX30001G builds plus offline stress tests.
5. Repeat the hardware captures and close the acceptance items only from
   byte-count/checksum/stat evidence.

### Evidence Capture

The first command completed without a visible pause. The second paused twice and
shows both missing content/CRLF and an injected BLESerial diagnostic:

```text
[INFO] BLESerial: Client e0:d4:64:23:f6:1c subscribed (notify) 6e400003-b5a3-f393-e0a9-e50e24dcca9e.
================================================================================
| MAX30001G ECG and Bio-Impedance Program                                      |
| 2026 Urs Utzinger & GPT                                                      |
================================================================================
| GENERAL COMMANDS                       | DATA COMMANDS                       |
|----------------------------------------|-------------------------------------|
| ?: help screen                         | z: toggle data reporting on/off     |
| s: show current settings               | z[s|b|a]: USB, BLE, both, off       |
| h: run health check                    | z[S|B|A]: USB, BLE, both, on        |
| i: print device info                   | c: reset sample counter             |
| r: print all registers                 | f: FIFO reset                       |
| t: print status registers              | p: print config registers           |
================================================================================
| PERSISTENCE                            | RAM SNAPSHOT                        |
|----------------------------------------|-------------------------------------|
| Ps: save NVS preferences               | (: save volatile register snapshot  |
| Pl: load NVS preferences               | ): restore volatile register snap   |
| Pd: print NVS preferences              |                                     |
| Pc: clear NVS preferences              |                                     |
|========================================|=====================================|
| OPERATION MODES (auto-stop previous)   | START/STOP                          |
|----------------------------------------|-------------------------------------|
| m1: ECG mode                           | .: toggle start/stop                |
| m2: BIOZ mode                          | >: start measurement                |
| m3: ECG + BIOZ mode                    | <: stop measurement                 |
| m4: ECG signal calibration             |                                     |
| m5: BIOZ signal calibration            |                                     |
| m6: BIOZ internal impedance            |                                     |
| m7: BIOZ external impedance            |                                     |
| m8: BIOZ impedance spectroscopy        |                                     |
|========================================|=====================================|
| ECG SETTINGS                           | BIOZ SETTINGS                       |
|----------------------------------------|-------------------------------------|
| Es<n>: speed      (0-2)     Es1        | Bs<n>: speed      (0-1)     Bs0     |
| Eg<n>: gain       (0-3)     Eg2        | Bg<n>: gain       (0-3)     Bg1     |
| El<n>: dig LPF    (0-3,255) El255      | Ba<n>: analog HPF (0-7)     Ba1     |
| Eh<n>: dig HPF    (0-1,255) Eh255      | Bd<n>: digital LPF(0-3)     Bd1     |
| Ee<n>: leads      (2 or 3)  Ee3        | Bh<n>: digital HPF(0-3)     Bh0     |
| Er<n>: R-to-R     (0=off,1) Er1        |                                     |
| En<n>: notch  (0=off,50,60) En0        | Bf<n>: frequency Hz         Bf8000  |
| Eq<n>: notch Q    (1-100)   Eq20       | Bc<n>: current nA           Bc8000  |
|                                        | Bp<n>: phase deg            Bp0     |
|                                        | Bl<n>: lead bias  (0=off,1) Bl1     |
|                                        | Bo<n>: lead-off   (0=off,1) Bo0     |
|                                        | Bw<n>: wires      (2 or 4)  Bw2     |
|========================================|=====================================|
| BIOZ SCAN SETTINGS                     | INTERNAL CALIBRATION SETTINGS       |
|----------------------------------------|-------------------------------------|
| Sa<n>: averages   (1-8)     Sa8        | Cr<n>: internal resistor    Cr1000  |
| Sf<n>: fast mode  (0=off,1) Sf0        | Cm<n>: cal modulation(0-3)  Cm0     |
| Sr<n>: full range (0=off,1) Sr0        | Cf<n>: mod frequency(0-4)   Cf3     |
| Si<n>: source     (0=ext,1=int) Si0    | Ce<n>: ECG sig mode(0/1)    Ce1     |
| Sp<n>: phase rng  (0=full,1) Sp0       | Cb<n>: BIOZ sig mode(0/1)   Cb0     |
| Sh<n>: AHPF mode  (0=dyn,1=fix,2=byp)  |-------------------------------------|
| Sv<n>: fixed AHPF (0-7)     Sv1        |                                     |
| Sx<n>: int AHPF   (255,0-7) Sx255      |                                     |
| St<n>: settle     (1-64)    St24       | Sm<n>: max current nA       Sm96000 |
| Sc<n>: cur settle (1-64)    Sc24       | So<n>: autorange  (0=off,1) So1     |
|                                        | Sy<n>: human-safe (0/1)     Sy0     |
|----------------------------------------|-------------------------------------|
| Kp:    print scan calibration          | Kr:  reset scan calibration default |
| Kg<n>: set global K ppm (1250650)      | Ke<n>: enable correction (0/1)      |
|========================================|=====================================|
| LOG LEVEL                              | SPECIAL                             |
|----------------------------------------|-------------------------------------|
| l0: none (silent)                      | w: software reset                   |
| l1: errors only                        | y: synchronize                      |
| l2: warnings                           | k: clear latched status flags       |
| l3: info (default)                     | a: apply current settings (re-setup)|
| l4: debug (verbose)                    |                                     |
================================================================================

Examples:
  m1       - Switch to ECG mode
  Eg3      - Set ECG gain to 160 V/V (level 3)
  Bf40000  - Set BIOZ frequency to 40 kHz
  .        - Start/stop measurement
  z        - Toggle continuous data display

================================================================================
| MAX30001G ECG and Bio-Impedance Program                                      |
| 2026 Urs Utzinger & GPT                                                      |
================================================================================
| GENERAL COMMANDS                       | DATA COMMANDS                       |
|----------------------------------------|-------------------------------------|
| ?: help screen                         | z: toggle data reporting on/off     |
| s: show current settings               | z[s|b|a]: USB, BLE, both, off       |
| h: run health check                    | z[S|B|A]: USB, BLE, both, on        |
| i: print device info                   | c: reset sample counter             |
| r: print all registers                 | f: FIFO reset                       |
| t: print status registers              | p: print config registers           |
================================================================================
| PERSISTENCE                            | RAM SNAPSHOT                        |
|----------------------------------------|-------------------------------------|
| Ps: save NVS preferences               | (: save volatile register snapshot  |
| Pl: load NVS preferences               | ): restore volatile register snap   |
| Pd: print NVS preferences              |                                     |
| Pc: clear NVS preferences              |                                     |
|========================================|=====================================|
| OPERATION MODES (auto-stop previous)   | START/STOP                          |
|----------------------------------------|-------------------------------------|
| m1: ECG mode                           | .: toggle start/stop                |
| m2: BIOZ mode                          | >: start measurement                |
| m3: ECG + BIOZ mode                    | <: stop measurement                 |
| m4: ECG signal calibration             |                                     |
| m5: BIOZ signal calibration            |                                     |
| m6: BIOZ internal impedance            |                                     |
| m7: BIOZ external impedance            |                                     |
| m8: BIOZ impedance spectroscopy        |                                     |
|========================================|=====================================|
| ECG SETTINGS                           | BIOZ SETTINGS                       |
|----------------------------------------|-------------------------------------|
| Es<n>: speed      (0-2)     Es1        | Bs<n>: speed      (0-1)     Bs0     |
| Eg<n>: gain       (0-3)     Eg2        | Bg<n>: gain       (0-3)     Bg1     |
| El<n>: dig LPF    (0-3,255) El255      | Ba<n>: analog HPF (0-7)     Ba1     |
| Eh<n>: dig HPF    (0-1,255) Eh255      | Bd<n>: digital LPF(0-3)     Bd1     |
| Ee<n>: leads      (2 or 3)  Ee3        | Bh<n>: digital HPF(0-3)     Bh0     || Eq<n>: notch Q    (1-100)   Eq20       | Bc<n>: current nA           Bc8000  |
|                                        | Bp<n>: phase deg            Bp0     |
|                                        | Bl<n>: lead bias  (0=off,1) Bl1     |
|                                        | Bo<n>: lead-off   (0=off,1) Bo0     |
|                                        | Bw<n>: wires      (2 or 4)  Bw2     |
|========================================|=====================================|
| BIOZ SCAN SETTINGS                     | INTERNAL CALIBRATION SETTINGS       |
|----------------------------------------|-------------------------------------|
| Sa<n>: averages   (1-8)     Sa8        | Cr<n>: internal resistor    Cr1000  |
| Sf<n>: fast mode  (0=off,1) Sf0        | Cm<n>: cal modulation(0-3)  Cm0     |
| Sr<n>: full range (0=off,1) Sr0        | Cf<n>: mod frequency(0-4)   Cf3     |
| Si<n>: source     (0=ext,1=int) Si0    | Ce<n>: ECG sig mode(0/1)    Ce1     |
| Sp<n>: phase rng  (0=full,1) Sp0       | Cb<n>: BIOZ sig mode(0/1)   Cb0     |
| Sh<n>: AHPF mode  (0=dyn,1=fix,2=byp)  |-------------------------------------|
| Sv<n>: fixed AHPF (0-7)     Sv1        |                                     |
| Sx<n>: int AHPF   (255,0-7) Sx255      |                                     |
| St<n>: settle     (1-64)    St24       | Sm<n>: max current nA       Sm96000 |
| Sc<n>: cur settle (1-64)    Sc24       | So<n>: autorange  (0=off,1) So1     |
|                                        | Sy<n>: human-safe (0/1)     Sy0     |
|----------------------------------------|-------------------------------------|
| Kp:    print scan calibration          | Kr:  reset scan calibration default |
| Kg<n>: set global K ppm (1250650)      | Ke<n>: enable correction (0/1)      |
|========================================|=====================================|
| LOG LEVEL                              | SPECIAL                             |
|----------------------------------------|-------------------------------------|
| l0: none (silent)                      | w: software reset                   |
| l1: errors only                        | y: synchronize                      |[INFO] BLESerial: Probe 1948 accepted. LKG=1948.

| l2: warnings                           | k: clear latched status flags       |
| l3: info (default)                     | a: apply current settings (re-setup)|
| l4: debug (verbose)                    |                                     |
================================================================================

Examples:
  m1       - Switch to ECG mode
  Eg3      - Set ECG gain to 160 V/V (level 3)
  Bf40000  - Set BIOZ frequency to 40 kHz
  .        - Start/stop measurement
  z        - Toggle continuous data display
```
