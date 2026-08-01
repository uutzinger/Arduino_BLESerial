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

- [ ] Make TX task creation failure switch `pumpMode` to `Polling` so `update()` can
  continue transmitting. Keep the warning consistent with the resulting state.
- [ ] Correct the MTU API contract. `NimBLEDevice::setMTU()` sets the preferred local
  MTU but does not initiate renegotiation for an existing server connection. Either
  rename/document the method accordingly or implement a supported exchange path.
- [ ] Keep requested and negotiated MTU state separate. `getMtu()` must continue to
  report the negotiated value, and `requestMTU()` must return the actual NimBLE result.
- [ ] Add centralized `begin()` failure cleanup. Clear the custom GAP handler and
  active instance, deinitialize partial NimBLE state, release callbacks/resources,
  and leave the object reusable after any failed initialization step.
- [ ] Own the dynamically allocated RX and TX characteristic callbacks and release
  them during `end()` and failed initialization. NimBLE-Arduino does not delete
  characteristic callback objects.
- [ ] Avoid overriding the application's global logger level in `begin()`. A level set
  through `logSetLevel()` by the sketch or another library must remain effective;
  document that `setLogLevel()` controls the shared logger.
- [ ] Record `lastTxUs` at the actual task-mode transmission attempt, after any pacing
  delay, so consecutive notifications respect `sendIntervalUs`.

### P2 - Improvements and Cleanup

- [ ] Revisit two-PDU transmission in `Mode::Fast` when NimBLE-Arduino supports it.
  Re-enable the two-PDU limit in `computeTxChunkSize()` and keep ATT/L2CAP header,
  MIC, `computeSendIntervalUs()`, and `PDUS_PER_WINDOW` calculations consistent.
- [ ] Remove or replace the deprecated no-op `NimBLEService::start()` call.
- [ ] Resolve warning-enabled build findings: unused variables and deprecated
  increment operations on volatile-qualified fields. Use explicit synchronization
  rather than treating `volatile` as thread safety.
- [ ] Review lifecycle resets so `end()` and `clearStats()` consistently reset all
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

- [ ] Compile every example for `esp32:esp32:nano_nora` with explicit local
  `UUtzinger_RingBuffer` and `UUtzinger_logger` dependencies and distinct build paths.
- [ ] Compile with warnings enabled and treat new library warnings as regressions.
- [ ] Compile at least one example with `DEBUG` defined and one without it.
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
- [ ] Force congestion and timeout recovery, then confirm bounded retries, pacing
  backoff, last-known-good recovery, and eventual disconnect behavior.
- [ ] Disconnect during queued and in-flight TX. Confirm `txState` returns to Waiting,
  `pendingLen` is cleared, locks are reconciled, and advertising restarts correctly.
- [ ] Measure notification spacing in task mode and confirm no interval is shorter
  than the configured/computed pacing floor after scheduler delays.
- [ ] Run long-duration throughput and reconnect tests while monitoring heap usage,
  task handles, callback allocation, RX/TX drops, and error counters.
