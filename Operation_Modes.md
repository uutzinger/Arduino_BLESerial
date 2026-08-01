## Operation Modes

Short recipes for common goals. Adjust as needed for your environment.

### Fast (throughput + low latency)

```cpp
ble.setPreferredMTU(BLE_SERIAL_MAX_MTU); // central negotiates when it connects
ble.begin(BLESerial::Mode::Fast, "BLEFast", BLESerial::Security::None);
ble.setPower(+6, PWR_CONN);           // tune +3..+9 dBm
```

Tips:
- Prefer 2M PHY when RSSI is good; fall back to 1M only after three consecutive
  weak filtered-RSSI samples. Returning to 2M requires six strong samples.
- Fast does not select coded PHY; use LongRange when coded PHY is required.
- Let the library probe pacing only after clean success streaks.
- Isolated congestion returns to the exact confirmed last-known-good pace.
  General congestion must persist for one second; high-water escalation requires
  at least eight events over 250 ms. Backoff is capped at four times the computed
  pacing floor.

### Balanced (middle ground)

```cpp
ble.setPreferredMTU(BLE_SERIAL_DEFAULT_MTU); // central negotiates when it connects
ble.begin(BLESerial::Mode::Balanced, "BLEBalanced", BLESerial::Security::None);
ble.setPower(+3, PWR_CONN);           // tune +0..+6 dBm
```

Tips:
- Keep interval in the 15–30 ms range.
- Balanced stays on 1M PHY for broad compatibility and stable pacing.
- Use probing conservatively; prioritize stability.

### LongRange (coverage on coded PHY)

```cpp
ble.setPreferredMTU(BLE_SERIAL_MIN_MTU); // central negotiates when it connects
ble.begin(BLESerial::Mode::LongRange, "BLELongRange", BLESerial::Security::None);
ble.setPower(+9, PWR_CONN);           // within local/regulatory limits
```

Tips:
- Coded S=2/S=8 is selected automatically from confirmed filtered-RSSI decisions;
  S=8 requires three weak samples and returning to S=2 requires six strong samples.
- LongRange remains on coded PHY; RSSI adaptation changes only S=2 versus S=8.
- Pace sends above the minimum to avoid collisions under coded.
- Consider slightly longer supervision timeout for flaky links.

### LowPower (minimize airtime)

```cpp
ble.setPreferredMTU(BLE_SERIAL_MAX_MTU); // efficient bursts; short writes stay short
ble.begin(BLESerial::Mode::LowPower, "BLELowPower", BLESerial::Security::None);
ble.setPower(0, PWR_CONN);            // tune -6 .. 0 dBm conservative power
ble.setPumpMode(BLESerial::PumpMode::Polling); // update transmission in main loop
```

Tips:
- Larger intervals (60–120 ms) and latency 8 reduce wakeups.
- LowPower stays on 1M PHY. Its large preferred MTU reduces notification overhead
  for queued bursts without padding short writes.
- Pacing probes remain bounded by the negotiated connection-event share.
