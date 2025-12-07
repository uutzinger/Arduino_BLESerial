## Operation Modes

Short recipes for common goals. Adjust as needed for your environment.

### Fast (throughput + low latency)

```cpp
ble.begin(BLESerial::Mode::Fast, "BLEFast", BLESerial::Security::None);
ble.requestMTU(BLE_SERIAL_MAX_MTU);   // if peer supports
ble.setPower(+6, PWR_CONN);           // tune +3..+9 dBm
```

Tips:
- Prefer 2M PHY when RSSI is good; fall back to 1M automatically.
- Let the library probe pacing only after clean success streaks.
- If you see congestion/timeouts, increase `sendIntervalUs` slightly or pause probing.

### Balanced (middle ground)

```cpp
ble.begin(BLESerial::Mode::Balanced, "BLEBalanced", BLESerial::Security::None);
ble.requestMTU(BLE_SERIAL_DEFAULT_MTU);
ble.setPower(+3, PWR_CONN);           // tune +0..+6 dBm
```

Tips:
- Keep interval in the 15–30 ms range.
- Use probing conservatively; prioritize stability.

### LongRange (coverage on coded PHY)

```cpp
ble.begin(BLESerial::Mode::LongRange, "BLELongRange", BLESerial::Security::None);
ble.requestMTU(BLE_SERIAL_MIN_MTU);
ble.setPower(+9, PWR_CONN);           // within local/regulatory limits
```

Tips:
- Coded S=2/S=8 selected automatically based on RSSI thresholds.
- Pace sends above the minimum to avoid collisions under coded.
- Consider slightly longer supervision timeout for flaky links.

### LowPower (minimize airtime)

```cpp
ble.begin(BLESerial::Mode::LowPower, "BLELowPower", BLESerial::Security::None);
ble.requestMTU(BLE_SERIAL_MIN_MTU);
ble.setPower(0, PWR_CONN);            // tune -6 .. 0 dBm conservative power
ble.setPumpMode(BLESerial::PumpMode::Polling); // update transmission in main loop
```

Tips:
- Larger intervals (60–120 ms) and latency 8 reduce wakeups.
- Disable or greatly relax probing.
