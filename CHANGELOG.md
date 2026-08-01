## Changelog
- July 2026 version 1.3.0: corrected the MTU API with separate preferred and negotiated values (`setPreferredMTU()`, `getPreferredMTU()`); retained `requestMTU()` as a deprecated compatibility alias. Hardened RX/TX lifecycle handling, task-mode pacing and fallback behavior, callback ownership, logger-level preservation, and initialization cleanup. Added offline warning-enabled build validation.
- July 2026 version 1.2.1: switched BLESerial diagnostics to `UUtzinger_logger`.
- March 2026 version 1.2.0: added pairing policies and pairing window control; switched RingBuffer to external dependency `UUtzinger_RingBuffer`.
- November 2025 error handling of onStatus, buffer optimizations.
- November 2025 first release
