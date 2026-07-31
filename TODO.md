## ToDo:

### computeTxChunkSize
We currently force onePduMax (2-PDU disabled). 
Once NimBLE for Arduino supports 2 PDU message size we can retry this attempt.
Re-enable two-PDU for Mode::Fast, ensure computeSendIntervalUs matches (PDUS_PER_WINDOW) and ATT/L2CAP headers logic remains consistent.

### Edge Cases
- Force EMSGSIZE: temporarily set a small MTU (e.g., requestMTU(23)) and send a chunk > 20.
- Timeout: briefly disable notifications on the client to simulate stalled confirmations.
- Disconnect: ensure onDisconnected state resets (txState Waiting, pendingLen 0, lock reconciled).

### Assertions
After onMessageTooBig, confirm:
- txChunkSize decreases or clamps to MIN_CHUNKSIZE.
- minSendIntervalUS recomputed.
- sendIntervalUs >= minSendIntervalUS.
- probing disabled and cooldown set.

### Use Logger library

- setLogLevel(), getLogLevel() should convert to logger's logSetLevel/getLevel function.

- log output should use standrad logger library format. e.g.[INFO] BLESerial: ....

- DEBUG should not be a numeric and BLESerial should use the #define DEBUG option to enable it. If DEBUG is not declared, LOGD should no be compiled into the program.

- The function calls should use LOG..() syntax and if possible. If possible no new loglevel macros should be creteated e.g. no BLE_SERIAL_LOG_NONE

if (logLevel >= INFO) {
  Serial.printf("BLESerial: ...\r\n", ...);
}

should become

LOGI("...", ...);

- [INFO] shall be proceeded with the library name BLESerial:

- When multiple line logging is present consider using multiple LOG statements.

- If multiple LOG statements are needed that combine into single line consider useing LOGIS,LOGIC and LOGIE.

- Update docs

- Verify build

- Do not remove stale comments

- Change logging also in commented out sections.
