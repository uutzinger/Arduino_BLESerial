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

