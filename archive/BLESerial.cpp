// ****************************************************************************************************
// BLE Serial Library
//
// BLE Serial Communication for Arduino using NimBLE
// This creates a Nordic UART Service (NUS) allowing to send and receive serial data over BLE
// in a similar fashion as Serial.print, Serial.read, etc.
// ****************************************************************************************************
// This code is maintained by
// Urs Utzinger, November 2025
// ****************************************************************************************************
#include <algorithm>
#include <cctype>
#include <cstdarg> 
#include "BLESerial.h"

#ifdef ARDUINO_ARCH_ESP32
  TaskHandle_t BLESerial::rssiTaskHandle = nullptr;
  TaskHandle_t BLESerial::txTaskHandle   = nullptr;
#endif

BLESerial *BLESerial::active = nullptr;

// ==============================================================================================
// ==============================================================================================

// ===== Server Callbacks =======================================================================

class BLESerial::ServerCallbacks : public NimBLEServerCallbacks {
public:
  explicit ServerCallbacks(BLESerial *owner) : owner(owner) {}

  void onConnect(NimBLEServer *srv, NimBLEConnInfo &connInfo) override{
    if (!owner) return;
    auto &s = *owner;

    // Peer address as string
    s.peerAddr = connInfo.getAddress().toString();

    s.deviceConnected = true;
    s.connHandle = connInfo.getConnHandle();
    s.linkEncrypted = connInfo.isEncrypted();
    s.probing = false;
    s.eappRetries = 0;

    uint16_t minItvl, maxItvl, latency, supTimeout;
    switch (s.mode) {
      case Mode::Fast:
        minItvl    = MIN_BLE_INTERVAL_SPEED;
        maxItvl    = MAX_BLE_INTERVAL_SPEED;
        latency    = BLE_SLAVE_LATENCY_SPEED;
        supTimeout = BLE_SUPERVISION_TIMEOUT_SPEED;
        break;
      case Mode::LowPower:
        minItvl    = MIN_BLE_INTERVAL_LOWPWR;
        maxItvl    = MAX_BLE_INTERVAL_LOWPWR;
        latency    = BLE_SLAVE_LATENCY_LOWPWR;
        supTimeout = BLE_SUPERVISION_TIMEOUT_LOWPWR;
        break;
      case Mode::LongRange:
        minItvl    = MIN_BLE_INTERVAL_LONG_RANGE;
        maxItvl    = MAX_BLE_INTERVAL_LONG_RANGE;
        latency    = BLE_SLAVE_LATENCY_LONG_RANGE;
        supTimeout = BLE_SUPERVISION_TIMEOUT_LONG_RANGE;
        break;
      case Mode::Balanced:
      default:
        minItvl    = MIN_BLE_INTERVAL_BALANCED;
        maxItvl    = MAX_BLE_INTERVAL_BALANCED;
        latency    = BLE_SLAVE_LATENCY_BALANCED;
        supTimeout = BLE_SUPERVISION_TIMEOUT_BALANCED;
        break;
    }

    (void)srv->updateConnParams(s.connHandle, minItvl, maxItvl, latency, supTimeout);

    // Ask for desired PHY per our current policy (adjustLink may change them later)
    uint8_t codedSchemePref = 0;
    if (s.desiredPhyMask == BLE_GAP_LE_PHY_CODED_MASK && s.desiredCodedScheme) {
      codedSchemePref = (s.desiredCodedScheme == 2 ? BLE_GAP_LE_PHY_CODED_S2 : BLE_GAP_LE_PHY_CODED_S8);
    }

    (void)ble_gap_set_prefered_le_phy(s.connHandle, s.desiredPhyMask, s.desiredPhyMask, codedSchemePref);

    // Learn actual negotiated PHY
    uint8_t txPhy = 0, rxPhy = 0;
    if (ble_gap_read_le_phy(s.connHandle, &txPhy, &rxPhy) == 0) {
      s.phyIs2M     = (txPhy == BLE_HCI_LE_PHY_2M) && (rxPhy == BLE_HCI_LE_PHY_2M);
      s.phyIsCoded  = (txPhy == BLE_HCI_LE_PHY_CODED) && (rxPhy == BLE_HCI_LE_PHY_CODED);
      s.codedScheme = s.phyIsCoded ? (s.desiredCodedScheme ? s.desiredCodedScheme : 8) : 0; // best guess until event
    } else {
      // PHY read failed: fall back to 1M assumptions
      s.phyIs2M     = false;
      s.phyIsCoded  = false;
      s.codedScheme = 0;
    }

    s.desiredllTxOctets = LL_MAX_OCTETS;
    s.desiredllTxTimeUs = s.estimate_LL_PDUTimeUs(s.desiredllTxOctets, s.phyIs2M, s.phyIsCoded, s.codedScheme); // best guess, likely too short

    (void)ble_gap_set_data_len(s.connHandle, s.desiredllTxOctets, s.desiredllTxTimeUs);

    s.llTxOctets = s.desiredllTxOctets; // for now
    s.llTxTimeUs = s.desiredllTxTimeUs; // for now

    s.updateTxTiming();
    // computes txChunkSize
    // minSendIntervalUS
    // low & high Water
    // lkgIntervalUs
    // sendIntervalUs will be >= minSendIntervalUS
    // probing/backoff state reset    

    // Start security if enabled
    if (s.secure != Security::None) NimBLEDevice::startSecurity(s.connHandle);

    // Resume Tasks
    #ifdef ARDUINO_ARCH_ESP32
      s.wakeTxTask();
      s.wakeRSSITask();
    #endif

    if (s.logLevel >= INFO) {
      Serial.printf(
        "BLESerial: Connected %s PHY=%s.\r\n"
        "           chunk=%u send_interval=%uµs min_send_interval=%uµs.\r\n"
        "           llTx: octets=%u time=%uµs | llRx: octets=%u time=%uµs.\r\n",
        s.peerAddr.c_str(),
        s.phyToStr(),
        (unsigned)s.txChunkSize, (unsigned)s.sendIntervalUs,(unsigned)s.minSendIntervalUS,
        (unsigned)s.llTxOctets,(unsigned)s.llTxTimeUs,
        (unsigned)s.llRxOctets,(unsigned)s.llRxTimeUs
      );
    }
    if (s.onClientConnect)
      s.onClientConnect(s.peerAddr);
  }

  void onDisconnect(NimBLEServer *srv, NimBLEConnInfo &connInfo, int reason) override
  {
    if (!owner) return;
    auto &s = *owner;

    // Link down
    s.deviceConnected   = false;
    s.clientSubscribed  = false;
    s.connHandle        = BLE_HS_CONN_HANDLE_NONE;
    s.linkEncrypted     = false;

    // Reset PHY state and conservative LL timing defaults
    s.phyIs2M           = false;
    s.phyIsCoded        = false;
    s.codedScheme       = 0;
    s.llTxTimeUs        = LL_DEFAULT_TIME_US;
    s.llTxOctets        = LL_MAX_OCTETS; // propose max octets next time (controller may downscale)
    s.llRxTimeUs        = LL_DEFAULT_TIME_US;
    s.llRxOctets        = LL_MAX_OCTETS; // propose max octets next time (controller may downscale)

    // Reset pacing/backoff/probing state
    s.probing           = false;
    s.probeSuccesses    = 0;
    s.probeFailures     = 0;
    s.lkgFailStreak     = 0;
    s.recentlyBackedOff = false;
    s.cooldownSuccess   = 0;
    s.successStreak     = 0;
    s.lastEscalateAtUs  = 0;
    s.eappRetries       = 0;

    // Reset MTU/chunk/EBADDATA counters; drop any staged frame
    s.mtuRetryCount     = 0;
    TX_CRITICAL_ENTER(&s);
    s.txOk              = false;
    s.pendingLen        = 0;
    s.txPending         = false;
    s.lastTxUs          = 0;
    TX_CRITICAL_EXIT(&s);

    s.updateTxTiming();
    // computes txChunkSize
    // minSendIntervalUS
    // low & high Water
    // lkgIntervalUs
    // sendIntervalUs will be >= minSendIntervalUS
    // probing/backoff state reset    

    // Restart advertising
    if (s.advertising) {
      s.advertising->start();
    } else {
      NimBLEDevice::startAdvertising();
    }

    #ifdef ARDUINO_ARCH_ESP32
      s.suspendTxTask();
      s.suspendRSSITask();
    #endif

    if (s.logLevel >= INFO) {
      const uint8_t hci = static_cast<uint8_t>(reason & 0xFF);
      Serial.printf(
        "BLESerial: Client [%s] disconnected "
        "(reason=%u %s). Advertising restarted.\r\n",
        connInfo.getAddress().toString().c_str(),
        hci, hciDisconnectReasonStr(hci));
    }
    if (s.onClientDisconnect) {
      const uint8_t hci = static_cast<uint8_t>(reason & 0xFF);
      s.onClientDisconnect(connInfo.getAddress().toString(), hci);
    }
  }

  void onMTUChange(uint16_t m, NimBLEConnInfo &connInfo) override {
    if (!owner) return;
    auto &s = *owner;

    // Update negotiated MTU
    s.mtu = m;

    s.updateTxTiming(); // also clamps sendIntervalUs and resets ramp to floor
    // computes txChunkSize
    // minSendIntervalUS
    // low & high Water
    // lkgIntervalUs
    // sendIntervalUs will be >= minSendIntervalUS
    // probing/backoff state reset    

    // Update attribute max lengths to reflect negotiated MTU (clamped to 512 per spec)
    // NOT AVAILABLE IN THIS VERSION OF NIMBLE (left for future)
    // uint16_t attMax = (m > 3) ? (uint16_t)std::min<uint16_t>(BLE_SERIAL_MAX_GATT, (uint16_t)(m - 3)) : (uint16_t)20;
    // if (s.rxChar) s.rxChar->setMaxLen(attMax);
    // if (s.txChar) s.txChar->setMaxLen(attMax);

    if (s.logLevel >= INFO) {
      Serial.printf(
        "BLESerial: MTU=%u (conn=%u), " 
        "tx_chunk_size=%u, min_send_interval=%uµs.\r\n",
        m, connInfo.getConnHandle(), 
        s.txChunkSize, (unsigned)s.minSendIntervalUS);
    }
    if (s.onMtuChanged)
      s.onMtuChanged(m);
  }

  // NOT AVAILABLE IN THIS VERSION OF NIMBLE (left for future)

  // // Generate and return a random 6-digit passkey (000000–999999)
  // uint32_t onPassKeyRequest() override {
  //   if (!owner) return 0;
  //   auto& s = *owner;

  //   // Generate random 6-digit code; ensure leading zeros possible on display
  //   uint32_t key = (uint32_t)random(0UL, 1000000UL);
  //   s.passkey = key;

  //   if (s.logLevel >= INFO) {
  //       Serial.printf("BLESerial: Server Passkey Request: %06u\r\n", key);
  //   }
  //   return key;
  // }

  // Display callback (called to present the passkey to the user)
  uint32_t onPassKeyDisplay() override {
    if (!owner) return 0;
    auto &s = *owner;

    if (s.logLevel >= INFO) {
      Serial.printf(
        "BLESerial: Server Passkey: %06u.\r\n", s.passkey);
    }
    return s.passkey;
  }

  // Confirm the passkey shown/entered by the peer
  void onConfirmPassKey(NimBLEConnInfo &connInfo, uint32_t peerKey) override {
    if (!owner) return;
    auto &s = *owner;

    bool match = (peerKey == s.passkey);
    NimBLEDevice::injectConfirmPasskey(connInfo, match);

    if (s.logLevel >= INFO) {
      Serial.printf(
        "BLESerial: Confirm Passkey: "
        "local=%06u peer=%06u %s.\r\n",
        s.passkey, peerKey, 
        match ? "MATCH" : "MISMATCH");
    }
  }

  void onAuthenticationComplete(NimBLEConnInfo &connInfo) override {
    if (!owner) return;
    auto &s = *owner;

    if (!connInfo.isEncrypted()) {
      s.linkEncrypted = false;
      NimBLEDevice::getServer()->disconnect(connInfo.getConnHandle());
      if (s.logLevel >= WARNING) {
        Serial.println(
          "BLESerial: Encrypt connection failed - disconnecting client.");
      }
      return;
    }

    s.linkEncrypted = true;

    if (s.logLevel >= INFO) {
      Serial.printf(
        "BLESerial: Secured connection to: %s.\r\n", 
        connInfo.getAddress().toString().c_str());
    }
  }

  void onConnParamsUpdate(NimBLEConnInfo &connInfo) override {
    if (!owner)return;
    if (owner->getLogLevel() >= INFO) {
      auto itvl = connInfo.getConnInterval();
      auto lat  = connInfo.getConnLatency();
      auto sup  = connInfo.getConnTimeout();
      Serial.printf(
        "BLESerial: Connection parameters updated:\r\n"
        "           interval=%.2fms, latency=%u, timeout=%ums.\r\n",
        itvl * 1.25f, lat, sup * 10u);
    }
  }

  void onPhyUpdate(NimBLEConnInfo &connInfo, uint8_t txPhy, uint8_t rxPhy) override {
    if (!owner) return;
    // Duplicate PHY logging suppressed; GAP event handler provides authoritative update.
    // (Intentionally left silent to avoid log spam.)
    (void)txPhy; (void)rxPhy; (void)connInfo;
  }

  void onIdentity(NimBLEConnInfo &connInfo) override {
    if (!owner) return;
    if (owner->getLogLevel() >= INFO) {
      Serial.printf(
        "BLESerial: Identity resolved: %s.\r\n",
        connInfo.getAddress().toString().c_str());
    }
  }

private:
  BLESerial *owner{nullptr};

  static const char *hciDisconnectReasonStr(uint8_t r) {
    switch (r) {
      case 0x08: return "Connection Timeout";
      case 0x10: return "Remote User Terminated";
      case 0x13: return "Remote User Terminated"; // 0x13 (same meaning)
      case 0x16: return "Connection Terminated by Local Host";
      case 0x3B: return "Unacceptable Connection Parameters";
      case 0x3D: return "MIC Failure";
      case 0x3E: return "Connection Failed to be Established";
      default:   return "Unknown";
      }
  }
}; // end of ServerCallbacks ====================================================================

// ===== RxCallbacks: handles incoming data =====================================================

class BLESerial::RxCallbacks : public NimBLECharacteristicCallbacks
{
public:
  explicit RxCallbacks(BLESerial *owner) : owner(owner) {}

  // Received data from client
  void onWrite(NimBLECharacteristic *ch, NimBLEConnInfo &connInfo) override {
    if (!owner) return;
    BLESerial &s = *owner;

    const std::string &v = ch->getValue();
    if (v.empty()) return;

    // Push into RX ring; overwrite oldest to avoid blocking the NimBLE task.
    const uint8_t *data = reinterpret_cast<const uint8_t *>(v.data());
    const size_t len = v.size();

    // RingBuffer is internally synchronized on ESP32; no external critical section needed here.
    size_t pushed = s.rxBuf.push(data, len, true); // overwrite oldest if full

    // RX accounting (optional but handy)
    s.bytesRx += pushed;
    if (pushed < len) {
      size_t lost = len - pushed;
      s.rxDrops += lost;
      if (s.onRxOverflow)
        s.onRxOverflow(lost);
    }
    s.lastRxUs = micros();

    // Clear last value held by the characteristic to free heap.
    ch->setValue(nullptr, 0);

    if (s.onDataReceived && pushed)
      s.onDataReceived(data, pushed);
  }

private:
  BLESerial *owner;

}; // end of RxCallbacks ========================================================================

// ===== TxCallbacks: handles notification status ===============================================

class BLESerial::TxCallbacks : public NimBLECharacteristicCallbacks
{
public:
  explicit TxCallbacks(BLESerial *owner) : owner(owner) {}

  // After notification, success and error handling
  void onStatus(NimBLECharacteristic *ch, int code) override {
    /*

    ==========================================
    I will need to work on this. ChatGPT 5.1 says:

    BLE_HS_NOMEM payload did not enter controller mbuf queue: need to resend preferably with backoff
    BLE_HS_EBUSY payload did not enter controller mbuf queue: need to resend with delay
    BLE_HS_EAPP application error: fully recoverable need to resend, fix application call back issue, software problem
    BLE_HS_EMSGSIZE payload too large for controller: need to reduce size and resend,
    resend = setValue then notify, but notify by itself is sufficient 
    If there is error no partial data was sent.

    GOAL is to reprogram this section and handle it with these rules.
    Normal path
      setValue(chunk)
      notify()
    On success (status 0 / BLE_HS_EDONE for indications) → advance ring buffer.
     
    Errors that allow simple retry (same chunk size)
      BLE_HS_ENOMEM – out of mbufs / buffers.
      BLE_HS_EBUSY – some other procedure in progress.
      BLE_HS_EAPP – app callback misbehaved (once you’re confident you fixed the bug).
      → Just notify again later with the same data. Value is still present; no partial.

    Error that requires changing the message
      BLE_HS_EMSGSIZE – payload too large for context (MTU / data len / buffer). 
      → Reduce chunkSize and resend (you can split into two smaller chunks that each fit).

    ==========================================

      Status codes:
      0                       → Success (notification queued/sent).
      14 (BLE_HS_EDONE)       → Success for indication (confirmation received).
      1  (BLE_HS_EAGAIN)      → Operation failed and should be retried later.
      2  (BLE_HS_EALREADY)    → Operation already in progress.
      3  (BLE_HS_EINVAL)      → Invalid parameters.
      4  (BLE_HS_EMSGSIZE)    → Payload too big for context. (For notifies you should already be ≤ MTU−3.)
      5  (BLE_HS_ENOENT)      → No such entry.
      6  (BLE_HS_ENOMEM)      → Out of buffers / resource exhaustion. You’re sending faster than the stack can drain, or mbufs are tight. Back off or throttle.
      7  (BLE_HS_ENOTCONN)    → Connection went away / bad handle.
      8  (BLE_HS_ENOTSUP)     → Not supported.
      9  (BLE_HS_EAPP)        → Application error.
      10 (BLE_HS_EBADDATA)    → Malformed data.
      11 (BLE_HS_EOS)         → Operating system error.
      12 (BLE_HS_ECONTROLLER) → Controller error.
      13 (BLE_HS_ETIMEOUT)    → Operation timed out.
      15 (BLE_HS_EBUSY)       → Another LL/GATT procedure is in progress; try again later.
      16 (BLE_HS_EREJECT)     → Operation rejected.
      17 (BLE_HS_EUNKNOWN)    → Unknown error.
      18 (BLE_HS_EROLE)       → Role error.
      19 (BLE_HS_ETIMEOUT_HCI)→ HCI timeout.
      20 (BLE_HS_ENOMEM_EVT)  → Out of memory to handle event.
      21 (BLE_HS_ENOADDR)     → No valid address.
      22 (BLE_HS_ENOTSYNCED)  → Host not synced with controller yet.
      23 (BLE_HS_EAUTHEN)     → Authentication failed.
      24 (BLE_HS_EAUTHOR)     → Authorization failed.
      25 (BLE_HS_EENCRYPT)    → Encryption failed.
      26 (BLE_HS_EENCRYPT_KEY_SZ) → Encryption key size insufficient.
      27 (BLE_HS_ESTORE_CAP)  → Storage capacity exceeded.
      28 (BLE_HS_ESTORE_FAIL) → Storage operation failed.
      29 (BLE_HS_EPREEMPTED)  → Operation preempted.
      30 (BLE_HS_EDISABLED)   → Feature disabled.
      31 (BLE_HS_ESTALLED)    → Operation stalled.

      // ========================================

      BLE_HS_ENOMEM – “Operation failed due to resource exhaustion.”
      BLE_HS_ENOMEM_EVT – “Controller failed to send event due to memory exhaustion.”
      BLE_HS_ECONTROLLER – “Event from controller is invalid.”
      BLE_HS_EAPP – “Application callback behaved unexpectedly.”
      BLE_HS_EBADDATA – “Command from peer is invalid.”
      BLE_HS_EAGAIN – “Temporary failure; try again.”
      BLE_HS_EALREADY – “Operation already in progress or completed.”
      BLE_HS_ETIMEOUT – “Operation timed out.”
      BLE_HS_ETIMEOUT_HCI – “HCI request timed out; controller unresponsive.”
      BLE_HS_EBUSY – “Operation cannot be performed until procedure completes.”
      BLE_HS_ENOTCONN – “No open connection with the specified handle.”
      BLE_HS_EOS – “Mynewt OS error.”
      plus the others like EUNKNOWN, EINVAL, etc.

      // === Success ===
      static inline bool isOkOrDone(int code) {
        return (code == 0 || code == BLE_HS_EDONE);
      }

      // === Payload-related ===
      // -> modify payload before retrying.
      static inline bool isMsgSize(int code) {
        // Buffer too small / chunk too big.
        return (code == BLE_HS_EMSGSIZE);
      }

      // === Retryable "backpressure" / congestion ===
      // -> retry same payload later with backoff.
      // temporary resource exhaustion
      static inline bool isRetryableCongestion(int code) {
        return (code == BLE_HS_EAGAIN    ||  // temporary; try again
                code == BLE_HS_EALREADY  ||  // other proc already in progress
                code == BLE_HS_EBUSY     ||  // cannot perform until proc completes
                code == BLE_HS_ENOMEM    ||  // host out of mbufs
                code == BLE_HS_ENOMEM_EVT);  // controller couldn’t send event
      }

      // === Timeouts (maybe retry a few times, then treat as fatal) ===
      // -> retry a few times, then treat as fatal
      static inline bool isTimeoutMaybeRetryable(int code) {
        return (code == BLE_HS_ETIMEOUT ||
                code == BLE_HS_ETIMEOUT_HCI);
      }

      // === Link gone / OS-level fatal ===
      // -> do not retry
      // teardown connection, clean up
      static inline bool isDisconnectedOrEOS(int code) {
        return (code == BLE_HS_ENOTCONN ||  // link closed
                code == BLE_HS_EOS);       // Mynewt OS error
      }

      // === Local bug / invalid state / non-retryable ===
      // -> do not retry
      // software bug / invalid state
      static inline bool isLocalBugOrBadState(int code) {
        return (code == BLE_HS_EINVAL      ||  // bad args / state
                code == BLE_HS_EAPP        ||  // your callback misbehaved
                code == BLE_HS_EBADDATA    ||  // peer command invalid (protocol bad)
                code == BLE_HS_ECONTROLLER ||  // controller event invalid
                code == BLE_HS_EUNKNOWN);      // catch-all unexpected
      }



    */

    if (!owner) return;
    auto &s = *owner;

    // Check for stale staged frame (e.g., due to renegotiated MTU)
    const bool staleStaged = (s.txPending && s.pendingLen && (s.pendingChangeEpoch != s.chunkChangeEpoch));
    
    // Success path: OK or EDONE -------------------------------------------------------
    if (isOkOrDone(code)) {
      TX_CRITICAL_ENTER(&s);
      s.bytesTx      += s.pendingLen;
      s.pendingLen    = 0;
      s.txPending     = false;
      s.txOk          = true;
      s.txSuccessCount++;
      s.mtuRetryCount = 0;
      // After commit, ensure low-water unlock
      if (s.txBuf.available() <= s.lowWater) s.txLocked = false;
      TX_CRITICAL_EXIT(&s);

      // Cooldown after a backoff before probing again
      if (s.recentlyBackedOff) {
        if (++s.cooldownSuccess >= COOL_SUCCESS_REQUIRED) {
          s.recentlyBackedOff = false;
          s.cooldownSuccess   = 0;
          s.successStreak     = 0;
          s.lkgFailStreak     = 0;
        }
        return;
      }

      // Probe success handling
      if (s.probing)
      {
        if (++s.probeSuccesses >= PROBE_CONFIRM_SUCCESSES)
        {
          s.lkgIntervalUs    = s.sendIntervalUs; // accept new floor
          s.probing          = false;
          s.probeSuccesses   = 0;
          s.probeFailures    = 0;
          s.lkgFailStreak    = 0;
          s.successStreak    = 0;
          if (s.logLevel >= INFO)
            Serial.printf(
              "BLESerial: Probe accepted. LKG=%u.\r\n", 
              s.lkgIntervalUs);
        }
        return;
      }

      s.lkgFailStreak = 0; // Not probing: clear fail streak and maybe start a probe later
      s.eappRetries   = 0; // Reset EAPP retry counter on any success so future EAPPs get fresh retries

      if (s.successStreak >= PROBE_AFTER_SUCCESSES) {
        // We have enough successes to store a probe and update last known good interval value
        s.successStreak = 0;
        if (s.sendIntervalUs <= s.minSendIntervalUS) 
          return; // already at floor, do not lower interval further
        s.lkgIntervalUs = s.sendIntervalUs;
        // Either lower interval by 10 microseconds or by 2%. Add small jitter
        //   to avoid synchronized probes with peers.
        uint32_t stepAbs = PROBE_STEP_US;
        uint32_t stepPct = (s.sendIntervalUs * PROBE_STEP_PCT) / 100u;
        uint32_t baseStep = (stepPct > stepAbs) ? stepPct : stepAbs;
        // Jitter: +/- up to 25% of baseStep; to avoid synching with client who might be probing also
        uint32_t jitter = (baseStep / 4);
        uint32_t rnd = (uint32_t)random(0, (int)(jitter * 2 + 1));
        uint32_t step = (rnd > jitter) ? (baseStep + (rnd - jitter)) : (baseStep - rnd);
        if (s.sendIntervalUs > step) {
          s.sendIntervalUs -= step;
          if (s.sendIntervalUs < s.minSendIntervalUS)
            s.sendIntervalUs = s.minSendIntervalUS;
          s.probing        = true;
          s.probeSuccesses = 0;
          s.probeFailures  = 0;
          if (s.logLevel >= INFO)
            Serial.printf(
              "BLESerial: Starting probe: %u -> %u.\r\n", 
              s.lkgIntervalUs, s.sendIntervalUs);
        } else {
          return; // can't go lower than 0
        }
      }
      s.successStreak++;
      return;
    } // end of success path

    // EMSGSIZE: payload too big for context --------------------------------------------------------
    //   — adjust chunking for future frames and drop staged frame
    if (isMsgSize(code)) {
      
      if (staleStaged) {
        // Renegotiation shrank the allowed frame size; drop the stale staged frame quietly.
        s.dropStagedAccounting(false /*txDrops*/, false /*unlock*/); // txDrops ,unlock 
        if (s.logLevel >= WARNING) {
          Serial.println("BLESerial: EMSGSIZE on stale frame after renegotiation; dropping staged and continuing.");
        }
        return;
      }

      s.msgSizeCount++; // count only when staleStaged is false

      // Reduce chunk size and recompute pacing floor; 
      //   bump epoch so future frames restage under new limits
      uint16_t prevChunk = s.txChunkSize;
      if (++s.mtuRetryCount <= MTU_RETRY_MAX) {
        s.txChunkSize = (uint16_t)std::max(20, (int)s.txChunkSize / 2);
      } else {
        if (s.txChunkSize > 20) {
          s.txChunkSize   = 20;
          s.mtuRetryCount = 0;
        } else {
          if (s.logLevel >= WARNING)
            Serial.printf("BLESerial: %s: persistent -> disconnect.\r\n", codeName(code));
          if (s.server && s.connHandle != BLE_HS_CONN_HANDLE_NONE)
            s.server->disconnect(s.connHandle);
          s.mtuRetryCount = 0;
        }
      }
      if (s.txChunkSize != prevChunk) s.chunkChangeEpoch++;
      s.updateWaterMarks(s.txChunkSize);
      s.minSendIntervalUS = s.computeSendIntervalUs(s.txChunkSize);
      s.sendIntervalUs    = s.minSendIntervalUS;
      if (s.logLevel >= WARNING) {
        Serial.printf(
          "BLESerial: %s adjust chunk %u->%u min_send_interval=%uµs (retry %d/%d).\r\n",
          codeName(code), prevChunk, s.txChunkSize, s.minSendIntervalUS, s.mtuRetryCount, MTU_RETRY_MAX);
      }

      // Drop the currently staged frame (cannot be re-segmented post-pop)
      // Consider unlocking producers if usage is already <= lowWater
      s.dropStagedAccounting(true /*txDrops*/, true /*unlock*/);
      return;
    } // end of EMSGSIZE handling

    // EBADDATA (malformed data): --------------------------------------------------------------- 
    //  cannot recover this staged frame; drop it.
    if (isBadData(code)) {
      s.badDataCount++; // count always
      if (s.logLevel >= WARNING) {
        Serial.printf("BLESerial: %s: dropping malformed frame.\r\n", codeName(code));
      }
      if (staleStaged) {
        // Treat as renegotiation-induced reject for a stale frame; drop quietly (no txDrops).
        s.dropStagedAccounting(false /*txDrops*/, false /*unlock*/);
        return;
      }
      // Account loss for current staged frame; consider unlock if already below lowWater
      s.dropStagedAccounting(true /*txDrops*/, true /*unlock*/);
      return;
    } // Bad Data handling

    // EAPP (application error): ---------------------------------------------------------------
    //   typically send interval too tight; backoff/cooldown
    if (isAppError(code)) {
      // Application error typically means send interval was too tight.
      // Pop model: we cannot restage the same popped bytes; adopt drop-on-failure policy.

      if (staleStaged) {
        // Stale staged under old limits; drop quietly and continue.
        s.dropStagedAccounting(false /*txDrops*/, false /*unlock*/);
        if (s.logLevel >= WARNING) {
          Serial.println("BLESerial: EAPP on stale frame after renegotiation; dropping staged and continuing.");
        }
        return;
      }

      s.eappCount++; // count only when staleStaged is false

      // Backoff/cooldown bookkeeping
      s.successStreak     = 0;
      s.recentlyBackedOff = true;
      s.cooldownSuccess   = 0;

      if (s.probing) {
        s.probeFailures++;
        s.probing        = false;
        s.sendIntervalUs = s.lkgIntervalUs; // revert to floor
        // On a slower interval, reset lastTxUs so pacing waits properly.
        if (s.sendIntervalUs >= s.lkgIntervalUs)
          s.lastTxUs = (uint32_t)micros();
        s.lkgFailStreak  = 0;
        if (s.logLevel >= INFO)
          Serial.printf(
            "BLESerial: %s failing probe, revert to LKG=%u.\r\n",
            codeName(code), s.lkgIntervalUs);
      } else {
        // Not probing: escalate LKG after repeated failures
        if (++s.lkgFailStreak >= LKG_ESCALATE_AFTER_FAILS) {
          if (s.escalateLKG(true /*require cooldown*/)) {
            s.lkgFailStreak = 0;
            if (s.logLevel >= INFO)
              Serial.printf(
                "BLESerial: %s escalate LKG to %u, txBuf=%u lowWater=%u min_send_interval=%uµs.\r\n",
                codeName(code), s.lkgIntervalUs,
                (unsigned)s.txBuf.available(), (unsigned)s.lowWater, (unsigned)s.minSendIntervalUS);
          }
        }
      }

      // Drop the staged frame (cannot safely retry in pop model without a resend path)
      // Consider unlocking producers if usage is already <= lowWater
      s.dropStagedAccounting(true /*txDrops*/, true /*unlock*/);
      if (s.logLevel >= WARNING)
        Serial.printf("BLESerial: %s: dropped staged frame and backed off.\r\n", codeName(code));
      return;
    } // End Application Error handling

    // Out of resources ----------------------------------------------------------
    //   not recoverable for the current staged frame in pop model
    if (isOutOfResources(code)) {
      s.enomemCount++; // count always
      s.lastEnomemAtUs = (uint32_t)micros();

      // Drop and account the staged frame; cannot safely retry without a resend path
      s.dropStagedAccounting(true /*txDrops*/, true /*unlock*/);

      // Strong backoff: unconditional escalation
      s.escalateLKG(false /*do not require cooldown*/);

      s.successStreak     = 0;
      s.recentlyBackedOff = true;
      s.cooldownSuccess   = 0;

      if (s.logLevel >= WARNING) {
        Serial.printf(
          "BLESerial: %s: dropped staged frame, escalated to %uµs, txBuf=%u lowWater=%u, enomemCount=%u.\r\n",
          codeName(code), s.sendIntervalUs,
          (unsigned)s.txBuf.available(), (unsigned)s.lowWater, (unsigned)s.enomemCount);
      }
      return;
    } // end of out-of-resources handling

    // Congestion/timeouts/busy ----------------------------------------------------------
    if (isCongestionRecoverable(code)) {
      s.congestionCount++; // count always
      if (code == BLE_HS_ESTALLED) s.estalledCount++;
      s.successStreak     = 0;
      s.recentlyBackedOff = true;
      s.cooldownSuccess   = 0;

      // If this frame is stale due to renegotiation, drop it quietly and return
      if (staleStaged) {
        s.dropStagedAccounting(false /*txDrops*/, false /*unlock*/);
        if (s.logLevel >= WARNING)
          Serial.println("BLESerial: congestion on stale frame; dropping staged and continuing.");
        return;
      }

      if (s.probing) {
        s.probeFailures++;
        s.probing           = false;
        s.sendIntervalUs    = s.lkgIntervalUs;
        s.lkgFailStreak     = 0;
        if (s.logLevel >= WARNING)
          Serial.printf(
            "BLESerial: %s: failing probe, revert to LKG=%u.\r\n",
            codeName(code), s.sendIntervalUs);
      } else {
        if (++s.lkgFailStreak >= LKG_ESCALATE_AFTER_FAILS) {
          if (s.escalateLKG(true /*require cooldown*/)) {
            s.lkgFailStreak = 0;
            if (s.logLevel >= WARNING)
              Serial.printf(
                "BLESerial: %s: escalate LKG to %uµs txBuf=%u lowWater=%u min_send_interval=%uµs.\r\n",
                codeName(code), s.sendIntervalUs, (unsigned)s.txBuf.available(),
                (unsigned)s.lowWater, (unsigned)s.minSendIntervalUS);
          }
        }
      }
      // Drop the staged frame for pop model; cannot safely retry without resend path
      s.dropStagedAccounting(true /*txDrops*/, true /*unlock*/);
      return;
    } // end of congestion handling

    // Disconnect or EOS-like ------------------------------------------------------------
    if (isDisconnectedOrEOS(code)) {
      s.disconnectCount++; // count always
      s.successStreak     = 0;
      s.recentlyBackedOff = false;
      s.cooldownSuccess   = 0;
      s.probing           = false;
      s.probeSuccesses    = 0;
      s.probeFailures     = 0;
      s.lkgFailStreak     = 0;
      s.sendIntervalUs    = 0;
      s.minSendIntervalUS = 0;
      s.lkgIntervalUs     = s.sendIntervalUs;
      // Clear any staged frame left from pop-based stage to avoid stuck state
      TX_CRITICAL_ENTER(&s);
      s.pendingLen        = 0;
      s.txPending         = false;isCongestionRecoverab
      s.txOk              = false;
      s.txLocked          = false; // unlock producers; link is down anyway
      TX_CRITICAL_EXIT(&s);
      if (s.logLevel >= WARNING)
        Serial.printf(
          "BLESerial: %s: link closed (ENOTCONN/EOS).\r\n", 
          codeName(code));
      return;
    } // end of disconnect/EOS

    // Unclassified: ---------------------------------------------------------------------
    //   drop probe if probing; otherwise no pacing change
    s.unclassifiedCount++; // count always
    // Always clear any staged frame to avoid lingering txPending/pendingLen.
    // Treat as loss unless the frame is stale due to renegotiation.
    if (staleStaged) {
      s.dropStagedAccounting(false /*txDrops*/, true /*considerUnlock*/);
    } else {
      s.dropStagedAccounting(true /*txDrops*/, true /*considerUnlock*/);
    }
    if (s.probing){
      s.probing           = false;
      s.sendIntervalUs    = s.lkgIntervalUs;
      s.lkgFailStreak     = 0;
      if (s.logLevel >= WARNING)
        Serial.printf(
          "BLESerial: %s: unclassified issue while probing: revert to LKG=%u.\r\n",
          codeName(code), s.sendIntervalUs);
    } else {
      if (s.logLevel >= WARNING)
        Serial.printf(
          "BLESerial: %s: unclassified issue.\r\n",
          codeName(code));
    } // end of unclassified
  }
  // end of onStatus ---------------------------------------------------------------------------------

  void onSubscribe(NimBLECharacteristic *ch, NimBLEConnInfo &connInfo, uint16_t subValue) override {
    if (!owner) return;
    auto &s = *owner;

    bool notify        = (subValue & 0x0001);
    bool indicate      = (subValue & 0x0002);
    s.clientSubscribed = notify || indicate;

    if (s.logLevel >= INFO) {
      std::string uuid = ch->getUUID().toString();
      if (subValue == 0)
        Serial.printf(
          "BLESerial: Client %s unsubscribed %s.\r\n", 
          s.peerAddr.c_str(), uuid.c_str());
      else
        Serial.printf(
          "BLESerial: Client %s subscribed (%s%s) %s.\r\n",
          s.peerAddr.c_str(),
          notify ? "notify" : "",
          indicate ? (notify ? "+indicate" : "indicate") : "",
          uuid.c_str());
    }
    if (s.onSubscribeChanged)
      s.onSubscribeChanged(s.clientSubscribed);
  }
  // end of onSubscribe -------------------------------------------------------------------------------------

private:
  BLESerial *owner;

  // ---- Status code normalization helpers ----
  static inline bool isOkOrDone(int code) {
    return (code == 0 || code == BLE_HS_EDONE);
  }

  static inline bool isMsgSize(int code) {
    return (code == BLE_HS_EMSGSIZE);
  }

  static inline bool isBadData(int code) {
    return (code == BLE_HS_EBADDATA);
  }

  static inline bool isAppError(int code) {
    return (code == BLE_HS_EAPP);
  }

  static inline bool isOutOfResources(int code) {
    return (code == BLE_HS_ENOMEM ||
            code == BLE_HS_ENOMEM_EVT ||
            code == BLE_HS_ECONTROLLER);
  }

  static inline bool isCongestionRecoverable(int code) {
    return (code == BLE_HS_EAGAIN ||
            code == BLE_HS_EALREADY ||
            code == BLE_HS_EBUSY ||
            code == BLE_HS_ESTALLED ||
            code == BLE_HS_EPREEMPTED ||
            code == BLE_HS_ETIMEOUT ||
            code == BLE_HS_ETIMEOUT_HCI);
  }

  static inline bool isDisconnectedOrEOS(int code) {
    return (code == BLE_HS_ENOTCONN ||
            code == BLE_HS_EOS);
  }

  static const char *codeName(int code) {
    switch (code) {
      case 0:      return "OK(0)"; // notify success
      case 1:      return "EAGAIN(1)"; // retry later
      case 2:      return "EALREADY(2)"; // op in progress
      case 3:      return "EINVAL(3)";
      case 4:      return "EMSGSIZE(4)";
      case 5:      return "ENOENT(5)";
      case 6:      return "ENOMEM(6)";
      case 7:      return "ENOTCONN(7)";
      case 8:      return "ENOTSUP(8)";
      case 9:      return "EAPP(9)";
      case 10:     return "EBADDATA(10)";
      case 11:     return "EOS(11)";
      case 12:     return "ECONTROLLER(12)";
      case 13:     return "ETIMEOUT(13)";
      case 14:     return "EDONE(14)"; // indicate success
      case 15:     return "EBUSY(15)";
      case 16:     return "EREJECT(16)";
      case 17:     return "EUNKNOWN(17)";
      case 18:     return "EROLE(18)";
      case 19:     return "ETIMEOUT_HCI(19)";
      case 20:     return "ENOMEM_EVT(20)";
      case 21:     return "ENOADDR(21)";
      case 22:     return "ENOTSYNCED(22)";
      case 23:     return "EAUTHEN(23)";
      case 24:     return "EAUTHOR(24)";
      case 25:     return "EENCRYPT(25)";
      case 26:     return "EENCRYPT_KEY_SZ(26)";
      case 27:     return "ESTORE_CAP(27)";
      case 28:     return "ESTORE_FAIL(28)";
      case 29:     return "EPREEMPTED(29)";
      case 30:     return "EDISABLED(30)";
      case 31:     return "ESTALLED(31)";
      default:     return nullptr; // not a core code
    }
  } // end of codeName

}; // end of TxCallbacks ========================================================================

// ==============================================================================================
// ==============================================================================================

// ===== Additional GAP event handler ===============================================================

int BLESerial::gapEventHandler(struct ble_gap_event *ev, void * /*arg*/) {
  // Obtain BLESerial instance
  if (!ev) return 0;
  BLESerial *inst = BLESerial::active;

  if (!inst) return 0;
  BLESerial &s = *inst;

  switch (ev->type) {
    case BLE_GAP_EVENT_PHY_UPDATE_COMPLETE:
    {
      const auto &p = ev->phy_updated;

      if (p.status != 0) {
        // error
        s.phyIs2M     = false;
        s.phyIsCoded  = false;
        s.codedScheme = 0;
        s.updateTxTiming();
        // computes txChunkSize
        // minSendIntervalUS
        // low & high Water
        // lkgIntervalUs
        // sendIntervalUs
        // probing/backoff state reset      
        return 0;
      }

      bool prev2M = s.phyIs2M;
      bool prevCoded = s.phyIsCoded;
      uint8_t prevScheme = s.codedScheme;

      s.phyIs2M = (p.tx_phy == BLE_HCI_LE_PHY_2M) && (p.rx_phy == BLE_HCI_LE_PHY_2M);
      s.phyIsCoded = (p.tx_phy == BLE_HCI_LE_PHY_CODED) && (p.rx_phy == BLE_HCI_LE_PHY_CODED);
      s.codedScheme = s.phyIsCoded ? (s.desiredCodedScheme == 2 ? 2 : 8) : 0;

      // Early return if PHY/coding unchanged (suppress redundant recompute/log)
      if (prev2M == s.phyIs2M && prevCoded == s.phyIsCoded && prevScheme == s.codedScheme) {
        if (s.logLevel >= DEBUG) {
          Serial.printf(
            "BLESerial: %s: \r\n"
            "           tx=%u, rx=%u, (%s), llTxTime=%uµs, tx_chunk_size=%u, min_send_interval=%uµs.\r\n",
            "PHY unchanged (GAP)",
            p.tx_phy, p.rx_phy, s.phyToStr(),
            (unsigned)s.llTxTimeUs,
            (unsigned)s.txChunkSize,
            (unsigned)s.minSendIntervalUS
          );
        }
        return 0;
      }

      s.updateTxTiming();
      // computes txChunkSize
      // minSendIntervalUS
      // low & high Water
      // lkgIntervalUs
      // sendIntervalUs
      // probing/backoff state reset

      if (s.logLevel >= INFO) {
        Serial.printf(
          "BLESerial: %s: \r\n"
          "           tx=%u, rx=%u, (%s), llTxTime=%uµs, tx_chunk_size=%u, min_send_interval=%uµs.\r\n",
          "PHY updated (GAP)",
          p.tx_phy, p.rx_phy, s.phyToStr(),
          (unsigned)s.llTxTimeUs,
          (unsigned)s.txChunkSize,
          (unsigned)s.minSendIntervalUS
        );
      }

      #ifdef ARDUINO_ARCH_ESP32
        if (s.pumpMode == PumpMode::Task) {
          s.wakeTxTask();
          s.wakeRSSITask();
        }
      #endif
      return 0;
    }

    // Not available in this NimBLE build; ignore gracefully.
    // case BLE_GAP_EVENT_L2CAP_UPDATE_REQ: 
    // {
    //   // // Peer proposes new data-lengths; don't apply immediately — wait for DATA_LEN_CHG which is authoritative.
    //   // const auto &q = ev->l2cap_update_req; // fields similar to data_len_chg (proposal)
    //   // if (s.logLevel >= INFO) {
    //   //   Serial.printf(
    //   //     "BLESerial: L2CAP DLE proposal: "
    //   //     "tx_octets=%u tx_time=%uµs rx_octets=%u rx_time=%uµs\r\n",
    //   //     (unsigned)q.tx_octets, (unsigned)q.tx_time,
    //   //     (unsigned)q.rx_octets, (unsigned)q.rx_time);
    //   // }
    //   // // No state change here; DATA_LEN_CHG will follow with negotiated values.
    //   // This variant is not available in this NimBLE build; ignore gracefully.
    //   (void)ev;      return 0;
    // }

    // Fires whenever the controller updates data length for this link
    case BLE_GAP_EVENT_DATA_LEN_CHG:
    {
      const auto &p = ev->data_len_chg; // negotiated per-link values

      // Update LL payload/time; prefer tx metrics for our TX pacing
      // If you also store RX metrics, you can mirror them here.
      const uint16_t oldTxOctets = s.llTxOctets;
      const uint16_t oldTxTimeUs = s.llTxTimeUs;
      s.llTxOctets = p.max_tx_octets;
      s.llRxOctets = p.max_rx_octets;

      // Compute a PHY-based PDU time for verification/fallback
      uint32_t computedTime = s.estimate_LL_PDUTimeUs(
        s.llTxOctets, s.phyIs2M, s.phyIsCoded, s.codedScheme);
      // Controller is authoritative whenever it provides a number.
      // If it's zero, fall back to our own estimate.
      uint32_t chosenTime = (p.max_tx_time != 0) ? p.max_tx_time : computedTime;
      s.llTxTimeUs = chosenTime;

      // Mirror RX time if provided (optional)
      if (p.max_rx_time != 0) s.llRxTimeUs = p.max_rx_time;

      s.updateTxTiming();
      // computes txChunkSize
      // minSendIntervalUS
      // low & high Water
      // lkgIntervalUs
      // sendIntervalUs
      // probing/backoff state reset

      if (s.logLevel >= INFO) {
        // Compute diagnostic sizing to confirm single-LL-PDU chunks
        const uint16_t attPayloadMax = (s.mtu > BLE_SERIAL_ATT_HDR_BYTES)
                                         ? (uint16_t)std::min<uint32_t>(BLE_SERIAL_MAX_GATT, (uint32_t)s.mtu - BLE_SERIAL_ATT_HDR_BYTES)
                                         : (uint16_t)20;
        const uint16_t mic = micBytes(s.secure);
        const uint16_t onePduMaxPayload = (s.llTxOctets > (BLE_SERIAL_L2CAP_HDR_BYTES + BLE_SERIAL_ATT_HDR_BYTES + mic))
                                            ? (uint16_t)(s.llTxOctets - (BLE_SERIAL_L2CAP_HDR_BYTES + BLE_SERIAL_ATT_HDR_BYTES) - mic)
                                            : (uint16_t)0;
        const uint16_t M = (s.llTxOctets > mic) ? (uint16_t)(s.llTxOctets - mic) : (uint16_t)0; // SDU bytes per full LL PDU
        const uint32_t sduBytes = (uint32_t)s.txChunkSize + BLE_SERIAL_L2CAP_HDR_BYTES + BLE_SERIAL_ATT_HDR_BYTES;
        const bool fitsSingle = (M > 0) && (sduBytes <= M);

        Serial.printf(
          "BLESerial: DLE updated:\r\n"
          "           tx %u→%u octets, %u→%uµs; mtu=%u\r\n"
          "           max_att_payload=%u, max_one_PDU_payload=%u\r\n"
          "           sdu_size=%u, sdu_bytes_per_full_ll_pdu=%u, fits_single_LL_PDU=%s\r\n"
          "           tx_chunk_size=%u, min_send_interval=%uµs.\r\n",
          (unsigned)oldTxOctets, (unsigned)s.llTxOctets,
          (unsigned)oldTxTimeUs, (unsigned)s.llTxTimeUs,
          (unsigned)s.mtu,
          (unsigned)attPayloadMax, (unsigned)onePduMaxPayload,
          (unsigned)sduBytes, (unsigned)M,
          fitsSingle ? "YES" : "NO",
          (unsigned)s.txChunkSize, (unsigned)s.minSendIntervalUS
        );
      }
      #ifdef ARDUINO_ARCH_ESP32
        if (s.pumpMode == PumpMode::Task) {
          s.wakeTxTask();
          s.wakeRSSITask();
        }
      #endif
      return 0;
    }

    case BLE_GAP_EVENT_MTU:
    {
      const uint16_t mtu = ev->mtu.value; // ATT MTU negotiated
      s.mtu = mtu;                        // keep class MTU in sync

      s.updateTxTiming();
      // computes txChunkSize
      // minSendIntervalUS
      // low & high Water
      // lkgIntervalUs
      // sendIntervalUs
      // probing/backoff state reset

      if (s.logLevel >= INFO) {
        Serial.printf(
          "BLESerial: MTU=%u, tx_chunk_size=%u, min_send_interval=%uµs.\r\n",
          (unsigned)mtu, (unsigned)s.txChunkSize, (unsigned)s.minSendIntervalUS
        );
      }
      return 0;
    }
  
    //
    // case BLE_GAP_EVENT_CONNECT:
    // case BLE_GAP_EVENT_DISCONNECT:
    // case BLE_GAP_EVENT_SUBSCRIBE:
    // case BLE_GAP_EVENT_NOTIFY_TX:
    // case BLE_GAP_EVENT_NOTIFY_RX:
    // case BLE_GAP_EVENT_ADV_COMPLETE:
    // case BLE_GAP_EVENT_CONN_UPDATE:
    // case BLE_GAP_EVENT_REPEAT_PAIRING:
    // case BLE_GAP_EVENT_ENC_CHANGE:
    // case BLE_GAP_EVENT_IDENTITY_RESOLVED:
    // case BLE_GAP_EVENT_PASSKEY_ACTION:
    default:
      return 0;
  }
} // end gapEventHandler ========================================================================

// ==============================================================================================
// ==============================================================================================

// ===== BLESerial begin() ======================================================================

bool BLESerial::begin(Mode newMode, const char *deviceName, Security newSecure)
{
  // Minimal init; full feature set can be added incrementally
  mode = newMode;
  secure = newSecure;
  logLevel = INFO;

  BLESerial::active = this; // allow static GAP handler to reach our instance

  // Decide desired link behavior from mode (desired != current)
  int8_t dBmAdv, dBmScan, dBmConn;

  switch (mode)
  {
  case Mode::Fast:
    mtu                 = BLE_SERIAL_MAX_MTU;
    desiredPhyMask      = BLE_GAP_LE_PHY_2M_MASK;
    desiredCodedScheme  = 0;
    desiredllTxOctets   = LL_MAX_OCTETS;
    desiredllTxTimeUs   = estimate_LL_PDUTimeUs(desiredllTxOctets, true, false, desiredCodedScheme);
    dBmAdv              = BLE_TX_DBP9;
    dBmScan             = BLE_TX_DBP9;
    dBmConn             = BLE_TX_DBP9;
    break;
  case Mode::LowPower:
    mtu                 = BLE_SERIAL_MIN_MTU;
    desiredPhyMask      = BLE_GAP_LE_PHY_1M_MASK;
    desiredCodedScheme  = 0;
    desiredllTxOctets   = LL_MAX_OCTETS;
    desiredllTxTimeUs   = estimate_LL_PDUTimeUs(desiredllTxOctets, false, false, desiredCodedScheme);
    dBmAdv              = BLE_TX_DBN9;
    dBmScan             = BLE_TX_DBN9;
    dBmConn             = BLE_TX_DBN6;
    break;
  case Mode::LongRange:
    mtu                 = BLE_SERIAL_DEFAULT_MTU;
    desiredPhyMask      = BLE_GAP_LE_PHY_CODED_MASK;
    desiredCodedScheme  = 2;
    desiredllTxOctets   = LL_MAX_OCTETS;
    desiredllTxTimeUs   = estimate_LL_PDUTimeUs(desiredllTxOctets, false, true, desiredCodedScheme);
    dBmAdv              = BLE_TX_DBP9;
    dBmScan             = BLE_TX_DBP9;
    dBmConn             = BLE_TX_DBP9;
    break;
  case Mode::Balanced:
  default:
    mtu                 = BLE_SERIAL_DEFAULT_MTU;
    desiredPhyMask      = BLE_GAP_LE_PHY_1M_MASK;
    desiredCodedScheme  = 0;
    desiredllTxOctets   = LL_MAX_OCTETS;
    desiredllTxTimeUs   = estimate_LL_PDUTimeUs(desiredllTxOctets, false, false, desiredCodedScheme);
    dBmAdv              = BLE_TX_DBN6;
    dBmScan             = BLE_TX_DBN3;
    dBmConn             = BLE_TX_DB0;
    break;
  }

  // Current, negotiated state is unknown pre-connection
  phyIs2M               = false;
  phyIsCoded            = false;
  codedScheme           = 0;

  // BLE: init stack, create service, start adv; UART: config UART
  NimBLEDevice::init(deviceName);
  if (logLevel >= INFO) {
    Serial.printf(
      "BLESerial: Device created with name %s.\r\n", deviceName);
  }
  NimBLEDevice::setCustomGapHandler(&BLESerial::gapEventHandler);
  if (logLevel >= INFO) {
    Serial.print("BLESerial: Custom Gap handler set.\r\n");
  }
  NimBLEDevice::setMTU(mtu);
  if (logLevel >= INFO) {
    Serial.printf("BLESerial: MTU set to %u.\r\n", mtu);
  }

  NimBLEDevice::setPower(dBmAdv, PWR_ADV);
  NimBLEDevice::setPower(dBmScan, PWR_SCAN);
  NimBLEDevice::setPower(dBmConn, PWR_CONN);
  if (logLevel >= INFO) {
    Serial.printf("BLESerial: Power levels set: Adv=%d, Scan=%d, Conn=%d.\r\n", dBmAdv, dBmScan, dBmConn);
  }

  powerAdv  = NimBLEDevice::getPower(PWR_ADV);
  powerScan = NimBLEDevice::getPower(PWR_SCAN);
  powerConn = NimBLEDevice::getPower(PWR_CONN);

  // Address type
  // Options:
  // BLE_OWN_ADDR_PUBLIC Use the chip’s factory-burned IEEE MAC (the “public” address). Stable, globally unique.
  // BLE_OWN_ADDR_RANDOM Use the static random address you’ve set with ble_hs_id_set_rnd(). Stable across reboots only if you persist it yourself.
  // BLE_OWN_ADDR_RPA_PUBLIC_DEFAULT Use a Resolvable Private Address (RPA) derived from your public identity. This gives privacy (rotating address) but still resolvable if the peer has your IRK (bonded).
  // BLE_OWN_ADDR_RPA_RANDODEFAULT Use an RPA derived from your random static identity.

  if (secure == Security::PasskeyDisplay ||
      secure == Security::JustWorks) {
    NimBLEDevice::setOwnAddrType(BLE_OWN_ADDR_RPA_PUBLIC_DEFAULT);
    if (logLevel >= INFO) {
      Serial.print("BLESerial: Random address initialized.\r\n");
    }
    // your client will need to reacquire the address each time you want to connect
  }
  else
  {
    NimBLEDevice::setOwnAddrType(BLE_OWN_ADDR_PUBLIC);
    if (logLevel >= INFO)  {
      Serial.print("BLESerial: Public address initialized.\r\n");
    }
    // address remains static and can be reused by the client
  }

  NimBLEDevice::setDefaultPhy(desiredPhyMask, desiredPhyMask);
  if (logLevel >= INFO) {
    Serial.printf("BLESerial: Default PHY set to %u.\r\n", desiredPhyMask);
  }

  // Suggested default data length: use safe, spec-aligned maximum (not dynamic). Typical: 251 octets, LL_DEFAULT_TIME_USµs.
  ble_gap_write_sugg_def_data_len(desiredllTxOctets, desiredllTxTimeUs);

  llTxOctets = desiredllTxOctets; // for now until negotiated at connection time
  llTxTimeUs = desiredllTxTimeUs; // for now until negotiated at connection time
  if (logLevel >= INFO) {
    Serial.printf(
      "BLESerial: Suggested default data length set: "
      "%u octets, %uµs.\r\n", 
      llTxOctets, llTxTimeUs);
  }

  // Security posture
  if (secure == Security::PasskeyDisplay) {
    NimBLEDevice::setSecurityAuth(/*bonding*/ true, /*mitm*/ true, /*sc*/ true);

    // Generate random 6-digit code; ensure leading zeros possible on display
    uint32_t key = (uint32_t)random(0UL, 1000000UL);
    passkey = key;
    NimBLEDevice::setSecurityPasskey(key);

    // IO capability: display only (ESP_IO_CAP_OUT)
    NimBLEDevice::setSecurityIOCap(BLE_HS_IO_DISPLAY_ONLY); /** Display only passkey */

    // Key distribution (init/rsp) ~ ESP_BLE_SSET_INIT_KEY / SET_RSP_KEY
    NimBLEDevice::setSecurityInitKey(KEYDIST_ENC | KEYDIST_ID);
    NimBLEDevice::setSecurityRespKey(KEYDIST_ENC | KEYDIST_ID);
    if (logLevel >= INFO) {
      Serial.print(
        "BLESerial: Secure passkey connection initialized.\r\n");
    }
  } else if (secure == Security::JustWorks) {
    NimBLEDevice::setSecurityAuth(/*bonding*/ true, /*mitm*/ false, /*sc*/ false);
    // IO capability: no input/output (ESP_IO_CAP_NONE)
    NimBLEDevice::setSecurityIOCap(BLE_HS_IO_NO_INPUT_OUTPUT); /** Just works */

    // Key distribution (init/rsp) ~ ESP_BLE_SSET_INIT_KEY / SET_RSP_KEY
    NimBLEDevice::setSecurityInitKey(KEYDIST_ENC | KEYDIST_ID);
    NimBLEDevice::setSecurityRespKey(KEYDIST_ENC | KEYDIST_ID);
    if (logLevel >= INFO) {
      Serial.print("BLESerial: Secure justworks connection initialized.\r\n");
    }
  } else if (secure == Security::None) {
    NimBLEDevice::setSecurityAuth(/*bonding*/ false, /*mitm*/ false, /*sc*/ false); // no pairing needed
    if (logLevel >= INFO) {
      Serial.print("BLESerial: Insecure connection initialized.\r\n");
    }
  }

  // Create server and service
  server = NimBLEDevice::createServer();
  if (!server) {
    if (logLevel >= ERROR) {
      Serial.print("BLESerial: ERROR: Server creation failed.\r\n");
    }
    return false;
  }
  server->setCallbacks(new ServerCallbacks(this));
  service = server->createService(BLE_SERIAL_SERVICE_UUID);
  if (!service) {
    if (logLevel >= ERROR) {
      Serial.print("BLESerial: ERROR: Service creation failed.\r\n");
    }
    return false;
  } else {
    if (logLevel >= INFO) {
      Serial.print("BLESerial: Server and Services created.\r\n");
    }
  }

  // Characteristics
  if (secure == Security::PasskeyDisplay ||
      secure == Security::JustWorks) {
    rxChar = service->createCharacteristic(
        BLE_SERIAL_CHARACTERISTIC_UUID_RX,
        NIMBLE_PROPERTY::WRITE |
        NIMBLE_PROPERTY::WRITE_NR | // write without response (faster)
        NIMBLE_PROPERTY::WRITE_ENC  // require encryption for writes (triggers pairing)
    );

    txChar = service->createCharacteristic(
        BLE_SERIAL_CHARACTERISTIC_UUID_TX,
        NIMBLE_PROPERTY::NOTIFY |
        NIMBLE_PROPERTY::READ_ENC // require encryption for notify subscription
    );
    if (logLevel >= INFO) {
      Serial.print("BLESerial: Secure Rx and Tx services initialized.\r\n");
    }

  } else {
    rxChar = service->createCharacteristic(
        BLE_SERIAL_CHARACTERISTIC_UUID_RX,
        NIMBLE_PROPERTY::WRITE |
        NIMBLE_PROPERTY::WRITE_NR // write without response (faster)
    );
    txChar = service->createCharacteristic(
        BLE_SERIAL_CHARACTERISTIC_UUID_TX,
        NIMBLE_PROPERTY::NOTIFY);
    if (logLevel >= INFO) {
      Serial.print("BLESerial: Insecure Rx and Tx services initialized.\r\n");
    }
  }

  if (!rxChar || !txChar) {
    if (logLevel >= ERROR) {
      Serial.print("BLESerial: ERROR: Characteristic creation failed.\r\n");
    }
    return false;
  }

  // Set attribute max length for RX/TX characteristics.
  // GATT attribute values are limited to 512 bytes by spec; ATT payload per PDU is MTU-3.
  // Use min(512, mtu-3) as initial cap so we can accept/emit up to negotiated MTU later.
  {
    // Attribute max length is implicitly handled by NimBLE; no explicit setMaxLen API in 2.x
    // Left here for future versions supporting an explicit cap.
    (void)mtu;
  }

  // Callbacks
  txChar->setCallbacks(new TxCallbacks(this));
  rxChar->setCallbacks(new RxCallbacks(this));

  // Start the service
  service->start();
  if (logLevel >= INFO) {
    Serial.print("BLESerial: Services started.\r\n");
  }

  // Primary Advertising: Flags and Service UUID
  advertising = NimBLEDevice::getAdvertising();
  if (mode == Mode::Fast) {
    advertising->setMinInterval(0x00A0); // 100ms
    advertising->setMaxInterval(0x00F0); // 150ms
  } else if (mode == Mode::LowPower) {
    advertising->setMinInterval(0x0640); // 1.0 s
    advertising->setMaxInterval(0x0C80); // 2.0 s
  } else {
    advertising->setMinInterval(0x0320); // 0.5 s
    advertising->setMaxInterval(0x0640); // 1.0 s
  }
  // Flags are recommended in primary ADV (general discoverable, no BR/EDR)
  advData.setFlags(BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP);

  // Put service UUID in the primary ADV
  advData.addServiceUUID(BLE_SERIAL_SERVICE_UUID);

  // If you have multiple services, call addServiceUUID(...) for each:
  // advData.addServiceUUID(NimBLEUUID(SERVICE_UUID_2));

  advData.addTxPower();

  // Scan Response: put the full name here (saves ADV space)
  scanData.setName(deviceName);
  scanData.setAppearance(BLE_SERIAL_APPEARANCE);
  const uint8_t mfg[] = {0xFF, 0xFF}; // 0xFFFF + 27 bytes max
  scanData.setManufacturerData(std::string((const char *)mfg, sizeof(mfg)));
  advertising->setAdvertisementData(advData);
  advertising->setScanResponseData(scanData);
  advertising->start();
  if (logLevel >= INFO) {
    Serial.print("BLESerial: Advertising started.\r\n");
  }

  // Initialize watermarks
  updateWaterMarks(static_cast<size_t>(txChunkSize));
 
  #ifdef ARDUINO_ARCH_ESP32
    startRSSITask(); // creates if absent
    startTxTask(); // creates if absent; does NOT actively pump until notified
  #endif

  // Print MAC (purely informational)
  deviceMac = NimBLEDevice::getAddress().toString();
  for (char &c : deviceMac)
    c = (char)toupper((unsigned char)c);
  if (logLevel >= INFO) {
    Serial.printf("BLESerial: MAC=%s.\r\n", deviceMac.c_str());
  }

  randomSeed(analogRead(0));
  if (logLevel >= INFO) {
    Serial.print("BLESerial: Initialization completed.\r\n");
  }

  return true;
}

// ===== BLESerial end() ========================================================================

void BLESerial::end()
{
  // Stop RSSI task first (ESP32)
  #ifdef ARDUINO_ARCH_ESP32
    stopTxTask();
    stopRSSITask();
  #endif

  // Stop advertising
  if (advertising)
    advertising->stop();


  // Disconnect active client (if any)
  if (server && connHandle != BLE_HS_CONN_HANDLE_NONE) {
    server->disconnect(connHandle);
    connHandle = BLE_HS_CONN_HANDLE_NONE;
  }

  // Stop service (characteristics live under service)
  if (service) {
    // NimBLEService has no stop() in this version; services stop on deinit.
  }

  // Release NimBLE resources (frees server/service/chars/adv objects)
  NimBLEDevice::setCustomGapHandler(nullptr);
  NimBLEDevice::deinit(true);

  // Clear pointers after deinit
  server             = nullptr;
  service            = nullptr;
  rxChar             = nullptr;
  txChar             = nullptr;
  advertising        = nullptr;

  // Reset link/PHY state
  deviceConnected    = false;
  clientSubscribed   = false;
  phyIs2M            = false;
  phyIsCoded         = false;
  codedScheme        = 0;
  desiredCodedScheme = 0;
  connHandle         = BLE_HS_CONN_HANDLE_NONE;

  // Reset pacing/timing
  TX_CRITICAL_ENTER(this);
  txOk               = false;
  pendingLen         = 0;
  txPending          = false;
  lastTxUs           = 0;
  TX_CRITICAL_EXIT(this);
  llTxOctets         = LL_MAX_OCTETS;
  llTxTimeUs         = LL_DEFAULT_TIME_US;
  llRxOctets         = LL_MAX_OCTETS;
  llRxTimeUs         = LL_DEFAULT_TIME_US;
  updateTxTiming();
  // computes txChunkSize
  // minSendIntervalUS
  // low & high Water
  // lkgIntervalUs
  // sendIntervalUs
  // probing/backoff state reset

  // Drain buffers
  // TX: drop any queued bytes
  size_t txUsed = txBuf.available();
  if (txUsed) txBuf.consume(txUsed);

  // RX: pop until empty
  uint8_t b;
  while (rxBuf.pop(b) == 1)
  { /* discard */ }

  // Reset stats/counters
  bytesRx            = 0;
  bytesTx            = 0;
  rxDrops            = 0;
  mtuRetryCount      = 0;
  eappRetries        = 0;

  // Reset watermarks
  updateWaterMarks(static_cast<size_t>(txChunkSize));
 
  // Detach active instance pointer
  if (BLESerial::active == this) {
    BLESerial::active = nullptr;
  }

  if (logLevel >= INFO) {
    Serial.println(
      "BLESerial ended: BLE deinitialized and resources released.");
  }
}

// ===== BLESerial read/write/flush ======================================================================

int BLESerial::available() {
  // Stream::available(): number of bytes that can be read without blocking
  return readAvailable();
}

int BLESerial::readAvailable() {
  return (int)rxBuf.available();
}

int BLESerial::read() {
  // Assumes RingBuffer::pop() returns int (or -1 when empty)
  uint8_t b = 0;
  if (rxBuf.pop(b) == 1)
    return (int)b;
  return -1;
}

// Helper to read up to n bytes into dst using RingBuffer::pop(T*, n)
int BLESerial::read(uint8_t *dst, size_t n) {
  if (!dst || n == 0) return 0;
  return (int)rxBuf.pop(dst, n);
}

// Implement Stream::peek() using RingBuffer::peek(T&)
int BLESerial::peek() {
  uint8_t b = 0;
  if (rxBuf.peek(b) == 1) return (int)b;
  return -1;
}

// Helper to peak up to n bytes into dst using RingBuffer::peek(T*, n)
int BLESerial::peek(uint8_t *dst, size_t n) {
  if (!dst || n == 0) return 0;
  return (int)rxBuf.peek(dst, n);
}

void BLESerial::flush() {
  // If there's no active subscription, nothing will drain—return immediately.
  if (!isSubscribed()) {
    return;
  }

  const uint32_t deadline = millis() + FLUSH_MAX_WAIT_MS;
  while (txBuf.available() > 0) {
    pumpTx();
    if (txBuf.available() == 0) break;
    if (!isSubscribed()) break; // link went away mid-flush
    if ((int32_t)(millis() - deadline) >= 0) break; // bounded wait
    delay(1);
  }
}

bool BLESerial::writeReady() const {
// If false you should delay and not write
  #ifdef ARDUINO_ARCH_ESP32
    // Read readiness atomically w.r.t. TX state to reduce races with the TX task
    TX_CRITICAL_ENTER(const_cast<BLESerial*>(this));
  #endif
  // Pop-based model: readiness depends on buffer lock (watermarks) and having a subscriber.
  // Do NOT gate on txPending; producers are decoupled from in-flight notifies.
  bool ready = !txLocked && isSubscribed();
  #ifdef ARDUINO_ARCH_ESP32
    TX_CRITICAL_EXIT(const_cast<BLESerial*>(this));
  #endif
  return ready;
}

size_t BLESerial::write(uint8_t b) {
  // Write single byte

  if (!isSubscribed()) return 0; // no client subscribed, nothing will drain

  // Respect high-water lock; only unlock once below lowWater
  if (txLocked) {
    if (txBuf.available() <= lowWater) {
      txLocked = false;
    } else {
      return 0;
    }
  }

  // Push single byte
  size_t pushed = txBuf.push(&b, 1, false);

  if (pushed) {
    // Adjust lock state after push
    if (txBuf.available() >= highWater) txLocked = true;
    #ifdef ARDUINO_ARCH_ESP32
      // Wake up TX task if in task mode
      if (pumpMode == PumpMode::Task && !txPending)
        wakeTxTask();
    #endif
  }

  return pushed;
}

size_t BLESerial::write(const uint8_t *p, size_t n) {
  if (!p || n == 0) return 0;

  if (!isSubscribed()) return 0; // no client subscribed, nothing will drain

  if (txLocked) {
    if (txBuf.available() <= lowWater) {
      txLocked = false;
    } else {
      return 0;
    }
  }

  const size_t pushed = txBuf.push(p, n, false);

  if (pushed) {
    if (txBuf.available() >= highWater) txLocked = true;
    #ifdef ARDUINO_ARCH_ESP32
      if (pumpMode == PumpMode::Task && !txPending)
        wakeTxTask();
    #endif
  }
  return pushed;
}

size_t BLESerial::writeTimeout(const uint8_t *p, size_t n, uint32_t timeoutMs) {
  // blocking write with timeout, returns number of bytes written
  // ensure all bytes are queued or timeout expires
  if (!p || n == 0) return 0;
  const uint32_t endAt = millis() + timeoutMs;
  size_t pushed = 0;
  while (pushed < n) {
    // Respect staged/in-flight frames: wait until they clear
    if (txPending || pendingLen) {
      if ((int32_t)(millis() - endAt) >= 0) break; // timeout
      #ifdef ARDUINO_ARCH_ESP32
        if (pumpMode == PumpMode::Polling) {
          pumpTx();
        } else if (!txPending && isSubscribed()) {
          wakeTxTask();
        }
      #else
        pumpTx();
      #endif
      delay(1);
      continue;
    }
    // Handle lock state
    if (txLocked) {
      if (txBuf.available() <= lowWater) {
        txLocked = false; // unlock and allow write(s)
      } else {
        if ((int32_t)(millis() - endAt) >= 0) break; // timeout
        #ifdef ARDUINO_ARCH_ESP32
          if (pumpMode == PumpMode::Polling) {
            pumpTx();
          } else if (!txPending && isSubscribed()) {
            wakeTxTask();
          }
        #else
          pumpTx();
        #endif
        delay(1);
        continue; // still locked
      }
    }
    size_t s = txBuf.push(p + pushed, n - pushed, false);
    pushed += s;
    // After push, buffer only grows: check high-water lock
    if (txBuf.available() >= highWater) txLocked = true;
    if (pushed == n)
      break;
    #ifdef ARDUINO_ARCH_ESP32
      if (pumpMode == PumpMode::Polling) {
        pumpTx();
      } else if (!txPending && isSubscribed()) {
        wakeTxTask();
      }
    #else
      pumpTx();
    #endif
    if ((int32_t)(millis() - endAt) >= 0) break; // timeout
    delay(1);
    // After wait (no push), buffer likely shrank: check low-water unlock
    if (txBuf.available() <= lowWater) txLocked = false;
  }
  return pushed;
}

size_t BLESerial::printf(const char *fmt, ...) {
  if (!fmt) return 0;

  char buf[128]; // adjust size as you like
  va_list ap;
  va_start(ap, fmt);
  int n = vsnprintf(buf, sizeof(buf), fmt, ap);
  va_end(ap);

  if (n <= 0) return n;

  // If output was truncated, n is the number that would have been written,
  // but we only send sizeof(buf) - 1 bytes.
  size_t toSend = (n < (int)sizeof(buf)) ? (size_t)n : (sizeof(buf) - 1);
  size_t pushed = write(reinterpret_cast<const uint8_t*>(buf), toSend);
  return pushed;
}

void BLESerial::printStats(Stream &out) {
  out.print(F("BLESerial Stats:\r\n"));

  out.print(F("  Mode: "));
  switch (mode) {
    case Mode::Fast:      out.print(F("Fast")); break;
    case Mode::LowPower:  out.print(F("LowPower")); break;
    case Mode::LongRange: out.print(F("LongRange")); break;
    case Mode::Balanced:  out.print(F("Balanced")); break;
  }
  out.print(F("\r\n"));

  out.print(F("  Security: "));
  if (secure == Security::PasskeyDisplay)  out.print(F("PasskeyDisplay"));
  else if (secure == Security::JustWorks)  out.print(F("JustWorks"));
  else                                     out.print(F("None"));

  // Link summary
  out.print(F("  Link: connected="));
  out.print(deviceConnected ? F("yes") : F("no"));
  out.print(F(" subscribed="));
  out.print(clientSubscribed ? F("yes") : F("no"));
  out.print(F(" PHY="));
  out.print(getPhy());
  out.print(F("\r\n"));

  // MTU / Chunk and whether it fits a single LL PDU
  out.print(F("  MTU: "));    out.print(mtu);
  out.print(F("  Chunk: "));  out.print(txChunkSize);
  uint16_t mic = micBytes(secure);
  uint16_t M   = (llTxOctets > mic) ? (uint16_t)(llTxOctets - mic) : 0u; // SDU bytes per full LL PDU
  uint32_t sduBytes = (uint32_t)txChunkSize + BLE_SERIAL_L2CAP_HDR_BYTES + BLE_SERIAL_ATT_HDR_BYTES;
  bool fits1 = (M > 0) && (sduBytes <= M);
  out.print(F("  Fits 1 PDU: "));
  out.print(fits1 ? F("YES") : F("NO"));
  out.print(F("\r\n"));

  // Timing
  out.print(F("  Timing (us): send="));
  out.print(sendIntervalUs);
  out.print(F(" min="));
  out.print(minSendIntervalUS);
  out.print(F(" lkg="));
  out.print(lkgIntervalUs);
  out.print(F("\r\n"));

  // LL negotiated parameters
  out.print(F("  LL: tx_octets="));
  out.print(llTxOctets);
  out.print(F(" tx_time="));
  out.print(llTxTimeUs);
  out.print(F("us rx_octets="));
  out.print(llRxOctets);
  out.print(F(" rx_time="));
  out.print(llRxTimeUs);
  out.print(F("us\r\n"));

  // Buffers (used/total and free, plus watermarks/lock/pending)
  size_t txUsed = txBuf.available();
  size_t txFree = txBuf.capacity() - txUsed;
  size_t rxUsed = rxBuf.available();
  size_t rxFree = rxBuf.capacity() - rxUsed;

  out.print(F("  TX Buffer: "));
  out.print(txUsed); out.print(F("/")); out.print(txBuf.capacity());
  out.print(F(" (free ")); out.print(txFree); out.print(F(")"));
  out.print(F(" low=")); out.print(lowWater);
  out.print(F(" high=")); out.print(highWater);
  out.print(F(" locked=")); out.print(txLocked ? F("yes") : F("no"));
  out.print(F(" pending=")); out.print(txPending ? F("yes") : F("no"));
  out.print(F(" pending length=")); out.print(pendingLen);
  out.print(F("\r\n"));

  out.print(F("  RX Buffer: "));
  out.print(rxUsed); out.print(F("/")); out.print(rxBuf.capacity());
  out.print(F(" (free ")); out.print(rxFree); out.print(F(")"));
  out.print(F("\r\n"));

    // RSSI (if connected)
  if (isConnected()) {
    out.print(F("  RSSI: "));
    out.print(rssiAvg);
    out.print(F(" dBm (raw "));
    out.print(rssiRaw);
    out.print(F(")\r\n"));
  }
  // Totals and drops
  out.print(F("  Bytes TX: "));
  out.print(bytesTx);
  out.print(F(" Bytes RX: "));
  out.print(bytesRx);
  out.print(F(" TxDrops: "));
  out.print(txDrops);
  out.print(F(" RxDrops: "));
  out.print(rxDrops);
  out.print(F("\r\n"));

  // Error counters
  out.print(F("  Errors: "));
  out.print(F("EAPP=")); out.print(eappCount);
  out.print(F(" ENOMEM=")); out.print(enomemCount);
  out.print(F(" Cong=")); out.print(congestionCount);
  out.print(F(" Escalations=")); out.print(lkgEscalateCount);
  out.print(F("\r\n"));
}

void BLESerial::clearStats() {
  bytesTx = 0;
  bytesRx = 0;
  txDrops = 0;
  rxDrops = 0;
  eappCount = 0;
  enomemCount = 0;
  congestionCount = 0;
  lkgEscalateCount = 0;
}

const char* BLESerial::phyToStr() const {
  if (phyIsCoded) {
    if (codedScheme == 2)
      return "CODED(S2)";
    else
      return "CODED(S8)";
  } else {
    if (phyIs2M)
      return "2M";
    else
      return "1M";
  }
}

// ===== BLESerial update, stageTx, pumpTx, setPower ==============================================

void BLESerial::update() {
  #ifdef ARDUINO_ARCH_ESP32
    // Use portable polling pump and not ESP32 FreeRTOS task
    if (pumpMode == PumpMode::Polling)
    {
      pumpTx();
    }
  #else
    // Portable BLE transmit data (ESP32 uses FreeRTOS task)
    pumpTx();
    // Portable RSSI polling (ESP32 uses FreeRTOS task)
    if (isConnected()) {
      uint32_t now = millis();
      if ((now - lastRSSIMs) >= RSSI_INTERVAL_MS)
        adjustLink();
    }
  #endif
}

bool BLESerial::stageTx() {
  // Stage the next chunk by popping from txBuf into 'pending'.
  // Atomicity: guard txPending/pendingLen checks and staging with TX_CRITICAL
  //   so no other context can alter them mid-stage.
  // RingBuffer has its own lock for pop.

  // Do not stage if
  //  - txPending : an notify is in flight
  //  - something already staged (should not happen)
  TX_CRITICAL_ENTER(this);
  // if (txPending || pendingLen != 0) {
  if (pendingLen != 0) {
    TX_CRITICAL_EXIT(this);
    return false;
  }

  size_t avail = txBuf.available();
  if (avail == 0) {
    TX_CRITICAL_EXIT(this);
    return false;
  }

  size_t toSend = (txChunkSize <= avail) ? txChunkSize : avail;
  pendingLen = txBuf.pop(pending, toSend);
  // Tag epoch at staging time to detect staleness after renegotiation
  pendingChangeEpoch = chunkChangeEpoch;
  // Mark in-flight before notify to avoid race where onStatus clears pendingLen
  // before we set txPending, which could leave txPending=true with pendingLen=0
  txPending = (pendingLen > 0);
  TX_CRITICAL_EXIT(this);

  if (pendingLen == 0) return false;

  // If characteristic vanished unexpectedly, account and drop staged chunk
  if (!txChar) {
    TX_CRITICAL_ENTER(this);
    txDrops    += pendingLen;
    pendingLen  = 0;
    txPending   = false;
    TX_CRITICAL_EXIT(this);
    return false;
  }
  // Set value (NimBLE copies internally) 
  txChar->setValue(pending, pendingLen);

  // Notify
  const bool notified = txChar->notify();
  if (!notified) {
    // Notify failed: drop staged data per streaming policy and account
    TX_CRITICAL_ENTER(this);
    txDrops    += pendingLen;
    pendingLen  = 0;
    txPending   = false;
    TX_CRITICAL_EXIT(this);
    if (logLevel >= WARNING) {
      Serial.println("BLESerial: notify() failed, dropped staged chunk.");
    }
    return false;
  }
  return true;
} // end stageTx

void BLESerial::pumpTx() {
  // Skip if not subscribed or no TX characteristic available; avoids dropping popped data
  if (!isSubscribed() || txChar == nullptr) return;

  // Check if previous TX succeeded
  TX_CRITICAL_ENTER(this);
  if (txOk) txOk = false; // clear success latch
  // Ensure producers can resume once buffer dropped below lowWater, even without a write()
  if (txBuf.available() <= lowWater) txLocked = false;
  TX_CRITICAL_EXIT(this);

  // Time to send next chunk?
  uint32_t now = micros();
  if ((uint32_t)(now - lastTxUs) < sendIntervalUs)
    return;
  
  // Try to stage and send next chunk
  if (pendingLen == 0) {
    if (stageTx()) {
      lastTxUs = now;
    }
  }
} // end pumpTx

// ===== BLESerial helpers ======================================================================
// computeTxChunkSize: compute max chunk size per notify based on MTU, LL octets, mode, encryption
// computeSendIntervalUs: compute interval between notifies based on chunk size, LL octets/time, mode, encryption
// updateWaterMarks: compute low- and highwater mark for tx buffer based on chunk size
// updateTxTiming: recompute txChunkSize and minSendIntervalUS based on current MTU, LL octets, mode, encryption
// estimate_LL_PDUTimeUs: compute LL PDU time inµs based on octets, PHY, coding scheme
// requestMtu: request MTU change and handle retries
// ==============================================================================================

uint16_t BLESerial::computeTxChunkSize(uint16_t mtuVal,
                                       uint16_t llTxOctets,
                                       Mode modeVal,
                                       Security securityVal,
                                       size_t txCapacity)
{
  // Compute chunk size so that its guaranteed to fit into MTU 
  // without unnecessary fragmentation at ATT layer and LL layer.

  // Base payload is MTU-3 (ATT header). 
  // For FAST mode, allow up to 2 LL PDUs to reduce per-notify overhead
  //   otherwise keep within a single LL PDU.
  // MIC is 4 when encryption is on, otherwise 0. 
  // MIC reduces available payload.

  // Max payload that fits in ONE LL PDU carrying an L2CAP SDU with ATT notify:
  //   onePduMax = llTxOctets - (BLE_SERIAL_L2CAP_HDR_BYTES + BLE_SERIAL_ATT_HDR_BYTES + MIC)1
  //             = llTxOctets - 7 (-4 if encrypted)

  // ATT value limit from MTU (exclude 3B ATT value header)
  uint16_t attPayload = 0u;
  if (mtuVal > BLE_SERIAL_ATT_HDR_BYTES) {
    attPayload = static_cast<uint16_t>(mtuVal - BLE_SERIAL_ATT_HDR_BYTES);
  }

  // Spec cap (common practice): 512 max attribute value
  if (attPayload > BLE_SERIAL_MAX_GATT)
    attPayload = BLE_SERIAL_MAX_GATT;

  const uint16_t hdrBytes = BLE_SERIAL_L2CAP_HDR_BYTES + BLE_SERIAL_ATT_HDR_BYTES; // 7
  uint16_t micPerPdu = 0u;
  if (securityVal == Security::PasskeyDisplay ||
      securityVal == Security::JustWorks)
    micPerPdu = BLE_SERIAL_ENCRYPT_BYTES; // 4

  // Max payload that fits in ONE LL PDU:
  //   fragment_len1_max = llTxOctets - 7 - MIC
  uint16_t onePduMax = 0u;
  if (llTxOctets > hdrBytes + micPerPdu)
    onePduMax = static_cast<uint16_t>(llTxOctets - hdrBytes - micPerPdu);

  // Max payload that fits in TWO LL PDUs:
  //   total_fragments_max = (llTxOctets - MIC) + (llTxOctets - MIC) - 7
  //                       = 2 * llTxOctets - 7 - 2*MIC
  uint16_t twoPduMax = onePduMax;
  if (llTxOctets > (hdrBytes + micPerPdu)) {
    uint32_t total = 2u * llTxOctets;
    if (total > (hdrBytes + 2u * micPerPdu)) {
      uint32_t usable = total - hdrBytes - 2u * micPerPdu;
      if (usable > 0xFFFFu) usable = 0xFFFFu;
      twoPduMax = static_cast<uint16_t>(usable);
    }
  }

  // Choose limit by mode
  // TEMPORARLY DISABLED TO DEBUG ISSUES WITH 2-PDU FRAGMENTS
  // uint16_t limit = (modeVal == Mode::Fast) ? twoPduMax : onePduMax;
  uint16_t limit = onePduMax;

  // chunkSize must not exceed half the buffer or low-high water hysteresis collapses
  size_t cap2 = txCapacity / 2;
  if (cap2 > 0xFFFFu) cap2 = 0xFFFFu;
  if (limit > static_cast<uint16_t>(cap2))
    limit = static_cast<uint16_t>(cap2);

  // Cap by ATT MTU-derived payload
  if (attPayload < limit) limit = attPayload;

  // Keep a practical floor (legacy 1-PDU safe)
  if (limit < 20) limit = 20;

  return limit;
}


void BLESerial::updateWaterMarks(size_t chunkSize)
// capacity sof buffer should be at least 2 * chunkSize for this to work
// lowWater 25% of capacity
// highWater 75% of capacity
{
  const size_t cap = txBuf.capacity();
  const size_t used = txBuf.available();
  if (cap == 0) {
    lowWater = 0;
    highWater = 0;
    txLocked = false;
    return;
  }  

  if (cap >= 4 * chunkSize) {
    // larger buffers
    lowWater  = cap / 4;               // 25% of buffer capacity, min 1 chunk
    highWater = (3 * cap) / 4;         // 75% of buffer capacity, min 1 chunk
  } else {
    // small / medium buffers
    lowWater  = chunkSize;             // 1 chunk
    if (cap > chunkSize) {
      highWater = cap - chunkSize + 1; // reserve no chunk
    } else {
      if (cap > 1)
        highWater = cap - 1;
      else
        highWater = 0;
    }
  }

  // Enforce lowWater < highWater
  if (lowWater >= highWater) {
    if (highWater + 1 < cap) {
      lowWater  = highWater - 1;
    } else {
      lowWater  = (highWater > 0) ? highWater - 1 : 0;
    }
  }

  // Reconcile lock state
  if      (used >= highWater) txLocked = true;
  else if (used <= lowWater)  txLocked = false;

}

void BLESerial::updateTxTiming() {
  // computes txChunkSize
  // minSendIntervalUS
  // low & high Water
  // lkgIntervalUs
  // sendIntervalUs
  // probing/backoff state reset

  const uint16_t oldChunk = txChunkSize;
  txChunkSize       = computeTxChunkSize(mtu, llTxOctets, mode, secure, txBuf.capacity());
  if (txChunkSize != oldChunk) chunkChangeEpoch++; // Bump epoch so we can detect stale staged frames
  minSendIntervalUS = computeSendIntervalUs(txChunkSize);
  updateWaterMarks(static_cast<size_t>(txChunkSize));

  // Clamp the active interval to the current floor if requested or out of bounds
  if (sendIntervalUs == 0 || sendIntervalUs < minSendIntervalUS)
    sendIntervalUs = minSendIntervalUS;

  // After any renegotiation, make LKG match the active interval (the new floor)
  lkgIntervalUs = sendIntervalUs;

  // Reset probing/backoff state to the new floor
  TX_CRITICAL_ENTER(this);
  probing           = false;
  probeSuccesses    = 0;
  probeFailures     = 0;
  lkgFailStreak     = 0;
  recentlyBackedOff = false;
  cooldownSuccess   = 0;
  successStreak     = 0;
  eappRetries       = 0;
  TX_CRITICAL_EXIT(this);
}

// Helper
// Central staged-frame cleanup helper for pop-based staging model.
// accountLoss      -> if true, add pendingLen to txDrops before clearing.
// considerUnlock   -> if true, conditionally unlock producers when usage <= lowWater.
void BLESerial::dropStagedAccounting(bool accountLoss, bool considerUnlock) {
  TX_CRITICAL_ENTER(this);
  if (pendingLen) {
    if (accountLoss) txDrops += pendingLen;
    pendingLen = 0;
    txPending  = false;
    txOk       = false;
    if (considerUnlock && txBuf.available() <= lowWater) txLocked = false;
  } else {
    // Defensive: clear flags even if pendingLen already zero.
    if (txPending || txOk) {
      txPending = false;
      txOk      = false;
      if (considerUnlock && txBuf.available() <= lowWater) txLocked = false;
    }
  }
  TX_CRITICAL_EXIT(this);
}

// Helper
// Escalate LKG (last-known-good) pacing interval.
// gated=true  → require cooldown interval since lastEscalateAtUs AND txBuf usage >= lowWater.
// gated=false → unconditional escalation (e.g. ENOMEM strong backoff).
// Returns true if escalation applied.
bool BLESerial::escalateLKG(bool gated) {
  uint32_t now = (uint32_t)micros();
  if (gated) {
    if ((uint32_t)(now - lastEscalateAtUs) < ESCALATE_COOLDOWN_US) return false;
    if (txBuf.available() < lowWater) return false; // avoid relaxing interval when buffer not under pressure
  }
  uint32_t next = (lkgIntervalUs * LKG_ESCALATE_NUM) / LKG_ESCALATE_DEN;
  if (next < minSendIntervalUS) next = minSendIntervalUS;
  // Only escalate if interval actually grows (protect against floor equality)
  if (next <= lkgIntervalUs) {
    // Still update timestamps if gated escalation qualifies (avoids re-trigger flood)
    if (gated) lastEscalateAtUs = now;
    return false;
  }
  lkgIntervalUs  = next;
  sendIntervalUs = next;
  lastTxUs       = now; // respect slower pacing immediately
  lastEscalateAtUs = now;
  lkgEscalateCount++;
  return true;
}

// Helper
// Determine per-PDU MIC bytes based on security mode
uint16_t BLESerial::micBytes(BLESerial::Security sec) {
  return (sec == Security::PasskeyDisplay || sec == Security::JustWorks) ? BLE_SERIAL_ENCRYPT_BYTES : 0;
}

uint32_t BLESerial::computeSendIntervalUs(uint16_t chunkSize) {
  // Compute time to transmit one chunk of given size over the link
  // Uses the negotiated llTxOctets and llTxTimeUs (globals)

  // mic per LL PDU (inside L)
  uint16_t mic = micBytes(secure);
  
  // Effective per-PDU capacity for SDU bytes
  uint16_t M = 0;
  if (llTxOctets > mic)  M = static_cast<uint16_t>(llTxOctets - mic);

  // If M is zero (shouldn't happen), bail out with a safe large interval
  if (M == 0) return 1000000u;

  // Total SDU bytes to send (ATT + L2CAP headers + payload)
  const uint32_t sduBytes = static_cast<uint32_t>(chunkSize) + BLE_SERIAL_ATT_HDR_BYTES + BLE_SERIAL_L2CAP_HDR_BYTES;

  // How many full PDUs and leftover SDU bytes
  uint32_t fullPduCount = sduBytes / M;
  uint32_t lastPduSize = sduBytes - fullPduCount * M;

  uint32_t totalUs = 0u;

  // Use negotiated llTxTimeUs as authoritative per-full-PDU time when available (>0),
  // otherwise compute per-PDU time from PHY and octets.
  uint32_t perFullPduUs = (llTxTimeUs > 0) ? llTxTimeUs : estimate_LL_PDUTimeUs(llTxOctets, phyIs2M, phyIsCoded, codedScheme);
  if (fullPduCount) totalUs += fullPduCount * perFullPduUs;

  // Estimate the remaining partial PDU time by linear interpolation
  // When used properly there should be no partial PDU here since we align to MTU
  if (lastPduSize) {
    // linear interpolation, round up to avoid underestimating
    uint32_t lastPduUs = (perFullPduUs * lastPduSize + (M - 1)) / M;
    totalUs += lastPduUs;
  }

  // Last (partial) PDU: compute its octet length and compute time from PHY for accuracy.
  // if (lastPduSize) {
  //   uint32_t lastOctets = lastPduSize + BLE_SERIAL_L2CAP_HDR_BYTES + BLE_SERIAL_ATT_HDR_BYTES + mic;
  //   if (lastOctets < LL_MIN_OCTETS) lastOctets = LL_MIN_OCTETS;
  //   if (lastOctets > llTxOctets) lastOctets = llTxOctets;
  //   uint32_t lastPduUs = estimate_LL_PDUTimeUs(static_cast<uint16_t>(lastOctets), phyIs2M, phyIsCoded, codedScheme);
  //   totalUs += lastPduUs;
  // }

  // Conservative guard: assume we need an additional spacing to account for stack/processing jitter
  const uint32_t guardNum = GUARD_NUM; // +10%
  return (totalUs * guardNum) / 100u;
}

uint32_t BLESerial::estimate_LL_PDUTimeUs(uint16_t llOctets,
                                       bool phy2M,
                                       bool phyCoded,
                                       uint8_t codedScheme) {
  /*
    estimate LL PDU Time in us:
    This is optimistic theoretical estimate based on PHY parameters.

    Compute the theoretical time to transmit a Link Layer PDU of given octet length on
    the selected PHY.

    Let L            = Octet Length (bytes)
    Max octet length = 251 bytes

    payload          = L - 4 (L2CAP) - 3 (ATT) - (4 (MIC) if encrypted) in bytes

    LE 1M:
    t_us             = ( Preamble(8) + AA(32) + LLhdr(16) + 8*L + CRC(24) ) / 1 + IFS(150)
                     = ( 80 + 8*L ) + 150

    LE 2M:
    t_us             = ( Preamble(16) + AA(32) + LLhdr(16) + 8*L + CRC(24) ) / 2 + IFS(150)
                     = ( 88 + 8*L ) / 2 + 150
                     = 44 + 4*L + 150

    LE  Coded:
    t_us             = Preamble(80)
                      + AA(32)*8
                      + (CI+TERM1)(5)*8
                      + LLhdr(16)*8
                      + (8*L + CRC(24))*S
                      + IFS(150)

                     = 504 + S*(8*L + 24) + 150

    Examples:
    if L=251 and
    1M: t = 80 + 8*251 + 150 = 80 + 2008 + 150                    = 2238µs
    2M: t = (88 + 8*251)/2 + 150 = 2096/2 + 150 = 1048 + 150      = 1198µs
    Coded S=2: t = 504 + 2*(8*251 + 24) + 150 = 504 + 4064 + 150  = 4718µs
    Coded S=8: t = 504 + 8*(8*251 + 24) + 150 = 504 + 16256 + 150 = 16910µs

  */

  // Inter-frame space (us)
  constexpr uint32_t IFS_US = 150;

  // Clamp octets to LL spec range
  if (llOctets < LL_MIN_OCTETS) llOctets = LL_MIN_OCTETS;
  if (llOctets > LL_MAX_OCTETS) llOctets = LL_MAX_OCTETS;

  if (!phyCoded) {
    codedScheme = 0;
    // ---------- Uncoded PHY ----------
    // LE 1M: t = 80 + 8*L + 150
    // LE 2M: t = (88 + 8*L)/2 + 150
    if (phy2M) {
      return ((88u + 8u * llOctets) / 2u) + IFS_US;
    } else {
      return (80u + 8u * llOctets) + IFS_US;
    }
  } else {
    if (codedScheme != 2 && codedScheme != 8) {
      // Default to S=8 if unknown
      codedScheme = 8;
    }
     // ---------- LE Coded PHY ----------
    // Access Address, CI/TERM1, and LL header are always S=8 coded.
    // Payload and CRC are coded at S (2 or 8).
    // t = 504 + S*(8*L + 24) + 150   [us], at 1msym/s
    const uint32_t S = (codedScheme == 2) ? 2u : 8u;
    return 504u + S * (8u * llOctets + 24u) + IFS_US;
  }
}

bool BLESerial::requestMTU(uint16_t newMtu) {
  // We want to change the MTU
  // Note: Chunk/timing are re-derived when NimBLE confirms the MTU (onMTUChange),
  // so this call just asks the stack to renegotiate.

  if (newMtu < BLE_SERIAL_MIN_MTU) newMtu = BLE_SERIAL_MIN_MTU;
  if (newMtu > BLE_SERIAL_MAX_MTU) newMtu = BLE_SERIAL_MAX_MTU;

  // Remember desired MTU; actual will be delivered in onMTUChange after negotiation.
  mtu = newMtu;
  NimBLEDevice::setMTU(mtu);

  // Adjust attribute max lengths now to avoid EAPP on setValue/write.
  // NOT AVAILABLE IN THIS VERSION OF NIMBLE (left for future)
  // const uint16_t attMax = (mtu > BLE_SERIAL_ATT_HDR_BYTES)
  //     ? (uint16_t)std::min<int>(BLE_SERIAL_MAX_GATT, (int)mtu - BLE_SERIAL_ATT_HDR_BYTES)
  //     : (uint16_t)20;

  // if (txChar) txChar->setMaxLen(attMax);
  // if (rxChar) rxChar->setMaxLen(attMax);

  // Recompute local chunking against the requested MTU; onMTUChange will refine later.
  // this is done onMTUChange now to use the actual negotiated MTU.  
  
  return true;
}

void BLESerial::setPower(int8_t dBm, NimBLETxPowerType scope) {
  NimBLEDevice::setPower(dBm, scope);
  if (logLevel >= INFO) {
    Serial.printf(
      "BLESerial: TX Power set to %d dBm (scope %d).\r\n", 
      dBm, (int)scope);
  }
}

// ===== RSSI and TX Tasks ===============================================================================================

#ifdef ARDUINO_ARCH_ESP32

void BLESerial::RSSITask(void *arg) {
  BLESerial *self = static_cast<BLESerial *>(arg);
  if (!self) {
    // Nothing to do if task wasn't given a valid instance pointer
    vTaskDelay(pdMS_TO_TICKS(RSSI_INTERVAL_MS));
    return;
  }
  BLESerial &s = *self;
  for (;;) {
    if (s.isConnected()) {
      s.adjustLink();
    }
    vTaskDelay(pdMS_TO_TICKS(RSSI_INTERVAL_MS));
  }
}

void BLESerial::pumpTxTask(void *arg) {
  // Lightweight TX pump task using direct-to-task notifications for efficient wakeups.

  BLESerial *self = static_cast<BLESerial *>(arg);
  if (!self) {
    // If no instance provided, nothing to pump
    vTaskDelay(pdMS_TO_TICKS(200));
    return;
  }

  BLESerial &s = *self;

  for (;;) {

    // Block until notified or timeout for periodic check
    if (s.isConnected()) {
      ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(5));
    } else {
      ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(200));
    }

    if (!s.isSubscribed() || s.txChar == nullptr) {
      continue; // no active link
    }

    // Drain while data/pending
    while (s.isSubscribed() && s.txChar != nullptr) {

      TX_CRITICAL_ENTER(&s);
      if (s.txOk) s.txOk = false; // clear success latch
      // Ensure producers can resume once buffer dropped below lowWater, even without a write()
      if (s.txBuf.available() <= s.lowWater) s.txLocked = false;
      TX_CRITICAL_EXIT(&s);

      // Debug invariant guard: txPending should imply pendingLen>0
      // REMOVE THIS CODE IN PRODUCTION BUILDS
      if (s.txPending && s.pendingLen == 0) {
        if (s.logLevel >= WARNING) Serial.println("BLESerial DEBUG: txPending true while pendingLen==0; clearing txPending.");
        TX_CRITICAL_ENTER(&s);
        s.txPending = false;
        TX_CRITICAL_EXIT(&s);
      }

      // If nothing staged and buffer empty, exit to wait for next notify
      bool empty = false;
      TX_CRITICAL_ENTER(&s); // protect pendingLen and txBuf
      empty = (s.txBuf.available() == 0 && s.pendingLen == 0);
      TX_CRITICAL_EXIT(&s);
      if (empty) break;

      // Respect sendIntervalUs
      uint32_t now = micros();
      uint32_t elapsed = (uint32_t)(now - s.lastTxUs);
      if (elapsed < s.sendIntervalUs) {
        uint32_t remainUs = s.sendIntervalUs - elapsed;
        // Diagnostic: if we are waiting but have an EAPP retry pending, log timing context
        if (remainUs < TASK_DELAY_THRESHOLD_US) {
          // Sub-ms gap: micro sleep (cooperative)
          delayMicroseconds(remainUs);
        } else {
          // Convert to ticks (ceil)
          vTaskDelay(pdMS_TO_TICKS((remainUs + 999) / 1000));
        }
        continue; // recheck after wait
      }

      // Stage next chunk only if no chunk currently staged (gate solely on pendingLen in pop model)
      if (s.pendingLen == 0) {
        if (!s.stageTx()) {
        // Staging failed (renegotiation); short yield then retry. Add a tiny delay to avoid hot-looping.
        taskYIELD();
        vTaskDelay(pdMS_TO_TICKS(1));
        continue;
        }
      }
      // Guard: only stamp lastTxUs when we've increased (slower) the active interval.
      // if (s.sendIntervalUs >= s.lkgIntervalUs) {
      //   s.lastTxUs = micros();
      // }
      s.lastTxUs = micros();

      // Optional cooperative yield after each send (remove if not needed)
      // taskYIELD();
    } // end while connected/subscribed
  } // end for (;;
} // end pumpTxTask

void BLESerial::setPumpMode(PumpMode m) {
  // If unchanged still allow waking when already connected & switching to Task
  const bool modeChanged = (pumpMode != m);
  pumpMode = m;

  if (pumpMode == PumpMode::Task) {
    startTxTask();
    startRSSITask();
    // If already connected/subscribed ensure tasks run
    if (isConnected()) {
      wakeTxTask();
      wakeRSSITask();
    }
  } else {
    stopTxTask();
    stopRSSITask();
  }
}

void BLESerial::startRSSITask() {
    // Create RSSI task (suspended until connect)
  if (!BLESerial::rssiTaskHandle) {
    BaseType_t rc = xTaskCreatePinnedToCore(
        BLESerial::RSSITask,
        "RSSITask",
        3072,
        this,
        2,
        &rssiTaskHandle,
        1);
    if (rc != pdPASS) {
      rssiTaskHandle = nullptr;
      if (logLevel >= WARNING) {
        Serial.printf("BLESerial: RSSITask creation failed rc=%d.\r\n", (int)rc);
      }
    } else {
      vTaskSuspend(rssiTaskHandle);
      if (logLevel >= INFO)
        Serial.println(
          "BLESerial: RSSI task created (suspended).");
    }
  }
}

void BLESerial::startTxTask() {
  if (!txTaskHandle){
    BaseType_t rc = xTaskCreatePinnedToCore(
      BLESerial::pumpTxTask,
      "BLETxPump",
      2304,
      this,
      1,
      &txTaskHandle,
      1);
    if (rc != pdPASS) {
      txTaskHandle = nullptr;
      if (logLevel >= WARNING)
        Serial.println(
          "BLESerial: TX task creation failed; "
          "staying in polling mode.");
    } else {
      // Keep task in a mostly idle state until notified or connected
      vTaskSuspend(txTaskHandle); // begin asleep as requested
      if (logLevel >= INFO)
        Serial.println(
          "BLESerial: TX task created (suspended).");
    }
  }
}

void BLESerial::stopRSSITask() {
  if (!rssiTaskHandle) return;
  vTaskDelete(rssiTaskHandle);
  rssiTaskHandle = nullptr;
  if (logLevel >= INFO)
    Serial.println("BLESerial: RSSI task stopped.");
}

void BLESerial::stopTxTask() {
  if (!txTaskHandle) return;
  vTaskDelete(txTaskHandle);
  txTaskHandle = nullptr;
  if (logLevel >= INFO)
    Serial.println("BLESerial: TX task stopped.");
}

void BLESerial::wakeTxTask() {
  // Direct-to-task notification is very light-weight (few CPU cycles)
  if (!txTaskHandle) return;
  // If task is suspended, resume it first so it can process notifications
  eTaskState st = eTaskGetState(txTaskHandle);
  if (st == eSuspended) {
    vTaskResume(txTaskHandle);
  }
  xTaskNotifyGive(txTaskHandle);
}

void BLESerial::wakeRSSITask() {
  if (!rssiTaskHandle) return;
  // RSSI task does not wait on notifications; ensure it's running
  eTaskState st = eTaskGetState(rssiTaskHandle);
  if (st == eSuspended) {
    vTaskResume(rssiTaskHandle);
  }
}

void BLESerial::suspendRSSITask() {
  if (rssiTaskHandle) {
    vTaskSuspend(rssiTaskHandle);
  }
}

void BLESerial::suspendTxTask() {
  if (txTaskHandle) {
    vTaskSuspend(txTaskHandle);
  }
}

#else
  void BLESerial::RSSITask(void * /*arg*/) {}
  void BLESerial::pumpTxTask(void * /*arg*/) {}
  void BLESerial::wakeTxTask() {}
  void BLESerial::wakeRSSITask() {}
  void BLESerial::suspendTxTask() {}
  void BLESerial::suspendRSSITask() {}
  void BLESerial::stopTxTask() {}
  void BLESerial::stopRSSITask() {}
  void BLESerial::startTxTask() {}
  void BLESerial::startRSSITask() {}
#endif

// ===== BLESerial adjustLink() ======================================================================

void BLESerial::adjustLink() {
  /*
  Adjust the link layer parameters (PHY, coded scheme) based on RSSI.

  This might not work as it would require disconnect and reconnect to change PHY
  */

  if (!isConnected()) return;

  lastRSSIMs = millis();
  int8_t val = 0;
  if (ble_gap_conn_rssi(connHandle, &val) != 0) {
    return; // read failed; ignore
  }
  rssiRaw = val;
  // Simple EMA: weight new sample 1/5
  if (rssiAvg == 0)
    rssiAvg = rssiRaw;
  else
    rssiAvg = (int8_t)((4 * (int)rssiAvg + (int)rssiRaw) / 5);

  // Cooldown before any further link adaptation
  if ((millis() - lastRSSIActionMs) < RSSI_ACTION_COOLDOWN_MS)
    return;

  // Decide target PHY / coded scheme
  uint8_t newDesiredCodedScheme = 0;
  uint8_t newDesiredPhyMask = BLE_GAP_LE_PHY_1M_MASK;

  if        (rssiAvg <= (RSSI_S8_THRESHOLD + RSSI_HYSTERESIS)) {
    newDesiredCodedScheme = 8;
  } else if (rssiAvg <= (RSSI_S2_THRESHOLD + RSSI_HYSTERESIS)) {
    newDesiredCodedScheme = 2;
  } else if (rssiAvg > (RSSI_FAST_THRESHOLD - RSSI_HYSTERESIS)) {
    newDesiredPhyMask = BLE_GAP_LE_PHY_2M_MASK;
  } // else stay 1M

  // Evaluate change necessity
  bool change = false;
  if (newDesiredCodedScheme > 0) {
    if (!phyIsCoded || codedScheme != newDesiredCodedScheme) {
      change = true;
    }
  } else if (newDesiredPhyMask == BLE_GAP_LE_PHY_2M_MASK)
  {
    if (!phyIs2M || phyIsCoded) {
      change = true;
    }
  }
  else {
    // Want 1M
    if (phyIs2M || phyIsCoded) {
      change = true;
    }
  }

  if (!change) return;

  // Apply PHY preference
  int rc = 0;
  desiredPhyMask = (newDesiredCodedScheme > 0) ? BLE_GAP_LE_PHY_CODED_MASK : newDesiredPhyMask;
  desiredCodedScheme = newDesiredCodedScheme;
  if (desiredCodedScheme > 0) {
    // renegotiate coded PHY with selected scheme
    rc = ble_gap_set_prefered_le_phy(
          connHandle,
          BLE_GAP_LE_PHY_CODED_MASK,
          BLE_GAP_LE_PHY_CODED_MASK,
          (desiredCodedScheme == 2 ? BLE_GAP_LE_PHY_CODED_S2 : BLE_GAP_LE_PHY_CODED_S8));
  } else if (desiredPhyMask == BLE_GAP_LE_PHY_2M_MASK) {
    // 2M
    desiredCodedScheme = 0;
    rc = ble_gap_set_prefered_le_phy(
        connHandle,
        BLE_GAP_LE_PHY_2M_MASK,
        BLE_GAP_LE_PHY_2M_MASK,
        desiredCodedScheme);
  } else { 
    // 1M
    desiredCodedScheme = 0;
    rc = ble_gap_set_prefered_le_phy(
        connHandle,
        BLE_GAP_LE_PHY_1M_MASK,
        BLE_GAP_LE_PHY_1M_MASK,
        desiredCodedScheme);
  }

  if (rc == 0) {
    lastRSSIActionMs = millis();
    if (logLevel >= INFO) {
      const char *target = (desiredCodedScheme ? (desiredCodedScheme == 2 ? "CODED(S2)" : "CODED(S8)")
                                               : (desiredPhyMask == BLE_GAP_LE_PHY_2M_MASK ? "2M" : "1M"));
      Serial.printf(
        "BLESerial: RSSI adapt: avg=%d raw=%d -> %s.\r\n", 
        rssiAvg, rssiRaw, target);
    }
  } else {
    if (logLevel >= WARNING) {
      Serial.printf(
        "BLESerial: PHY adapt failed (rc=%d).\r\n", rc);
    }
  }
} // end RSSI Link Adjust =======================================================================
