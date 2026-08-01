// ****************************************************************************************************
// BLE Serial Library
//
// BLE Serial Communication for Arduino using NimBLE
// This creates a Nordic UART Service (NUS) allowing to send and receive serial data over BLE
// ****************************************************************************************************
// This code is maintained by
// Urs Utzinger
// November/December 2025
// ****************************************************************************************************
#include <algorithm>
#include <cctype>
#include <cstdarg> 
#include <new>
#include "BLESerial.h"

#ifdef ARDUINO_ARCH_ESP32
  TaskHandle_t BLESerial::rssiTaskHandle = nullptr;
  TaskHandle_t BLESerial::txTaskHandle   = nullptr;
#endif

// === Code Names ===
static const char *hsCodeName(int code) {
  switch (code) {
    case 0:      return "OK(0)";
    case 1:      return "EAGAIN(1)";
    case 2:      return "EALREADY(2)";
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
    case 14:     return "EDONE(14)";
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
    default:     return "UNKNOWN"; 
  }
} // end of hsCodeName

static const char *pairingPolicyName(PairingPolicy policy) {
  switch (policy) {
    case PairingPolicy::AlwaysOpen: return "AlwaysOpen";
    case PairingPolicy::Windowed:   return "Windowed";
    case PairingPolicy::BondedOnly: return "BondedOnly";
    default:                        return "Unknown";
  }
}


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

    s.expirePairingWindowIfNeeded();

    // Peer address as string
    s.peerAddr        = connInfo.getAddress().toString();

    s.deviceConnected = true;
    s.connHandle      = connInfo.getConnHandle();
    s.linkEncrypted   = connInfo.isEncrypted();
    s.mtu             = connInfo.getMTU();
    s.probing         = false;
    s.emsgSizeRetries = 0;
    s.discardStreak   = 0;
    s.successStreak   = 0;
    s.timeoutRetries  = 0;
    s.lkgFails        = 0;
    s.recentlyBackedOff = false;
    s.coolDowns       = 0;
    s.probeSuccesses  = 0;

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

    // Capture conn params and compute per-event share
    s.connItvlUnits        = connInfo.getConnInterval();
    s.connLatency          = connInfo.getConnLatency();
    s.supervisionTimeoutMS = static_cast<uint16_t>(connInfo.getConnTimeout() * 10u);
    s.connIntervalUs       = intvl_us(s.connItvlUnits);
    s.perEventShareUs      = s.computePerEventShareUs(
                                s.connIntervalUs,
                                s.connLatency,
                                PDUS_PER_WINDOW);

    s.updateTxTiming();
    // computes txChunkSize
    // minSendIntervalUS
    // low & high Water
    // lkgIntervalUs
    // sendIntervalUs will be >= minSendIntervalUS
    // probing/backoff state reset    

    LOGI("BLESerial: Connected: interval=%.2f ms, latency=%u, perEventShare=%lu µs",
         s.connItvlUnits * 1.25f, s.connLatency, (unsigned long)s.perEventShareUs);

    if (!s.shouldAcceptPeer(connInfo)) {
      LOGW(
        "BLESerial: Rejecting unbonded peer %s "
        "(policy=%s pairing_window=%s).",
        s.peerAddr.c_str(),
        pairingPolicyName(s.pairingPolicy),
        s.isPairingWindowOpen() ? "open" : "closed");
      srv->disconnect(s.connHandle);
      return;
    }

    if (s.secure == Security::PasskeyDisplay && !connInfo.isBonded()) {
      s.refreshPasskey();
    }

    // Start security if enabled
    if (s.secure != Security::None) NimBLEDevice::startSecurity(s.connHandle);

    // Resume Tasks
    #ifdef ARDUINO_ARCH_ESP32
      s.wakeTxTask();
      s.wakeRSSITask();
    #endif

    LOGI("BLESerial: Connected %s PHY=%s.",
         s.peerAddr.c_str(),
         s.phyToStr());
    LOGI("BLESerial: chunk=%u send_interval=%uµs min_send_interval=%uµs.",
         (unsigned)s.txChunkSize, (unsigned)s.sendIntervalUs, (unsigned)s.minSendIntervalUS);
    LOGI("BLESerial: llTx: octets=%u time=%uµs | llRx: octets=%u time=%uµs.",
         (unsigned)s.llTxOctets, (unsigned)s.llTxTimeUs,
         (unsigned)s.llRxOctets, (unsigned)s.llRxTimeUs);
    if (s.onClientConnect) s.onClientConnect(s.peerAddr); // user provided addon callback
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
    s.lkgFails          = 0;
    s.recentlyBackedOff = false;
    s.coolDowns         = 0;
    s.successStreak     = 0;
    s.emsgSizeRetries   = 0;
    s.discardStreak     = 0;

    s.connItvlUnits     = 0;
    s.connLatency       = 0;
    s.connIntervalUs    = 0;
    s.perEventShareUs   = 0;
    s.mtu               = BLE_SERIAL_MIN_MTU;

    // Reset MTU/chunk/EBADDATA counters; drop any staged frame
    TX_CRITICAL_ENTER(&s);
    s.txSuccess         = TxSuccess::NotSet;
    s.pendingLen        = 0;
    s.lastTxUs          = 0;
    TX_CRITICAL_EXIT(&s);

    s.updateTxTiming();
    // computes txChunkSize
    // minSendIntervalUS
    // low & high Water
    // lkgIntervalUs
    // sendIntervalUs will be >= minSendIntervalUS
    // probing/backoff state reset    

    s.expirePairingWindowIfNeeded();
    s.applyPairingPolicy(false);

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

    const uint8_t hci = static_cast<uint8_t>(reason & 0xFF);
    LOGI(
      "BLESerial: Client [%s] disconnected "
      "(reason=%u %s). Advertising restarted.",
      connInfo.getAddress().toString().c_str(),
      hci, hciDisconnectReasonStr(hci));
    if (s.onClientDisconnect) {
      s.onClientDisconnect(connInfo.getAddress().toString(), hci); // user provided addon callback
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

    LOGI(
      "BLESerial: MTU=%u (conn=%u), "
      "tx_chunk_size=%u, min_send_interval=%uµs.",
      m, connInfo.getConnHandle(),
      s.txChunkSize, (unsigned)s.minSendIntervalUS);
    if (s.onMtuChanged) s.onMtuChanged(m); // user provided addon callback
  }

  // NOT AVAILABLE IN THIS VERSION OF NIMBLE (left for future)
 
  // // Generate and return a random 6-digit passkey (000000–999999)
  // uint32_t onPassKeyRequest() override {
  //   if (!owner) return 0;
  //   auto& s = *owner;

  //   // Generate random 6-digit code; ensure leading zeros possible on display
  //   uint32_t key = (uint32_t)random(0UL, 1000000UL);
  //   s.passkey = key;

  //   LOGI("BLESerial: Server Passkey Request: %06u", key);
  //   return key;
  // }

  // Display callback (called to present the passkey to the user)
  uint32_t onPassKeyDisplay() override {
    if (!owner) return 0;
    auto &s = *owner;

    LOGI("BLESerial: Server Passkey: %06u.", s.passkey);
    return s.passkey;
  }

  // Confirm the passkey shown/entered by the peer
  void onConfirmPassKey(NimBLEConnInfo &connInfo, uint32_t peerKey) override {
    if (!owner) return;
    auto &s = *owner;

    bool match = (peerKey == s.passkey);
    NimBLEDevice::injectConfirmPasskey(connInfo, match);

    LOGI(
      "BLESerial: Confirm Passkey: "
      "local=%06u peer=%06u %s.",
      s.passkey, peerKey,
      match ? "MATCH" : "MISMATCH");
  }

  void onAuthenticationComplete(NimBLEConnInfo &connInfo) override {
    if (!owner) return;
    auto &s = *owner;

    if (!connInfo.isEncrypted()) {
      s.linkEncrypted = false;
      NimBLEDevice::getServer()->disconnect(connInfo.getConnHandle());
      LOGW("BLESerial: Encrypt connection failed - disconnecting client.");
      return;
    }

    s.linkEncrypted = true;

    LOGI("BLESerial: Secured connection to: %s.",
         connInfo.getAddress().toString().c_str());
  }

  void onConnParamsUpdate(NimBLEConnInfo &connInfo) override {
    if (!owner)return;
    auto &s = *owner;

    s.connItvlUnits        = connInfo.getConnInterval();
    s.connLatency          = connInfo.getConnLatency();
    s.supervisionTimeoutMS = static_cast<uint16_t>(connInfo.getConnTimeout() * 10u);
    s.connIntervalUs       = intvl_us(s.connItvlUnits);
    s.perEventShareUs      = s.computePerEventShareUs(
                              s.connIntervalUs,
                              s.connLatency,
                              PDUS_PER_WINDOW);
                    
    s.updateTxTiming();

    LOGI("BLESerial: Conn params updated: interval=%.2f ms, latency=%u, perEventShare=%lu µs",
         s.connItvlUnits * 1.25f, s.connLatency, (unsigned long)s.perEventShareUs);
  }

  void onPhyUpdate(NimBLEConnInfo &connInfo, uint8_t txPhy, uint8_t rxPhy) override {
    if (!owner) return;
    // Duplicate PHY logging suppressed; GAP event handler provides authoritative update.
    // (Intentionally left silent to avoid log spam.)
    (void)txPhy; (void)rxPhy; (void)connInfo;
  }

  void onIdentity(NimBLEConnInfo &connInfo) override {
    if (!owner) return;
    LOGI("BLESerial: Identity resolved: %s.",
         connInfo.getAddress().toString().c_str());
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

    size_t pushed = 0;
    size_t lost = 0;
    RX_CRITICAL_ENTER(&s);
    // RingBuffer returns retained input bytes in overwrite mode, not bytes evicted from the ring.
    const size_t capacity = s.rxBuf.capacity();
    const size_t used = s.rxBuf.available();
    const size_t retained = std::min(len, capacity);
    const size_t free = capacity - used;
    const size_t overwritten = (retained > free) ? retained - free : 0;
    lost = (len - retained) + overwritten;
    pushed = s.rxBuf.push(data, len, true); // overwrite oldest if full

    // RX accounting is shared with the sketch-facing statistics accessors.
    s.bytesRx = s.bytesRx + len;
    if (lost) {
      s.rxDrops = s.rxDrops + lost;
    }
    s.lastRxUs = micros();
    RX_CRITICAL_EXIT(&s);

    // RX accounting includes all received bytes and all bytes the ring could not retain.
    if (lost) {
      if (s.onRxOverflow) s.onRxOverflow(lost); // optional user provided callback
    }

    // data points into the characteristic value and is valid only until that value is cleared.
    if (s.onDataReceived && pushed) s.onDataReceived(data, len); // optional user provided callback

    // Clear last value held by the characteristic to free heap.
    ch->setValue(nullptr, 0);
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
  // ============================================================================================
  void onStatus(NimBLECharacteristic *ch, int code) override {
  /*
    When code is not success or done: 
      payload did not enter controller mbuf queue
      we need to resend chunk
      errors do not result in partial data sent
     
    Status codes:
      0                       → Success (notification queued/sent).
      14 (BLE_HS_EDONE)       → Success for indication (confirmation received).
      1  (BLE_HS_EAGAIN)      → Temporary failure, retry later.
      2  (BLE_HS_EALREADY)    → Operation already in progress or completed.
      3  (BLE_HS_EINVAL)      → Invalid parameters.
      4  (BLE_HS_EMSGSIZE)    → Payload too big for context. (For notifies you should already be ≤ MTU−3.)
      5  (BLE_HS_ENOENT)      → No such entry.
      6  (BLE_HS_ENOMEM)      → Out of buffers / resource exhaustion. You’re sending faster than the stack can drain, or mbufs are tight. Back off or throttle.
      7  (BLE_HS_ENOTCONN)    → No open connection with the specified handle.
      8  (BLE_HS_ENOTSUP)     → Not supported.
      9  (BLE_HS_EAPP)        → Application error, callback behaved unexpectedly.
      10 (BLE_HS_EBADDATA)    → Malformed data. Command from peer is invalid.
      11 (BLE_HS_EOS)         → Mynewt operating system error.
      12 (BLE_HS_ECONTROLLER) → Controller error, event invalid..
      13 (BLE_HS_ETIMEOUT)    → Operation timed out.
      15 (BLE_HS_EBUSY)       → Another LL/GATT procedure is in progress; try again later.
      16 (BLE_HS_EREJECT)     → Operation rejected.
      17 (BLE_HS_EUNKNOWN)    → Unknown error.
      18 (BLE_HS_EROLE)       → Role error.
      19 (BLE_HS_ETIMEOUT_HCI)→ HCI request timed out; controller unresponsive.
      20 (BLE_HS_ENOMEM_EVT)  → Controller failed to send event due to memory exhaustion.
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

      === Success ===
      code == 0 || code == BLE_HS_EDONE

      === Payload-related ===
      Buffer too small / chunk too big.
      -> modify payload before retrying.
      code == BLE_HS_EMSGSIZE);

      === Retryable "backpressure" / congestion ===
      temporary resource exhaustion
      -> retry same payload later with backoff.
      code == BLE_HS_EAGAIN    ||  // temporary; try again
      code == BLE_HS_EALREADY  ||  // other proc already in progress
      code == BLE_HS_EBUSY     ||  // cannot perform until proc completes
      code == BLE_HS_ENOMEM    ||  // host out of mbufs
      code == BLE_HS_ENOMEM_EVT);  // controller couldn’t send event

      === Timeouts (maybe retry a few times, then treat as fatal) ===
      -> retry a few times, then treat as fatal
      code == BLE_HS_ETIMEOUT ||
      code == BLE_HS_ETIMEOUT_HCI);

      === Link gone / OS-level fatal ===
      teardown connection, clean up
      -> do not retry
      code == BLE_HS_ENOTCONN ||  // link closed
      code == BLE_HS_EOS);        // Mynewt OS error
  
      === Local bug / invalid state / non-retryable ===
      -> do not retry
      software bug / invalid state
      code == BLE_HS_EINVAL      ||  // bad args / state
      code == BLE_HS_EAPP        ||  // your callback misbehaved
      code == BLE_HS_EBADDATA    ||  // peer command invalid (protocol bad)
      code == BLE_HS_ECONTROLLER ||  // controller event invalid
      code == BLE_HS_EUNKNOWN);      // catch-all unexpected
  */

    if (!owner) return;
    auto &s = *owner;

    TX_CRITICAL_ENTER(&s);
    s.onStatusCode = code;
    if (isOkOrDone(code)) {
      s.txSuccessCount = s.txSuccessCount + 1u;
      s.txSuccess = TxSuccess::Success;
    } else if (isMsgSize(code)) {
      s.msgSizeCount = s.msgSizeCount + 1u;
      s.txSuccess = TxSuccess::MessageSizeTooBig;
    } else if (isCongestion(code)) {
      s.congestionCount = s.congestionCount + 1u;
      s.txSuccess = TxSuccess::Congestion;
    } else if (isTimeout(code)) {
      s.timeoutCount = s.timeoutCount + 1u;
      s.txSuccess = TxSuccess::Timeout;
    } else if (isDisconnectedOrEOS(code)) {
      s.disconnectCount = s.disconnectCount + 1u;
      s.txSuccess = TxSuccess::Disconnected;
    } else if (isSoftwareError(code)){
      s.softwareErrorCount = s.softwareErrorCount + 1u;
      s.txSuccess = TxSuccess::SoftwareError;
    } else {
      s.unclassifiedCount = s.unclassifiedCount + 1u;
      s.txSuccess = TxSuccess::Unclassified;
    }
    TX_CRITICAL_EXIT(&s);
  }
  // end of onStatus ---------------------------------------------------------------------------------
  // ============================================================================================

  // After subscription status changes
  // ============================================================================================
  void onSubscribe(NimBLECharacteristic *ch, NimBLEConnInfo &connInfo, uint16_t subValue) override {
    if (!owner) return;
    auto &s = *owner;

    bool notify        = (subValue & 0x0001);
    bool indicate      = (subValue & 0x0002);
    s.clientSubscribed = notify || indicate;

    if (logGetLevel() <= LOG_LEVEL_INFO) {
      std::string uuid = ch->getUUID().toString();
      if (subValue == 0)
        LOGI("BLESerial: Client %s unsubscribed %s.",
             s.peerAddr.c_str(), uuid.c_str());
      else
        LOGI(
          "BLESerial: Client %s subscribed (%s%s) %s.",
          s.peerAddr.c_str(),
          notify ? "notify" : "",
          indicate ? (notify ? "+indicate" : "indicate") : "",
          uuid.c_str());
    }
    if (s.onSubscribeChanged)  s.onSubscribeChanged(s.clientSubscribed); // user provided addon callback
  }
  // end of onSubscribe -------------------------------------------------------------------------------------
  // ============================================================================================

private:
  BLESerial *owner;

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
  static inline bool isCongestion(int code) {
    return (code == BLE_HS_EAGAIN    ||  // temporary; try again
            code == BLE_HS_EALREADY  ||  // other proc already in progress
            code == BLE_HS_EBUSY     ||  // cannot perform until proc completes
            code == BLE_HS_ENOMEM    ||  // host out of mbufs
            code == BLE_HS_ENOMEM_EVT);  // controller couldn’t send event
  }

  // === Link gone / OS-level fatal ===
  // -> do not retry
  // teardown connection, clean up
  static inline bool isDisconnectedOrEOS(int code) {
    return (code == BLE_HS_ENOTCONN ||  // link closed
            code == BLE_HS_EOS);       // Mynewt OS error
  }

  // === Timeouts (maybe retry a few times, then treat as fatal) ===
  // -> retry a few times, then treat as fatal
  static inline bool isTimeout(int code) {
    return (code == BLE_HS_ETIMEOUT ||
            code == BLE_HS_ETIMEOUT_HCI);
  }

  // === Local bug / invalid state / non-retryable ===
  // -> do not retry
  // software bug / invalid state
  static inline bool isSoftwareError(int code) {
    return (code == BLE_HS_EINVAL      ||  // bad args / state
            code == BLE_HS_EAPP        ||  // your callback misbehaved
            code == BLE_HS_EBADDATA    ||  // peer command invalid (protocol bad)
            code == BLE_HS_ECONTROLLER ||  // controller event invalid
            code == BLE_HS_EUNKNOWN);      // catch-all unexpected
  }
}; // end of TxCallbacks ========================================================================

// ==============================================================================================
// ==============================================================================================

// ===== Additional GAP event handler ===========================================================
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
        LOGD("BLESerial: PHY unchanged (GAP):");
        LOGD(
          "BLESerial: tx=%u, rx=%u, (%s), llTxTime=%uµs, tx_chunk_size=%u, min_send_interval=%uµs.",
          p.tx_phy, p.rx_phy, s.phyToStr(),
          (unsigned)s.llTxTimeUs,
          (unsigned)s.txChunkSize,
          (unsigned)s.minSendIntervalUS);
        return 0;
      }

      s.updateTxTiming();
      // computes txChunkSize
      // minSendIntervalUS
      // low & high Water
      // lkgIntervalUs
      // sendIntervalUs
      // probing/backoff state reset

      LOGI("BLESerial: PHY updated (GAP):");
      LOGI(
        "BLESerial: tx=%u, rx=%u, (%s), llTxTime=%uµs, tx_chunk_size=%u, min_send_interval=%uµs.",
        p.tx_phy, p.rx_phy, s.phyToStr(),
        (unsigned)s.llTxTimeUs,
        (unsigned)s.txChunkSize,
        (unsigned)s.minSendIntervalUS);

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
    //   // LOGI(
    //   //   "BLESerial: L2CAP DLE proposal: "
    //   //   "tx_octets=%u tx_time=%uµs rx_octets=%u rx_time=%uµs",
    //   //   (unsigned)q.tx_octets, (unsigned)q.tx_time,
    //   //   (unsigned)q.rx_octets, (unsigned)q.rx_time);
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

      if (logGetLevel() <= LOG_LEVEL_INFO) {
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

        LOGI("BLESerial: DLE updated:");
        LOGI("BLESerial: tx %u->%u octets, %u->%uµs; mtu=%u",
             (unsigned)oldTxOctets, (unsigned)s.llTxOctets,
             (unsigned)oldTxTimeUs, (unsigned)s.llTxTimeUs,
             (unsigned)s.mtu);
        LOGI("BLESerial: max_att_payload=%u, max_one_PDU_payload=%u",
             (unsigned)attPayloadMax, (unsigned)onePduMaxPayload);
        LOGI("BLESerial: sdu_size=%u, sdu_bytes_per_full_ll_pdu=%u, fits_single_LL_PDU=%s",
             (unsigned)sduBytes, (unsigned)M,
             fitsSingle ? "YES" : "NO");
        LOGI("BLESerial: tx_chunk_size=%u, min_send_interval=%uµs.",
             (unsigned)s.txChunkSize, (unsigned)s.minSendIntervalUS);
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

      LOGI(
        "BLESerial: MTU=%u, tx_chunk_size=%u, min_send_interval=%uµs.",
        (unsigned)mtu, (unsigned)s.txChunkSize, (unsigned)s.minSendIntervalUS);
      return 0;
    }

    case BLE_GAP_EVENT_CONN_UPDATE:
    {
      // const auto &cu = ev->conn_update;
      // if (cu.status != 0) {
      //     // update rejected or failed; keep old values
      //     return 0;
      // }

      // // Store connection parameters
      // s.connItvlUnits        = cu.conn_itvl;
      // s.connLatency          = cu.conn_latency;
      // s.supervisionTimeoutMS = cu.supervision_timeout * 10u; // units of 10ms -> ms
      // s.connIntervalUs       = static_cast<uint32_t>(cu.conn_itvl) * 1250u; // 1.25ms units -> us
      //
      // s.perEventShareUs = computePerEventShareUs(
      //   s.connIntervalUs,
      //   s.connLatency,
      //   PDUS_PER_WINDOW);

      // // s.updateTxTiming();

      // LOGI("BLESerial: Connection parameters updated (GAP):");
      // LOGI(
      //   "BLESerial: interval=%dµs, latency=%u, timeout=%ums.",
      //   s.connIntervalUs,
      //   s.connLatency,
      //   s.supervisionTimeoutMS);
      // Some NimBLE builds don’t expose conn_itvl/conn_latency/supervision_timeout here.
      // We already log updates in ServerCallbacks::onConnParamsUpdate(). No-op.
      (void)ev;
      return 0;
    }

    // case BLE_GAP_EVENT_CONNECT:
    // case BLE_GAP_EVENT_DISCONNECT:
    // case BLE_GAP_EVENT_SUBSCRIBE:
    // case BLE_GAP_EVENT_NOTIFY_TX:
    // case BLE_GAP_EVENT_NOTIFY_RX:
    // case BLE_GAP_EVENT_ADV_COMPLETE:
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
  const auto failBegin = [this]() {
    cleanupAfterBeginFailure();
    return false;
  };

  // Minimal init; full feature set can be added incrementally
  mode = newMode;
  secure = newSecure;
  BLESerial::active = this; // allow static GAP handler to reach our instance

  // Decide desired link behavior from mode (desired != current)
  int8_t dBmAdv, dBmScan, dBmConn;
  uint16_t modePreferredMtu;

  switch (mode)
  {
  case Mode::Fast:
    modePreferredMtu    = BLE_SERIAL_MAX_MTU;
    desiredPhyMask      = BLE_GAP_LE_PHY_2M_MASK;
    desiredCodedScheme  = 0;
    desiredllTxOctets   = LL_MAX_OCTETS;
    desiredllTxTimeUs   = estimate_LL_PDUTimeUs(desiredllTxOctets, true, false, desiredCodedScheme);
    dBmAdv              = BLE_TX_DBP9;
    dBmScan             = BLE_TX_DBP9;
    dBmConn             = BLE_TX_DBP9;
    break;
  case Mode::LowPower:
    modePreferredMtu    = BLE_SERIAL_MIN_MTU;
    desiredPhyMask      = BLE_GAP_LE_PHY_1M_MASK;
    desiredCodedScheme  = 0;
    desiredllTxOctets   = LL_MAX_OCTETS;
    desiredllTxTimeUs   = estimate_LL_PDUTimeUs(desiredllTxOctets, false, false, desiredCodedScheme);
    dBmAdv              = BLE_TX_DBN9;
    dBmScan             = BLE_TX_DBN9;
    dBmConn             = BLE_TX_DBN6;
    break;
  case Mode::LongRange:
    modePreferredMtu    = BLE_SERIAL_DEFAULT_MTU;
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
    modePreferredMtu    = BLE_SERIAL_DEFAULT_MTU;
    desiredPhyMask      = BLE_GAP_LE_PHY_1M_MASK;
    desiredCodedScheme  = 0;
    desiredllTxOctets   = LL_MAX_OCTETS;
    desiredllTxTimeUs   = estimate_LL_PDUTimeUs(desiredllTxOctets, false, false, desiredCodedScheme);
    dBmAdv              = BLE_TX_DBN6;
    dBmScan             = BLE_TX_DBN3;
    dBmConn             = BLE_TX_DB0;
    break;
  }

  if (!preferredMtuConfigured) {
    preferredMtu = modePreferredMtu;
  }
  mtu = BLE_SERIAL_MIN_MTU;

  // Current, negotiated state is unknown pre-connection
  phyIs2M               = false;
  phyIsCoded            = false;
  codedScheme           = 0;

  // BLE: init stack, create service, start adv; UART: config UART
  NimBLEDevice::init(deviceName);
  LOGI("BLESerial: Device created with name %s.", deviceName);
  NimBLEDevice::setCustomGapHandler(&BLESerial::gapEventHandler);
  LOGI("BLESerial: Custom Gap handler set.");
  if (!NimBLEDevice::setMTU(preferredMtu)) {
    LOGW("BLESerial: Could not set preferred MTU to %u.", preferredMtu);
  } else {
    LOGI("BLESerial: Preferred MTU set to %u.", preferredMtu);
  }

  NimBLEDevice::setPower(dBmAdv, PWR_ADV);
  NimBLEDevice::setPower(dBmScan, PWR_SCAN);
  NimBLEDevice::setPower(dBmConn, PWR_CONN);
  LOGI("BLESerial: Power levels set: Adv=%d, Scan=%d, Conn=%d.", dBmAdv, dBmScan, dBmConn);

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
    LOGI("BLESerial: Random address initialized.");
    // your client will need to reacquire the address each time you want to connect
  }
  else
  {
    NimBLEDevice::setOwnAddrType(BLE_OWN_ADDR_PUBLIC);
    LOGI("BLESerial: Public address initialized.");
    // address remains static and can be reused by the client
  }

  NimBLEDevice::setDefaultPhy(desiredPhyMask, desiredPhyMask);
  LOGI("BLESerial: Default PHY set to %u.", desiredPhyMask);

  // Suggested default data length: use safe, spec-aligned maximum (not dynamic). Typical: 251 octets, LL_DEFAULT_TIME_USµs.
  ble_gap_write_sugg_def_data_len(desiredllTxOctets, desiredllTxTimeUs);

  llTxOctets = desiredllTxOctets; // for now until negotiated at connection time
  llTxTimeUs = desiredllTxTimeUs; // for now until negotiated at connection time
  LOGI(
    "BLESerial: Suggested default data length set: "
    "%u octets, %uµs.",
    llTxOctets, llTxTimeUs);

  // Seed the PRNG before any security material is generated.
  randomSeed(((uint32_t)analogRead(0)) ^ micros());

  // Security posture
  if (secure == Security::PasskeyDisplay) {
    NimBLEDevice::setSecurityAuth(/*bonding*/ true, /*mitm*/ true, /*sc*/ true);

    // IO capability: display only (ESP_IO_CAP_OUT)
    NimBLEDevice::setSecurityIOCap(BLE_HS_IO_DISPLAY_ONLY); /** Display only passkey */

    // Key distribution (init/rsp) ~ ESP_BLE_SSET_INIT_KEY / SET_RSP_KEY
    NimBLEDevice::setSecurityInitKey(KEYDIST_ENC | KEYDIST_ID);
    NimBLEDevice::setSecurityRespKey(KEYDIST_ENC | KEYDIST_ID);
    LOGI("BLESerial: Secure passkey connection initialized.");
  } else if (secure == Security::JustWorks) {
    NimBLEDevice::setSecurityAuth(/*bonding*/ true, /*mitm*/ false, /*sc*/ true);
    // IO capability: no input/output (ESP_IO_CAP_NONE)
    NimBLEDevice::setSecurityIOCap(BLE_HS_IO_NO_INPUT_OUTPUT); /** Just works */

    // Key distribution (init/rsp) ~ ESP_BLE_SSET_INIT_KEY / SET_RSP_KEY
    NimBLEDevice::setSecurityInitKey(KEYDIST_ENC | KEYDIST_ID);
    NimBLEDevice::setSecurityRespKey(KEYDIST_ENC | KEYDIST_ID);
    LOGI("BLESerial: Secure justworks connection initialized.");
  } else if (secure == Security::None) {
    NimBLEDevice::setSecurityAuth(/*bonding*/ false, /*mitm*/ false, /*sc*/ false); // no pairing needed
    LOGI("BLESerial: Insecure connection initialized.");
  }

  // Create server and service
  server = NimBLEDevice::createServer();
  if (!server) {
    LOGE("BLESerial: Server creation failed.");
    return failBegin();
  }
  server->setCallbacks(new ServerCallbacks(this));
  service = server->createService(BLE_SERIAL_SERVICE_UUID);
  if (!service) {
    LOGE("BLESerial: Service creation failed.");
    return failBegin();
  } else {
    LOGI("BLESerial: Server and Services created.");
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
    LOGI("BLESerial: Secure Rx and Tx services initialized.");

  } else {
    rxChar = service->createCharacteristic(
        BLE_SERIAL_CHARACTERISTIC_UUID_RX,
        NIMBLE_PROPERTY::WRITE |
        NIMBLE_PROPERTY::WRITE_NR // write without response (faster)
    );
    txChar = service->createCharacteristic(
        BLE_SERIAL_CHARACTERISTIC_UUID_TX,
        NIMBLE_PROPERTY::NOTIFY);
    LOGI("BLESerial: Insecure Rx and Tx services initialized.");
  }

  if (!rxChar || !txChar) {
    LOGE("BLESerial: Characteristic creation failed.");
    return failBegin();
  }

  // Set attribute max length for RX/TX characteristics.
  // GATT attribute values are limited to 512 bytes by spec; ATT payload per PDU is MTU-3.
  // Use min(512, mtu-3) as initial cap so we can accept/emit up to negotiated MTU later.
  {
    // Attribute max length is implicitly handled by NimBLE; no explicit setMaxLen API in 2.x
    // Left here for future versions supporting an explicit cap.
    (void)mtu;
  }

  // NimBLECharacteristic does not own callback objects, so retain and release them.
  txCallbacks = new (std::nothrow) TxCallbacks(this);
  rxCallbacks = new (std::nothrow) RxCallbacks(this);
  if (!txCallbacks || !rxCallbacks) {
    LOGE("BLESerial: Characteristic callback allocation failed.");
    return failBegin();
  }
  txChar->setCallbacks(txCallbacks);
  rxChar->setCallbacks(rxCallbacks);

  // NimBLE starts all registered services when the server starts advertising.
  LOGI("BLESerial: Service registered.");

  // Primary Advertising: Flags and Service UUID
  advertising = NimBLEDevice::getAdvertising();
  if (!advertising) {
    LOGE("BLESerial: Advertising creation failed.");
    return failBegin();
  }
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
  if (!advData.setFlags(BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP) ||
      !advData.addServiceUUID(BLE_SERIAL_SERVICE_UUID) ||
      !advData.addTxPower() ||
      !scanData.setName(deviceName) ||
      !scanData.setAppearance(BLE_SERIAL_APPEARANCE)) {
    LOGE("BLESerial: Advertisement data construction failed.");
    return failBegin();
  }

  // Scan Response: put the full name here (saves ADV space)
  const uint8_t mfg[] = {0xFF, 0xFF}; // 0xFFFF + 27 bytes max
  if (!scanData.setManufacturerData(std::string((const char *)mfg, sizeof(mfg))) ||
      !advertising->setAdvertisementData(advData) ||
      !advertising->setScanResponseData(scanData)) {
    LOGE("BLESerial: Advertisement configuration failed.");
    return failBegin();
  }
  expirePairingWindowIfNeeded();
  applyPairingPolicy(false);
  if (!advertising->start()) {
    LOGE("BLESerial: Advertising start failed.");
    return failBegin();
  }
  LOGI("BLESerial: Advertising started.");

  // Initialize watermarks
  updateWaterMarks(static_cast<size_t>(txChunkSize));
 
  #ifdef ARDUINO_ARCH_ESP32
    if (pumpMode == PumpMode::Task) {
      startTxTask(); // creates if absent; does NOT actively pump until notified
      if (pumpMode == PumpMode::Task) {
        startRSSITask(); // creates if absent
      }
    }
  #endif

  // Print MAC (purely informational)
  deviceMac = NimBLEDevice::getAddress().toString();
  for (char &c : deviceMac)
    c = (char)toupper((unsigned char)c);
  LOGI("BLESerial: MAC=%s.", deviceMac.c_str());
  LOGI("BLESerial: Initialization completed.");

  return true;
}

bool BLESerial::begin(Mode newMode,
                      const char *deviceName,
                      Security newSecure,
                      PairingPolicy newPairingPolicy)
{
  pairingPolicy = newPairingPolicy;
  return begin(newMode, deviceName, newSecure);
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

  // NimBLECharacteristic does not delete callbacks. Restore the default callbacks
  // before deleting our owned instances, then let NimBLE destroy the characteristics.
  if (txChar) txChar->setCallbacks(nullptr);
  if (rxChar) rxChar->setCallbacks(nullptr);
  delete txCallbacks;
  txCallbacks = nullptr;
  delete rxCallbacks;
  rxCallbacks = nullptr;

  // Release NimBLE resources (frees server/service/chars/adv objects)
  NimBLEDevice::setCustomGapHandler(nullptr);
  if (NimBLEDevice::isInitialized()) {
    NimBLEDevice::deinit(true);
  }

  // Clear pointers after deinit
  server             = nullptr;
  service            = nullptr;
  rxChar             = nullptr;
  txChar             = nullptr;
  advertising        = nullptr;
  advData             = NimBLEAdvertisementData();
  scanData            = NimBLEAdvertisementData();

  // Reset link/PHY state
  deviceConnected    = false;
  clientSubscribed   = false;
  phyIs2M            = false;
  phyIsCoded         = false;
  codedScheme        = 0;
  desiredCodedScheme = 0;
  connHandle         = BLE_HS_CONN_HANDLE_NONE;
  peerAddr.clear();
  deviceMac.clear();
  pairingWindowOpen  = false;
  pairingWindowUntilMs = 0;
  rssiRaw            = 0;
  rssiAvg            = 0;
  lastRSSIMs         = 0;
  lastRSSIActionMs   = 0;
  lastCongestionAtUs = 0;

  // Reset pacing/timing
  TX_CRITICAL_ENTER(this); // ----------------
  txState            = TxState::Waiting;
  txLocked           = false;
  txSuccess          = TxSuccess::NotSet;
  onStatusCode       = 0;
  pendingLen         = 0;
  lastTxUs           = 0;
  notifyFailedCount  = 0;
  emsgSizeRetries    = 0;
  timeoutRetries     = 0;
  softwareRetries    = 0;
  probing            = false;
  probeSuccesses     = 0;
  lkgFails           = 0;
  recentlyBackedOff  = false;
  coolDowns          = 0;
  successStreak      = 0;
  discardStreak      = 0;
  TX_CRITICAL_EXIT(this); // -----------------

  llTxOctets         = LL_MAX_OCTETS;
  llTxTimeUs         = LL_DEFAULT_TIME_US;
  llRxOctets         = LL_MAX_OCTETS;
  llRxTimeUs         = LL_DEFAULT_TIME_US;
  mtu                = BLE_SERIAL_MIN_MTU;

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
  while (rxBuf.pop(b) == 1) { /* discard */ }

  clearStats();
  RX_CRITICAL_ENTER(this);
  lastRxUs = 0;
  RX_CRITICAL_EXIT(this);

  // Reset watermarks
  updateWaterMarks(static_cast<size_t>(txChunkSize));
 
  // Detach active instance pointer
  if (BLESerial::active == this) {
    BLESerial::active = nullptr;
  }

  LOGI("BLESerial: BLE deinitialized and resources released.");
}

void BLESerial::cleanupAfterBeginFailure() {
  LOGW("BLESerial: Initialization failed; releasing partial resources.");
  end();
}

void BLESerial::setPairingPolicy(PairingPolicy policy) {
  pairingPolicy = policy;
  expirePairingWindowIfNeeded();
  applyPairingPolicy();
}

bool BLESerial::openPairingWindow(uint32_t durationMs) {
  if (durationMs == 0) {
    closePairingWindow();
    return false;
  }

  pairingWindowOpen    = true;
  pairingWindowUntilMs = millis() + durationMs;

  LOGI("BLESerial: Pairing window opened for %lu ms.",
       (unsigned long)durationMs);

  applyPairingPolicy();

  #ifdef ARDUINO_ARCH_ESP32
    if (pumpMode == PumpMode::Task) {
      startRSSITask();
      wakeRSSITask();
    }
  #endif

  return true;
}

void BLESerial::closePairingWindow() {
  const bool wasOpen = isPairingWindowOpen() || pairingWindowOpen;
  pairingWindowOpen = false;
  pairingWindowUntilMs = 0;

  if (wasOpen) {
    LOGI("BLESerial: Pairing window closed.");
  }

  applyPairingPolicy();
}

bool BLESerial::isPairingWindowOpen() const {
  if (!pairingWindowOpen) return false;
  return (int32_t)(millis() - pairingWindowUntilMs) < 0;
}

bool BLESerial::shouldAllowUnbondedPeer() const {
  if (secure == Security::None) return true;
  if (pairingPolicy == PairingPolicy::AlwaysOpen) return true;
  return isPairingWindowOpen();
}

bool BLESerial::shouldAcceptPeer(const NimBLEConnInfo &connInfo) const {
  if (secure == Security::None) return true;
  if (connInfo.isBonded()) return true;
  return shouldAllowUnbondedPeer();
}

void BLESerial::refreshPasskey() {
  passkey = (uint32_t)random(0UL, 1000000UL);
  NimBLEDevice::setSecurityPasskey(passkey);

  LOGI("BLESerial: Refreshed passkey for pairing: %06u.", passkey);
}

void BLESerial::expirePairingWindowIfNeeded() {
  if (!pairingWindowOpen) return;
  if (isPairingWindowOpen()) return;

  pairingWindowOpen = false;
  pairingWindowUntilMs = 0;

  LOGI("BLESerial: Pairing window expired.");

  applyPairingPolicy();
}

void BLESerial::syncWhiteListFromBonds() {
  if (!NimBLEDevice::isInitialized()) return;

  while (NimBLEDevice::getWhiteListCount() > 0) {
    NimBLEAddress addr = NimBLEDevice::getWhiteListAddress(0);
    if (!NimBLEDevice::whiteListRemove(addr)) {
      break;
    }
  }

  const int bondCount = NimBLEDevice::getNumBonds();
  for (int i = 0; i < bondCount; ++i) {
    NimBLEDevice::whiteListAdd(NimBLEDevice::getBondedAddress(i));
  }
}

void BLESerial::applyPairingPolicy(bool restartAdvertising) {
  if (advertising == nullptr || !NimBLEDevice::isInitialized()) return;

  const bool allowUnbonded = shouldAllowUnbondedPeer();
  if (secure == Security::None || allowUnbonded) {
    advertising->setScanFilter(false, false);
  } else {
    syncWhiteListFromBonds();
    advertising->setScanFilter(false, true);
  }

  if (!restartAdvertising || isConnected() || !advertising->isAdvertising()) return;

  advertising->stop();
  advertising->start();
}

// ===== BLESerial read/write/flush =============================================================
int BLESerial::available() {
  // Stream::available(): number of bytes that can be read without blocking
  return readAvailable();
}

int BLESerial::readAvailable() {
  RX_CRITICAL_ENTER(this);
  const int available = (int)rxBuf.available();
  RX_CRITICAL_EXIT(this);
  return available;
}

// Read one byte
int BLESerial::read() {
  // Assumes RingBuffer::pop() returns int (or -1 when empty)
  uint8_t b = 0;
  RX_CRITICAL_ENTER(this);
  const bool read = rxBuf.pop(b) == 1;
  RX_CRITICAL_EXIT(this);
  if (read)
    return (int)b;
  return -1;
}

// Read up to n bytes into dst using RingBuffer::pop(T*, n)
int BLESerial::read(uint8_t *dst, size_t n) {
  if (!dst || n == 0) return 0;
  RX_CRITICAL_ENTER(this);
  const int read = (int)rxBuf.pop(dst, n);
  RX_CRITICAL_EXIT(this);
  return read;
}

// Implement Stream::peek() using RingBuffer::peek(T&)
int BLESerial::peek() {
  uint8_t b = 0;
  RX_CRITICAL_ENTER(this);
  const bool hasByte = rxBuf.peek(b) == 1;
  RX_CRITICAL_EXIT(this);
  if (hasByte) return (int)b;
  return -1;
}

// Peak up to n bytes into dst using RingBuffer::peek(T*, n)
int BLESerial::peek(uint8_t *dst, size_t n) {
  if (!dst || n == 0) return 0;
  RX_CRITICAL_ENTER(this);
  const int peeked = (int)rxBuf.peek(dst, n);
  RX_CRITICAL_EXIT(this);
  return peeked;
}

void BLESerial::flush() {
  // If there's no active subscription, nothing will drain—return immediately.
  if (!isSubscribed()) {
    return;
  }

  const uint32_t deadline = millis() + FLUSH_MAX_WAIT_MS;

  #ifdef ARDUINO_ARCH_ESP32
    if (pumpMode == PumpMode::Task) {
      // The TX task exclusively owns the state machine in task mode.
      wakeTxTask();
      while (isSubscribed()) {
        if (txBuf.available() == 0 && pendingLen == 0) break;
        if ((int32_t)(millis() - deadline) >= 0) break;
        delay(1);
      }
      return;
    }
  #endif

  while (txBuf.available() > 0 || pendingLen > 0) {
    pumpTx();
    if (txBuf.available() == 0 && pendingLen == 0) break;
    if (!isSubscribed()) break; // link went away mid-flush
    if ((int32_t)(millis() - deadline) >= 0) break; // bounded wait
    delay(1);
  }
}

// Is write queue available to receive data?
bool BLESerial::writeReady() const {
  return (!txLocked && isSubscribed());
}

// Write single byte
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
      if (pumpMode == PumpMode::Task && txState!=TxState::Pending)
        wakeTxTask();
    #endif
  }

  return pushed;
}

// Write n bytes
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
      if (pumpMode == PumpMode::Task && txState!=TxState::Pending)
        wakeTxTask();
    #endif
  }
  return pushed;
}

// Write n bytes with timeout
size_t BLESerial::writeTimeout(const uint8_t *p, size_t n, uint32_t timeoutMs) {
  // blocking write with timeout, returns number of bytes written
  // ensure all bytes are queued or timeout expires
  if (!p || n == 0) return 0;
  const uint32_t endAt = millis() + timeoutMs;
  size_t pushed = 0;

  while (pushed < n) {
    // Respect staged/in-flight frames: wait until they clear
    if (txLocked) {
      if (txBuf.available() <= lowWater) {
        txLocked = false; // unlock and allow write(s)
      } else {  
        if ((int32_t)(millis() - endAt) >= 0) break; // timeout
        #ifdef ARDUINO_ARCH_ESP32
          if (pumpMode == PumpMode::Polling) {
            pumpTx();
          } else if (isSubscribed() && txState != TxState::Pending) {
            wakeTxTask();
          }
        #else
          pumpTx();
        #endif
        delay(1);
        continue;
      }
    }

    size_t s = txBuf.push(p + pushed, n - pushed, false);
    pushed += s;

    // After push, buffer only grows: check high-water lock
    if (txBuf.available() >= highWater) txLocked = true;
    if (pushed == n) break;

    #ifdef ARDUINO_ARCH_ESP32
      if (pumpMode == PumpMode::Polling) {
        pumpTx();
      } else if (isSubscribed() && txState != TxState::Pending) {
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

// Print formatted string
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

// Report statistics
void BLESerial::printStats(Stream &out) {
  expirePairingWindowIfNeeded();
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
  out.print(F("  PairingPolicy: "));
  out.print(pairingPolicyName(pairingPolicy));
  out.print(F("  PairingWindow: "));
  out.print(isPairingWindowOpen() ? F("open") : F("closed"));
  out.print(F("\r\n"));

  // Link summary
  out.print(F("  Link: connected="));
  out.print(deviceConnected ? F("yes") : F("no"));
  out.print(F(" subscribed="));
  out.print(clientSubscribed ? F("yes") : F("no"));
  out.print(F(" PHY="));
  out.print(phyToStr());
  out.print(F("\r\n"));

  // MTU / Chunk and whether it fits a single LL PDU
  out.print(F("  MTU: negotiated=")); out.print(mtu);
  out.print(F(" preferred="));          out.print(preferredMtu);
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
  // Per-event share (effective window divided by PDUS_PER_WINDOW)
  out.print(F(" per-event-share="));
  out.print(perEventShareUs);  
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
  out.print(F(" con_intvl_ms="));
  out.print(connItvlUnits ? (connItvlUnits * 1.25f) : 0.0f);
  out.print(F(" con_latency="));
  out.print(connLatency);
  out.print(F("\r\n"));

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
  out.print(F(" pending=")); out.print(txState == TxState::Pending ? F("yes") : F("no"));
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
  } else {
    out.print(F("  RSSI: N/A (not connected)\r\n"));
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
  out.print(F(" Message Size="));  out.print(msgSizeCount);
  out.print(F(" Software="));      out.print(softwareErrorCount);
  out.print(F(" Congestions="));   out.print(congestionCount);
  out.print(F(" Timeouts="));      out.print(timeoutCount);
  out.print(F(" Disconects="));    out.print(disconnectCount);
  out.print(F(" Unclassifieds=")); out.print(unclassifiedCount);
  out.print(F("\r\n"));
}

// Clear statistics counters
void BLESerial::clearStats() {
  TX_CRITICAL_ENTER(this);
  bytesTx            = 0;
  txDrops            = 0;
  txSuccessCount     = 0;
  congestionCount    = 0;
  timeoutCount       = 0;
  disconnectCount    = 0;
  unclassifiedCount  = 0;
  softwareErrorCount = 0;
  msgSizeCount       = 0;
  TX_CRITICAL_EXIT(this);

  RX_CRITICAL_ENTER(this);
  bytesRx = 0;
  rxDrops = 0;
  RX_CRITICAL_EXIT(this);
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

// ==============================================================================================
// ==============================================================================================

// ===== Tx Pump ================================================================================

void BLESerial::update() {
  expirePairingWindowIfNeeded();
  #ifdef ARDUINO_ARCH_ESP32
    // Use portable polling pump and not ESP32 FreeRTOS task
    if (pumpMode == PumpMode::Polling)
      pumpTx();
    // link adjustment is handled in RSSI task
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

void BLESerial::pumpTx() {
  // Skip if not subscribed or no TX characteristic available; avoids dropping popped data
  if (!isSubscribed() || txChar == nullptr) return;

  if (txState == TxState::Staging) {
    // Throttle sending
    uint32_t now = micros();
    uint32_t elapsed = (uint32_t)(now - lastTxUs);
    if (elapsed >= sendIntervalUs) {
      advanceTxStateMachine();
      lastTxUs = now;
    }
  } else {
    advanceTxStateMachine(); 
  }
} // end pumpTx

void BLESerial::pumpTxTask(void *arg) {
#ifdef ARDUINO_ARCH_ESP32
  BLESerial *self = static_cast<BLESerial *>(arg);
  if (!self) {
    // If no instance provided, nothing to pump
    vTaskDelay(pdMS_TO_TICKS(200));
    return;
  }

  BLESerial &s = *self;

  // Forever loop
  for (;;) {

    // Block until notified or timeout for periodic check
    if (s.isConnected()) {
      ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(5));
    } else {
      ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(200));
    }

    // Spin while subsribed
    while (s.isSubscribed() && (s.txChar != nullptr)) {

      // Break if no data and we're in Waiting
      if (s.txState == TxState::Waiting && s.txBuf.available() == 0) {
        break;
      }

      if (s.txState == TxState::Staging) {
        // Throttle sending
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
        }
        // Timestamp the actual attempt after any pacing delay so the next attempt
        // cannot start before the configured interval has elapsed.
        s.lastTxUs = micros();
        s.advanceTxStateMachine();
      } else {
        s.advanceTxStateMachine(); 
      }
    } // end while connected/subscribed
  } // end for (;;
#endif
} // end pumpTxTask

void BLESerial::setPumpMode(PumpMode m) {
#ifdef ARDUINO_ARCH_ESP32
  pumpMode = m;

  if (pumpMode == PumpMode::Task) {
    startTxTask();
    if (pumpMode != PumpMode::Task) {
      stopRSSITask();
      return;
    }
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
#else
  // Non-ESP32 builds only support Polling mode
  pumpMode = PumpMode::Polling;
#endif
}

void BLESerial::startTxTask() {
#ifdef ARDUINO_ARCH_ESP32
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
      pumpMode = PumpMode::Polling;
      LOGW(
        "BLESerial: TX task creation failed; "
        "falling back to polling mode.");
    } else {
      // Keep task in a mostly idle state until notified or connected
      vTaskSuspend(txTaskHandle); // begin asleep as requested
      LOGI("BLESerial: TX task created (suspended).");
    }
  }
#endif
}

void BLESerial::stopTxTask() {
#ifdef ARDUINO_ARCH_ESP32
  if (!txTaskHandle) return;
  vTaskDelete(txTaskHandle);
  txTaskHandle = nullptr;
  LOGI("BLESerial: TX task stopped.");
#endif
}

void BLESerial::wakeTxTask() {
#ifdef ARDUINO_ARCH_ESP32
  // Direct-to-task notification is very light-weight (few CPU cycles)
  if (!txTaskHandle) return;
  // If task is suspended, resume it first so it can process notifications
  eTaskState st = eTaskGetState(txTaskHandle);
  if (st == eSuspended) {
    vTaskResume(txTaskHandle);
  }
  xTaskNotifyGive(txTaskHandle);
#endif
}

void BLESerial::suspendTxTask() {
#ifdef ARDUINO_ARCH_ESP32
  if (txTaskHandle) {
    vTaskSuspend(txTaskHandle);
  }
#endif
}

// ===== RSSI ================================================================================

void BLESerial::RSSITask(void *arg) {
#ifdef ARDUINO_ARCH_ESP32
  BLESerial *self = static_cast<BLESerial *>(arg);
  if (!self) {
    // Nothing to do if task wasn't given a valid instance pointer
    vTaskDelay(pdMS_TO_TICKS(RSSI_INTERVAL_MS));
    return;
  }
  BLESerial &s = *self;
  for (;;) {
    s.expirePairingWindowIfNeeded();

    if (s.isConnected()) {
      s.adjustLink();
    } else if (!s.isPairingWindowOpen()) {
      vTaskSuspend(nullptr);
    }

    vTaskDelay(pdMS_TO_TICKS(RSSI_INTERVAL_MS));
  }
#endif
}

void BLESerial::startRSSITask() {
#ifdef ARDUINO_ARCH_ESP32
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
      LOGW("BLESerial: RSSITask creation failed rc=%d.", (int)rc);
    } else {
      vTaskSuspend(rssiTaskHandle);
      LOGI("BLESerial: RSSI task created (suspended).");
    }
  }
#endif
}

void BLESerial::stopRSSITask() {
#ifdef ARDUINO_ARCH_ESP32
  if (!rssiTaskHandle) return;
  vTaskDelete(rssiTaskHandle);
  rssiTaskHandle = nullptr;
  LOGI("BLESerial: RSSI task stopped.");
#endif
}

void BLESerial::wakeRSSITask() {
#ifdef ARDUINO_ARCH_ESP32
  if (!rssiTaskHandle) return;
  // RSSI task does not wait on notifications; ensure it's running
  eTaskState st = eTaskGetState(rssiTaskHandle);
  if (st == eSuspended) {
    vTaskResume(rssiTaskHandle);
  }
#endif
}

void BLESerial::suspendRSSITask() {
#ifdef ARDUINO_ARCH_ESP32
  if (rssiTaskHandle && !isPairingWindowOpen()) {
    vTaskSuspend(rssiTaskHandle);
  }
#endif
}

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
    if (logGetLevel() <= LOG_LEVEL_INFO) {
      const char *target = (desiredCodedScheme ? (desiredCodedScheme == 2 ? "CODED(S2)" : "CODED(S8)")
                                               : (desiredPhyMask == BLE_GAP_LE_PHY_2M_MASK ? "2M" : "1M"));
      LOGI("BLESerial: RSSI adapt: avg=%d raw=%d -> %s.",
           rssiAvg, rssiRaw, target);
    }
  } else {
    LOGW("BLESerial: PHY adapt failed (rc=%d).", rc);
  }
}

// ===== State Machine ==========================================================================

void BLESerial::advanceTxStateMachine(){
  switch (txState)
  {   
    default:
    case TxState::Waiting:
      // Ringbuffer empty or no connection
      txState = txWaiting();
      // if now data stage, otherwise continue waiting
      break;
    case TxState::Staging:
      // Take from buffer and Setvalue/Notify
      txState = txStaging();
      // if no data go to waiting
      // if setvalue failed go to staging
      // if notify failed go to recovering
      // if notify success go to pending
      break;
    case TxState::Pending:
      // wait for status code
      txState = txPending();
      // depending on status go to staging, recovering, discarding, or waiting
      break;
    case TxState::Recovering:
      // resend staged data
      txState = txRecovering();
      // go to pending
      // if notify failed go to discarding
      break;
    case TxState::Discarding:
      // discard staged data
      txState = txDiscarding();
      // go to staging
      break;
  }
}

BLESerial::TxState BLESerial::txWaiting(){
  // check if there is data to send and connection is active
  if (isSubscribed() && (txBuf.available() > 0) && txChar != nullptr){
    return TxState::Staging;
  }
  return TxState::Waiting;
}

BLESerial::TxState  BLESerial::txStaging() {
  // Stage the next chunk by peeking into pending buffer.
 
  size_t avail = txBuf.available();
  if (avail == 0) return TxState::Waiting;

  size_t toSend = (txChunkSize <= avail) ? txChunkSize : avail;
  pendingLen = txBuf.peek(pending, toSend);

  if (pendingLen == 0) return TxState::Waiting;

  // If characteristic vanished unexpectedly
  if (!txChar) {
    LOGE("BLESerial: TX characteristic went missing during staging.");
    return TxState::Waiting;
  }

  TX_CRITICAL_ENTER(this); // ----------------
  txSuccess = TxSuccess::NotSet; // clear prior success state
  TX_CRITICAL_EXIT(this); // ----------------

  // Set value (NimBLE copies internally) 
  // if (!txChar->setValue(pending, pendingLen)) {
  //   // setValue failed
  //   setValueFailedCount++;
  //   if (setValueFailedCount >= MAX_SETVALUE_FAILS) {
  //     // Too many failures: assume char is broken
  //     LOGE("BLESerial: TX characteristic setValue() failed repeatedly, discarding.");
  //     return TxState::Discarding;
  //   } else {
  //     LOGE("BLESerial: TX setValue() failed.");
  //   }
  //   return TxState::Staging;
  // }

  txChar->setValue(pending, pendingLen);
  
  // Notify (attempt transmission)
  if (!txChar->notify()) {
    // Notify failed
    notifyFailedCount++;
    if (notifyFailedCount > MAX_NOTIFY_FAILS) {
      LOGE("BLESerial: TX notify() failed repeatedly. Discarding.");
      return TxState::Discarding ;
    } else {
      LOGW("BLESerial: TX notify() failed.");
      return TxState::Recovering; // notify again later
    }
  }
  return TxState::Pending;
} // end stageTx

BLESerial::TxState  BLESerial::txPending() {
  // Check if previous notify was successful
  //   and handle errors accordingly
  
  // setValueFailedCount = 0; // reset setValue failure counter
  notifyFailedCount = 0; // reset notify failure counter

  TxSuccess status;
  TX_CRITICAL_ENTER(this); // ----------------
  status    = txSuccess;
  txSuccess = TxSuccess::NotSet;    // consume the status atomically
  TX_CRITICAL_EXIT(this); // ----------------

  switch (status)
  {   
    case TxSuccess::NotSet:
      // still waiting for status
      onNotSet();
      return TxState::Pending; // return to itself

    case TxSuccess::Success:
      onTxSuccess();
      return TxState::Staging;
      
    case TxSuccess::MessageSizeTooBig:
      onMessageTooBig();
      return TxState::Staging; // stage with new smaller size

    case TxSuccess::Congestion:
      onCongestion();
      return TxState::Recovering; // resend staged data

    case TxSuccess::Timeout:
      onTimeout();
      return TxState::Recovering; // resend staged data

    case TxSuccess::Disconnected:
      onDisconnected();
      return TxState::Waiting; // go idle

    case TxSuccess::SoftwareError:
      onSoftwareError();
      return TxState::Discarding; // discard this chunk

    default:
    case TxSuccess::Unclassified:
      onUnclassified();
      return TxState::Staging; // take from buffer and Setvalue/Notify again

  }
} // end txPending

BLESerial::TxState  BLESerial::txRecovering() {
  // To recover we need to simply appply notification again
  // the data was already staged with setValue() previously
  
  // If characteristic vanished unexpectedly
  if (txChar == nullptr) {
    LOGE("BLESerial: TX characteristic missing during recovery.");
    return TxState::Waiting;
  }

  // Cooldown after congestion before re-notify
  uint32_t nowUs = (uint32_t)micros();
  if (lastCongestionAtUs && (nowUs - lastCongestionAtUs) < CONGESTION_RETRY_COOLDOWN_US) {
    #ifdef ARDUINO_ARCH_ESP32
      if (pumpMode == PumpMode::Task) vTaskDelay(pdMS_TO_TICKS(1));
      else delayMicroseconds(250);
    #else
      delayMicroseconds(250);
    #endif
  }

  // Bounded backoff retries for transient congestion (EAGAIN/EBUSY)
  // - For Polling mode: short micro sleep to avoid tight re-notify loops
  // - For Task mode: let the task scheduler run; no busy-wait
  // - Retry count capped by RECOVER_RETRY_MAX to avoid livelock

  if (notifyFailedCount < RECOVER_RETRY_MAX) {
    // Cooperative delay depending on pump mode
    #ifdef ARDUINO_ARCH_ESP32
    if (pumpMode == PumpMode::Polling) {
      // Sleep ~250–750us with tiny jitter
      uint32_t jitter = 250u + (uint32_t)random(0, 500);
      delayMicroseconds(jitter);
    } else {
      // Task mode: yield a tick if we’re hammering notify()
      vTaskDelay(pdMS_TO_TICKS(1));
    }
    #else
    // Portable polling path: micro sleep
    uint32_t jitter = 250u + (uint32_t)random(0, 500);
    delayMicroseconds(jitter);
    #endif
  }

  // Try to notify again
  if (!txChar->notify()) {
    notifyFailedCount++;
    // If we have exceeded bounded retries, discard this staged chunk
    if (notifyFailedCount >= RECOVER_RETRY_MAX) {
      LOGW("BLESerial: TX notify() failed %u times in recovery; discarding.",
           (unsigned)notifyFailedCount);
      return TxState::Discarding;
    }
    // Otherwise keep recovering (will re-enter with bounded backoff)
    LOGW("BLESerial: TX notify() failed in recovery (retry %u/%u).",
         (unsigned)notifyFailedCount, (unsigned)RECOVER_RETRY_MAX);
    return TxState::Recovering;
  }

  // Success: reset counter and proceed to wait for status
  notifyFailedCount = 0; 
  return TxState::Pending;
} // end txRecovering

BLESerial::TxState  BLESerial::txDiscarding() {
  // consume staged data without sending
  const size_t dropLen = pendingLen;
  txBuf.consume(dropLen);  
  txDrops += dropLen;
  TX_CRITICAL_ENTER(this); // ----------------
  pendingLen = 0;
  TX_CRITICAL_EXIT(this); // ----------------

  // setValueFailedCount = 0; // reset setValue failure counter
  notifyFailedCount   = 0; // reset notify failure counter

  // Update Tx lock
  if (txBuf.available() <= lowWater) txLocked = false;

  discardStreak = discardStreak + 1u;

  if (discardStreak >= DISCARD_BACKOFF_AFTER) {
    // Apply a heavier cooldown / backoff

    discardStreak      = 0;             // reset streak after escalation
    successStreak      = 0;
    coolDowns          = 0;
    probing            = false;
    probeSuccesses     = 0;

    // Back off pacing, similar to congestion escalation
    lkgIntervalUs      = (lkgIntervalUs * DISCARD_COOLDOWN_FACTOR_NUM) / DISCARD_COOLDOWN_FACTOR_DEN;
    sendIntervalUs     = lkgIntervalUs;
    lastTxUs           = (uint32_t)micros();  // respect slower pacing now

    recentlyBackedOff  = true;          // reuse your existing cooldown gate

    LOGW(
      "BLESerial: %s: discard burst -> backoff, new LKG=%uµs.",
      hsCodeName(onStatusCode), (unsigned)lkgIntervalUs);
  }

  // Update Tx lock
  if (txBuf.available() <= lowWater) txLocked = false;

  LOGE("BLESerial: %s: TX unsuccessful, discarding %u bytes.",
       hsCodeName(onStatusCode), (unsigned)dropLen);
  return TxState::Staging;
} // end txDiscarding

// ==============================================================================================
// ==============================================================================================

// === Response handlers for txStatus ===========================================================
// These handlers are called from txPending() based on the txSuccess code set by the tx callback

// --- No Response Yet ---------------------------------
// ---------------------------------------------------------------
// 
// short delay to yield to other tasks
void BLESerial::onNotSet(){
  // yield to other tasks
  if (pumpMode == PumpMode::Task) {
    #ifdef ARDUINO_ARCH_ESP32
    vTaskDelay(1); // yield to other tasks
    #endif
  } else {
    delayMicroseconds(500);
  }
}
// ---------------------------------------------------------------

// --- Success path: OK or EDONE ---------------------------------
// ---------------------------------------------------------------
// 
// Update txLock, initiate and handle probes
void BLESerial::onTxSuccess(){

  // 1) consume staged data from buffer
  txBuf.consume(pendingLen);
  {
    TX_CRITICAL_ENTER(this); // ---------------------
    bytesTx        += pendingLen;
    pendingLen      = 0;
    emsgSizeRetries = 0;
    timeoutRetries  = 0;
    discardStreak   = 0;
    TX_CRITICAL_EXIT(this); // ---------------------
  }

  // Update Tx lock
  if (txBuf.available() <= lowWater) txLocked = false;

  // 2) Cooldown gate after a backoff 
  {
    TX_CRITICAL_ENTER(this); // ----------------
    if (recentlyBackedOff) {
      coolDowns = coolDowns + 1u;
      if (coolDowns >= COOL_SUCCESS_REQUIRED) {
        recentlyBackedOff = false;
        coolDowns         = 0;
        successStreak     = 0;
        discardStreak     = 0;
        // lkgFails          = 0; 
      }
      TX_CRITICAL_EXIT(this); // ----------------
      return;
    }
    TX_CRITICAL_EXIT(this); // ----------------
  }

  // 3) If already probing, confirm success and finish if enough
  {
    TX_CRITICAL_ENTER(this);
    if (probing) {
      probeSuccesses = probeSuccesses + 1u;
      if (probeSuccesses >= PROBE_CONFIRM_SUCCESSES) {
        lkgIntervalUs  = sendIntervalUs;   // accept new interval
        probing        = false;
        probeSuccesses = 0;
        lkgFails       = 0;
        successStreak  = 0;
        TX_CRITICAL_EXIT(this);
        LOGI("BLESerial: Probe %u accepted. LKG=%u.", sendIntervalUs, lkgIntervalUs);
        return;
      }
      TX_CRITICAL_EXIT(this);
      return; // keep probing
    }
    TX_CRITICAL_EXIT(this);
  }

  // 4.) Not probing
  //  - clear failures
  //  - count success streak
  //  - if enough successes, start a probe by lowering send interval
  bool startProbe = false;
  uint32_t lkgSnapUs = 0;
  {
    TX_CRITICAL_ENTER(this); // ------------------
    lkgFails = 0;
    successStreak = successStreak + 1u;
    // Enough successes to consider a probe?
    if (successStreak >= PROBE_AFTER_SUCCESSES) {
      // Start a probe and update last known good interval value
      successStreak = 0; // reset success streak
      // if (sendIntervalUs <= minSendIntervalUS) 
      //   return; // already at floor, do not lower interval further
      lkgIntervalUs = sendIntervalUs;
      lkgSnapUs     = lkgIntervalUs;
      startProbe    = true;
    }
    TX_CRITICAL_EXIT(this); // -----------------
  }
    
  if (!startProbe) return;

  // 5) Compute probe step and soft floor, then attempt lowering interval

  // Either lower interval by microseconds or by %. 
  const uint32_t stepAbs = PROBE_STEP_US;
  const uint32_t stepPct = (sendIntervalUs * PROBE_STEP_PCT) / 100u;
  const uint32_t baseStep = (stepPct > stepAbs) ? stepPct : stepAbs;
  // Add small jitter
  //   Jitter: +/- up to 25% of baseStep; 
  const uint32_t jitter = (baseStep / 4);
  const uint32_t rnd = (uint32_t)random(0, (int)(jitter * 2 + 1));
  uint32_t       step = (rnd > jitter) ? (baseStep + (rnd - jitter)) : (baseStep - rnd);
  if (step == 0) step = 1; // ensure non-zero step

  uint32_t lowerBound = perEventShareUs ? perEventShareUs : minSendIntervalUS;
  
  // If already at/below the floor, don’t probe
  if (sendIntervalUs <= lowerBound) {
    // LOGI("BLESerial: Probe floor reached (%uµs), no further probe.", (unsigned)lowerBound);
    return;
  }

  // Compute next; clamp to softFloor; avoid no-op
  uint32_t next = (sendIntervalUs > step) ? (sendIntervalUs - step) : sendIntervalUs;
  if (next < lowerBound) next = lowerBound;
  if (next == sendIntervalUs) {
    // LOGI("BLESerial: Probe step collapsed (no change), floor=%uµs.", (unsigned)lowerBound);
    return;
  }

  // 6) Commit new interval and start probing
  {
    TX_CRITICAL_ENTER(this);
    sendIntervalUs = next;
    probing        = true;
    probeSuccesses = 0;
    lkgFails       = 0;  // reset for fresh probe attempt
    TX_CRITICAL_EXIT(this);
  }

  LOGI("BLESerial: Starting probe: %u -> %u.", lkgSnapUs, sendIntervalUs);
  return;
} // --- end of success path -------------------------------------
// ---------------------------------------------------------------

// --- EMSGSIZE: payload too big for context ---------------------
// ---------------------------------------------------------------
//
// Staged chunk too large for current MTU/ATT limits
// ✔ Split your pending chunk into smaller parts
// ✔ Or reduce your global chunkSize 
// ✔ Then retry setValue() with the smaller piece
void BLESerial::onMessageTooBig(){

  // Locals computed outside the lock
  uint16_t prevChunk       = txChunkSize;
  uint16_t newChunk        = prevChunk;
  uint16_t previousRetries = 0;
  bool     didExhaustRetries = false;
  bool     shouldDisconnect  = false;

  // Update retry count and decide next chunk size (no shared writes yet)
  emsgSizeRetries = emsgSizeRetries + 1;
  if (emsgSizeRetries <= EMSGSIZE_RETRY_MAX) {
    // reduce chunk size by half with floor
    newChunk = std::max<uint16_t>(MIN_CHUNKSIZE, static_cast<uint16_t>(prevChunk / 2));
  } else {
    didExhaustRetries = true;
    previousRetries   = emsgSizeRetries;

    if (prevChunk > MIN_CHUNKSIZE) {
      newChunk = MIN_CHUNKSIZE;
    } else {
      shouldDisconnect = true;      // give up and disconnect
    }
    emsgSizeRetries = 0;            // reset counter
  }

  // Recompute watermarks and minimum send interval for the candidate chunk (outside lock)
  // Note: updateWaterMarks writes shared fields; compute its values first:
  // We’ll call updateWaterMarks only once we’ve committed newChunk under lock.
  uint32_t newMinSendUs = computeSendIntervalUs(newChunk);

  // Commit the new chunk and timing; keep lock short and only for shared-state mutations
  {
    TX_CRITICAL_ENTER(this);
    txChunkSize = newChunk;

    // Because chunk size changed:
    updateWaterMarks(static_cast<size_t>(txChunkSize));  // recalculates low/high water and reconciles txLocked
    minSendIntervalUS = newMinSendUs;

    // Clamp active interval to the new floor and align LKG
    if (sendIntervalUs < minSendIntervalUS) {
      sendIntervalUs = minSendIntervalUS;
      lkgIntervalUs  = sendIntervalUs;
    } else {
      // Even if we don’t clamp down, reset LKG to current active for a clean floor after EMSGSIZE
      lkgIntervalUs = sendIntervalUs;
    }

    // Reset ramp/backoff state
    successStreak     = 0;
    discardStreak     = 0;
    recentlyBackedOff = true;
    coolDowns         = 0;

    if (probing) {
      probing           = false;
      probeSuccesses    = 0;
      lkgFails          = 0;
    }
    TX_CRITICAL_EXIT(this);
  }

  // Log 
  if (logGetLevel() <= LOG_LEVEL_INFO) {
    if (!didExhaustRetries) {
      if (txChunkSize == prevChunk) {
        LOGI(
          "BLESerial: %s: but chunk already at MIN (%u) min_send_interval=%uµs (retry %d/%d).",
          hsCodeName(onStatusCode), txChunkSize, minSendIntervalUS, emsgSizeRetries, EMSGSIZE_RETRY_MAX);
      } else {
        LOGI(
          "BLESerial: %s: reducing chunk %u->%u min_send_interval=%uµs (retry %d/%d).",
          hsCodeName(onStatusCode), prevChunk, txChunkSize, minSendIntervalUS, emsgSizeRetries, EMSGSIZE_RETRY_MAX);
      }
    } else {
      LOGI(
        "BLESerial: %s adjust chunk %u->%u min_send_interval=%uµs (retry %d/%d).",
        hsCodeName(onStatusCode),
        prevChunk, txChunkSize,
        minSendIntervalUS,
        previousRetries, EMSGSIZE_RETRY_MAX
      );
    }
  }

  if (shouldDisconnect) {
    LOGW(
      "BLESerial: %s: persistent EMSGSIZE at MIN_CHUNKSIZE -> disconnect.",
      hsCodeName(onStatusCode));
    if (server && connHandle != BLE_HS_CONN_HANDLE_NONE) {
      server->disconnect(connHandle);
    }
  }

} // end of EMSGSIZE handling ------------------------------------
// ---------------------------------------------------------------

// Out of resources ----------------------------------------------
// temporary resource exhaustion
// -> retry same payload later with backoff.
void BLESerial::onCongestion(){
  lastCongestionAtUs = (uint32_t)micros();
  const uint32_t nowUs = (uint32_t)micros();
  bool logStoppedProbe = false;
  bool logEscalated    = false;
  uint32_t logSendUs   = 0;
  size_t   logUsed     = 0;
  uint32_t prevLkg     = 0;
  uint32_t prevIntvl   = 0;
  uint32_t logLkgFails = 0;


  // #ifdef ARDUINO_ARCH_ESP32
  // // Capture heap metrics early; they’re cheap and help diagnosing ENOMEM vs general heap pressure
  // size_t heapFree           = heap_caps_get_free_size(MALLOC_CAP_DEFAULT);
  // size_t heapFreeInternal   = heap_caps_get_free_size(MALLOC_CAP_INTERNAL);
  // size_t heapLargestBlock   = heap_caps_get_largest_free_block(MALLOC_CAP_DEFAULT);
  // size_t psramFree          = heap_caps_get_free_size(MALLOC_CAP_SPIRAM);
  // size_t psramLargestBlock  = heap_caps_get_largest_free_block(MALLOC_CAP_SPIRAM);
  // #endif

  {
    TX_CRITICAL_ENTER(this); // ------------------

    successStreak     = 0;
    coolDowns         = 0;

    // Use a small bump like your probe step
    const uint32_t bumpAbs = PROBE_STEP_US;
    const uint32_t bumpPct = (lkgIntervalUs * PROBE_STEP_PCT) / 100u;
    const uint32_t bump    = (bumpPct > bumpAbs) ? bumpPct : bumpAbs;

    // Fall back

    prevLkg           = lkgIntervalUs;
    recentlyBackedOff = true;
    lkgFails = lkgFails + 1u;

    if (probing) {
      // Probing, falling back to LKG
      prevIntvl       = sendIntervalUs;
      probing         = false;
      probeSuccesses  = 0;
      // lkgFails        = 0;
      sendIntervalUs  = lkgIntervalUs;
      lastTxUs        = nowUs;  // respect slower pacing immediately
      logStoppedProbe = true;
      logSendUs       = sendIntervalUs; // equals lkgIntervalUs after fallback
    } else if (lkgFails >= LKG_ESCALATE_AFTER_FAILS) {
      // Not probing
      // falling back to LKG was not enough: escalate LKG after repeated failures
      prevIntvl       = sendIntervalUs;
      // Escalate LKG exactly once
      lkgIntervalUs   = (lkgIntervalUs * LKG_ESCALATE_NUM) / LKG_ESCALATE_DEN;
      // Set send interval slightly above the new LKG to give the stack room
      sendIntervalUs  = lkgIntervalUs + bump * (uint32_t)(lkgFails);
      logLkgFails     = lkgFails;
      lkgFails        = 1;
      lastTxUs        = nowUs; // respect slower pacing immediately
      logEscalated    = true;
      logSendUs       = sendIntervalUs;
      logUsed         = txBuf.available();
    } else {
      // Not yet at escalation threshold: lift above current LKG to avoid re-hitting the same pace
      prevIntvl = sendIntervalUs;
      sendIntervalUs = lkgIntervalUs + bump * (uint32_t)(lkgFails);
      lastTxUs = nowUs;
      logSendUs = sendIntervalUs;
      logUsed   = txBuf.available();
    }
    TX_CRITICAL_EXIT(this); // ------------------
  }

  if (logStoppedProbe) {
    LOGW("BLESerial: %s: failing probe, revert to LKG=%u µs.",
         hsCodeName(onStatusCode), (unsigned)logSendUs);
      // #ifdef ARDUINO_ARCH_ESP32
      // LOGW("BLESerial: Heap: free=%u int=%u largest=%u psram_free=%u psram_largest=%u.",
      //               (unsigned)heapFree, (unsigned)heapFreeInternal,
      //               (unsigned)heapLargestBlock,
      //               (unsigned)psramFree, (unsigned)psramLargestBlock);
      // #endif
    return;
  }

  if (logEscalated) {
    LOGW("BLESerial: %s: %u/%u, fallback %u -> %u & escalate LKG %u -> %u µs txBuf=%u.",
         hsCodeName(onStatusCode),
         (unsigned)logLkgFails, (unsigned)LKG_ESCALATE_AFTER_FAILS,
         (unsigned)(prevIntvl), (unsigned)logSendUs,
         (unsigned)prevLkg, (unsigned)lkgIntervalUs,
         (unsigned)logUsed);
      // #ifdef ARDUINO_ARCH_ESP32
      // LOGW("BLESerial: Heap: free=%u int=%u largest=%u psram_free=%u psram_largest=%u.",
      //               (unsigned)heapFree, (unsigned)heapFreeInternal,
      //               (unsigned)heapLargestBlock,
      //               (unsigned)psramFree, (unsigned)psramLargestBlock);
      // #endif                    
  } else {
    LOGW("BLESerial: %s: %u/%u, fallback %u -> %u µs.",
         hsCodeName(onStatusCode),
         (unsigned)lkgFails, (unsigned)LKG_ESCALATE_AFTER_FAILS,
         (unsigned)(prevIntvl), (unsigned)logSendUs);
      // #ifdef ARDUINO_ARCH_ESP32
      // LOGW("BLESerial: Heap: free=%u int=%u largest=%u psram_free=%u psram_largest=%u.",
      //               (unsigned)heapFree, (unsigned)heapFreeInternal,
      //               (unsigned)heapLargestBlock,
      //               (unsigned)psramFree, (unsigned)psramLargestBlock);
      // #endif    
  }

} // end of congestion handling
// ---------------------------------------------------------------

// Timeout Handling ----------------------------------------------
// Notification call timed out
// -> retry a couple times then disconnect if persistent
void BLESerial::onTimeout(){
  timeoutRetries = timeoutRetries + 1u;

  const uint32_t nowUs = (uint32_t)micros();
  bool logStoppedProbe = false;
  bool logEscalated    = false;
  uint32_t logSendUs   = 0;
  size_t   logUsed     = 0;
  uint32_t logLkgFails = 0;
  uint32_t prevLkg     = 0;
  uint32_t prevIntvl   = 0;

  // #ifdef ARDUINO_ARCH_ESP32
  // size_t heapFree           = heap_caps_get_free_size(MALLOC_CAP_DEFAULT);
  // size_t heapFreeInternal   = heap_caps_get_free_size(MALLOC_CAP_INTERNAL);
  // size_t heapLargestBlock   = heap_caps_get_largest_free_block(MALLOC_CAP_DEFAULT);
  // size_t psramFree          = heap_caps_get_free_size(MALLOC_CAP_SPIRAM);
  // size_t psramLargestBlock  = heap_caps_get_largest_free_block(MALLOC_CAP_SPIRAM);
  // #endif

  {
    TX_CRITICAL_ENTER(this);
    successStreak     = 0;
    coolDowns         = 0;

    const uint32_t bumpAbs = PROBE_STEP_US;
    const uint32_t bumpPct = (lkgIntervalUs * PROBE_STEP_PCT) / 100u;
    const uint32_t bump    = (bumpPct > bumpAbs) ? bumpPct : bumpAbs;

    // Fall back
    prevLkg           = lkgIntervalUs;
    recentlyBackedOff = true;
    lkgFails = lkgFails + 1u;

    if (probing) {
      // Probing, falling back to LKG
      prevIntvl       = sendIntervalUs;
      probing         = false;
      probeSuccesses  = 0;
      // lkgFails        = 0;
      sendIntervalUs  = lkgIntervalUs;     // first fallback: exact LKG
      lastTxUs        = nowUs;
      logStoppedProbe = true;
      logSendUs       = sendIntervalUs;    // equals LKG after fallback
    } else if (lkgFails >= LKG_ESCALATE_AFTER_FAILS) {
      // Not probing
      // falling back to LKG was not enough: escalate LKG after repeated failures
      prevIntvl       = sendIntervalUs;
      // Escalate LKG exactly once
      lkgIntervalUs   = (lkgIntervalUs * LKG_ESCALATE_NUM) / LKG_ESCALATE_DEN;
      // Place send just above new LKG using the current streak
      sendIntervalUs  = lkgIntervalUs + bump * (uint32_t)(lkgFails);
      logLkgFails     = lkgFails;
      lkgFails        = 1;                 // seed so next fallback stays above LKG
      lastTxUs        = nowUs; // respect slower pacing immediately
      logEscalated    = true;
      logSendUs       = sendIntervalUs;
      logUsed         = txBuf.available();
    } else {
      // First fallback: exact LKG; subsequent: LKG + bump × (fails−1)
      prevIntvl = sendIntervalUs;
      sendIntervalUs = lkgIntervalUs + bump * (uint32_t)(lkgFails);
      lastTxUs       = nowUs;
      logSendUs      = sendIntervalUs;
      logUsed        = txBuf.available();
    }
    TX_CRITICAL_EXIT(this);
  }

  if (logStoppedProbe) {
    LOGW("BLESerial: %s: failing probe, revert to LKG=%u.",
         hsCodeName(onStatusCode), logSendUs);
      // #ifdef ARDUINO_ARCH_ESP32
      // LOGW("BLESerial: Heap: free=%u int=%u largest=%u psram_free=%u psram_largest=%u.",
      //               (unsigned)heapFree, (unsigned)heapFreeInternal,
      //               (unsigned)heapLargestBlock,
      //               (unsigned)psramFree, (unsigned)psramLargestBlock);
      // #endif                    
  } else if (logEscalated) {
    LOGW("BLESerial: %s: %u/%u, fallback %u -> %u & escalate LKG %u -> %u µs txBuf=%u.",
         hsCodeName(onStatusCode),
         (unsigned)logLkgFails, (unsigned)LKG_ESCALATE_AFTER_FAILS,
         (unsigned)(prevIntvl), (unsigned)logSendUs,
         (unsigned)prevLkg, (unsigned)lkgIntervalUs,
         (unsigned)logUsed);
      // #ifdef ARDUINO_ARCH_ESP32
      // LOGW("BLESerial: Heap: free=%u int=%u largest=%u psram_free=%u psram_largest=%u.",
      //               (unsigned)heapFree, (unsigned)heapFreeInternal,
      //               (unsigned)heapLargestBlock,
      //               (unsigned)psramFree, (unsigned)psramLargestBlock);
      // #endif                    
  } else {
    LOGW("BLESerial: %s: %u/%u, fallback %u -> %u µs.",
         hsCodeName(onStatusCode),
         (unsigned)lkgFails, (unsigned)LKG_ESCALATE_AFTER_FAILS,
         (unsigned)(prevIntvl), (unsigned)logSendUs);      // #ifdef ARDUINO_ARCH_ESP32
      // LOGW("BLESerial: Heap: free=%u int=%u largest=%u psram_free=%u psram_largest=%u.",
      //               (unsigned)heapFree, (unsigned)heapFreeInternal,
      //               (unsigned)heapLargestBlock,
      //               (unsigned)psramFree, (unsigned)psramLargestBlock);
      // #endif                    
  }

  if (timeoutRetries >= ETIMEOUT_RETRY_MAX) {
    // too many retries: disconnect
    LOGE("BLESerial: %s: persistent -> disconnect.", hsCodeName(onStatusCode));
    if (server && connHandle != BLE_HS_CONN_HANDLE_NONE)
      server->disconnect(connHandle);
    timeoutRetries = 0; // reset counter
  } else {
    LOGW("BLESerial: %s: retry %d/%d.",
         hsCodeName(onStatusCode), timeoutRetries, ETIMEOUT_RETRY_MAX);
  }
}
// ---------------------------------------------------------------

// --- Disconnected or OS Error ---- -----------------------------
// ---------------------------------------------------------------
//      teardown connection, clean up
//      -> do not retry
void BLESerial::onDisconnected(){
  TX_CRITICAL_ENTER(this);
  successStreak     = 0;
  recentlyBackedOff = false;
  coolDowns         = 0;
  probing           = false;
  probeSuccesses    = 0;
  lkgFails          = 0;
  sendIntervalUs    = 0;
  minSendIntervalUS = 0;
  discardStreak     = 0;
  lkgIntervalUs     = sendIntervalUs;
  TX_CRITICAL_EXIT(this);
  LOGW("BLESerial: %s: link closed.", hsCodeName(onStatusCode));
  return;
} // end of disconnect/EOS
// ---------------------------------------------------------------

// Local Bug or Software Error: ----------------------------------
// ---------------------------------------------------------------
//  turn of probing
//  if too many retries, disconnect
void BLESerial::onSoftwareError(){
  softwareRetries = softwareRetries + 1u;

  bool logStoppedProbe = false;
  uint32_t logLkgUs    = 0;
  
  {
    TX_CRITICAL_ENTER(this); // ------------------
    successStreak = 0;
    coolDowns     = 0;

    if (probing) {
      // Probing, falling back to LKG
      probing        = false;
      probeSuccesses = 0;
      sendIntervalUs = lkgIntervalUs; // revert to floor
      recentlyBackedOff = true;
      lkgFails  = 0; // 

      logStoppedProbe = true;
      logLkgUs        = sendIntervalUs;
    }
    TX_CRITICAL_EXIT(this); // ------------------
  }

  if (logStoppedProbe) {
    LOGI("BLESerial: %s failing probe, revert to LKG=%u.",
         hsCodeName(onStatusCode), logLkgUs);
  }

  if (softwareRetries >= SOFTWARE_RETRY_MAX) {
    LOGE("BLESerial: %s: persistent -> disconnect.", hsCodeName(onStatusCode));
    if (server && connHandle != BLE_HS_CONN_HANDLE_NONE)
      server->disconnect(connHandle);
    softwareRetries = 0; // reset counter
  }

  LOGE("BLESerial: Software Error %s:.", hsCodeName(onStatusCode));
} // -- end of local bug/bad state handling
// ---------------------------------------------------------------

// Unclassified: -------------------------------------------------
// ---------------------------------------------------------------
//   drop probe if probing; 
//   fall back to lkg
void BLESerial::onUnclassified(){

  bool logStoppedProbe = false;
  uint32_t logLkgUs    = 0;
  
  {
    TX_CRITICAL_ENTER(this);
    successStreak     = 0;
    coolDowns         = 0;

    // Fall back
    sendIntervalUs    = lkgIntervalUs;   // fall back to LKG
    recentlyBackedOff = true;
    lkgFails = lkgFails + 1u;

    if (probing){
      probing           = false;
      probeSuccesses    = 0;
      lkgFails          = 0;
      logStoppedProbe = true;
      logLkgUs        = sendIntervalUs;  // equals lkgIntervalUs after fallback
    } 
    TX_CRITICAL_EXIT(this);
  }

  if (logStoppedProbe) {
    LOGW(
      "BLESerial: %s: unclassified issue while probing: revert to LKG=%u.",
      hsCodeName(onStatusCode), logLkgUs);
  } else {
    LOGW("BLESerial: %s: unclassified issue.", hsCodeName(onStatusCode));
  }
} // end of unclassified
// ---------------------------------------------------------------  

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
  if (llTxOctets > (hdrBytes + micPerPdu))
    onePduMax = static_cast<uint16_t>(llTxOctets - hdrBytes - micPerPdu);

  // Two-PDU Fast mode remains deferred until NimBLE-Arduino supports it.
  (void)modeVal;
  uint16_t limit = onePduMax;

  // Do not exceed half the TX buffer (keep hysteresis meaningful)
  size_t cap2 = txCapacity / 2;
  if (cap2 > 0xFFFFu) cap2 = 0xFFFFu;
  if (limit > static_cast<uint16_t>(cap2))
    limit = static_cast<uint16_t>(cap2);

  // Cap by ATT MTU-derived payload (MTU-3)
  if (attPayload < limit) limit = attPayload;

  // Keep a practical floor (legacy 1-PDU safe)
  if (limit < MIN_CHUNKSIZE) limit = MIN_CHUNKSIZE;

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

  txChunkSize       = computeTxChunkSize(mtu, llTxOctets, mode, secure, txBuf.capacity());
  minSendIntervalUS = computeSendIntervalUs(txChunkSize);
  updateWaterMarks(static_cast<size_t>(txChunkSize));

  // Clamp the active interval to the current floor if requested or out of bounds
  if (sendIntervalUs == 0 || sendIntervalUs < minSendIntervalUS)
    sendIntervalUs = minSendIntervalUS;

  // After any renegotiation, make LKG match the active interval (the new floor)
  lkgIntervalUs = sendIntervalUs;

  // Reset probing/backoff state to the new floor
  TX_CRITICAL_ENTER(this); // ----------------
  probing           = false;
  probeSuccesses    = 0;
  lkgFails          = 0;
  recentlyBackedOff = false;
  coolDowns         = 0;
  successStreak     = 0;
  discardStreak     = 0;
  TX_CRITICAL_EXIT(this); // ----------------
}

// Helper
uint16_t BLESerial::micBytes(BLESerial::Security sec) {
// Determine per-PDU MIC bytes based on security mode
  return (sec == Security::PasskeyDisplay || sec == Security::JustWorks) ? BLE_SERIAL_ENCRYPT_BYTES : 0;
}

uint32_t BLESerial::computePerEventShareUs(uint32_t connInt,
                                           uint16_t connLat,
                                           uint32_t pdus_per_window) {
  // Recompute event share time for lower bound of send interval based on link parameters
  // Return the computed send interval in microseconds
  if (connInt == 0) return 0u;
  const uint32_t windowUs = connInt * static_cast<uint32_t>(connLat + 1u);
  // ceil division
  const uint32_t denom = pdus_per_window ? pdus_per_window : 1u;
  return (windowUs + denom - 1u) / denom;
}

uint32_t BLESerial::computeSendIntervalUs(uint16_t chunkSize) {
  // Compute time to transmit one chunk of given size over the link
  // Uses the negotiated llTxOctets and llTxTimeUs (globals)
  // Compare with connection interval and slave latency
  // Return the larger of the two with a guard margin

  // mic per LL PDU (inside L)
  uint16_t mic = micBytes(secure);
  
  // Effective per-PDU capacity for SDU bytes
  uint16_t M = 0;
  if (llTxOctets > mic)  {
    M = static_cast<uint16_t>(llTxOctets - mic);
  }
  // If M is zero (shouldn't happen), bail out with a safe large interval
  if (M == 0) return 1000000u;

  // Total SDU bytes to send (ATT + L2CAP headers + payload)
  const uint32_t sduBytes = static_cast<uint32_t>(chunkSize)
                            + BLE_SERIAL_ATT_HDR_BYTES
                            + BLE_SERIAL_L2CAP_HDR_BYTES;

  // How many full PDUs and leftover SDU bytes
  const uint32_t fullPduCount = sduBytes / M;
  const uint32_t lastPduSize  = sduBytes - fullPduCount * M;
  // Use negotiated llTxTimeUs as authoritative per-full-PDU time when available (>0),
  // otherwise compute per-PDU time from PHY and octets.
  uint32_t perFullPduUs = (llTxTimeUs > 0) 
      ? llTxTimeUs 
      : estimate_LL_PDUTimeUs(llTxOctets, phyIs2M, phyIsCoded, codedScheme);

  uint32_t totalUs = fullPduCount * perFullPduUs;

  // Estimate the remaining partial PDU time by linear interpolation
  // When used properly there should be no partial PDU here since we align to MTU
  if (lastPduSize) {
    // Adjust last PDU proportionally when we modeled capacity in SDU space
    const uint32_t lastAdjUs = (perFullPduUs * lastPduSize + (M - 1)) / M;
    // Add the adjusted partial-PDU time
    totalUs += lastAdjUs;    
  }

  // Last (partial) PDU: compute its octet length and compute time from PHY for accuracy.
  // if (lastPduSize) {
  //   uint32_t lastOctets = lastPduSize + BLE_SERIAL_L2CAP_HDR_BYTES + BLE_SERIAL_ATT_HDR_BYTES + mic;
  //   if (lastOctets < LL_MIN_OCTETS) lastOctets = LL_MIN_OCTETS;
  //   if (lastOctets > llTxOctets) lastOctets = llTxOctets;
  //   uint32_t lastPduUs = estimate_LL_PDUTimeUs(static_cast<uint16_t>(lastOctets), phyIs2M, phyIsCoded, codedScheme);
  //   totalUs += lastPduUs;
  // }

  // Optional clamp: do not attempt more PDUs per event than PDUS_PER_WINDOW allows.
  if (connIntervalUs && perEventShareUs) {
    // If our airtime math says we can go faster than the per-event share, cap to the share.
    if (totalUs < perEventShareUs) totalUs = perEventShareUs;
  }
 
  /*
  totalUs is the airtime needed to transmit the SDU (per-PDU time with possible partial PDU).
  perEventShareUs is the pacing bound derived from the connection event spacing divided by PDUS_PER_WINDOW.
  To avoid exceeding the intended PDUs per event, 
    the interval must be at least both the airtime and the per-event share. 
  */


  // Conservative guard: assume we need an additional spacing to account for stack/processing jitter
  return ((GUARD_NUM * totalUs) / 100u);
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

bool BLESerial::setPreferredMTU(uint16_t newMtu) {
  if (newMtu < BLE_SERIAL_MIN_MTU) newMtu = BLE_SERIAL_MIN_MTU;
  if (newMtu > BLE_SERIAL_MAX_MTU) newMtu = BLE_SERIAL_MAX_MTU;

  preferredMtu = newMtu;
  preferredMtuConfigured = true;

  // Server-side NimBLE exposes no API to start an MTU exchange. The peer
  // negotiates the active MTU, which arrives through onMTUChange().
  if (!NimBLEDevice::isInitialized()) {
    return true;
  }

  return NimBLEDevice::setMTU(preferredMtu);
}

void BLESerial::setPower(int8_t dBm, NimBLETxPowerType scope) {
  NimBLEDevice::setPower(dBm, scope);
  LOGI("BLESerial: TX Power set to %d dBm (scope %d).", dBm, (int)scope);
}
