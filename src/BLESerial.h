/******************************************************************************************************/
// Include file for BLESerial library
//
// Urs Utzinger
// November/December 2025
/******************************************************************************************************/

#ifndef BLE_SERIAL_H
#define BLE_SERIAL_H

// Standard Libraries
#include <stdint.h>
#include <inttypes.h>  // for PRIu32 in printf
#include <cmath>
#include <functional>
#include <string>

#include <Arduino.h>
#include "RingBuffer.h"

#include <NimBLEDevice.h>
extern "C" {
  #include "host/ble_gap.h"      // ble_gap_* (conn params, PHY, DLE)
  #include "host/ble_hs_adv.h"   // BLE_HS_ADV_F_* flags for adv data
  #include "host/ble_hs.h"       // BLE_HS_EDONE for notivy backoff
}

// for mac address
#if __has_include(<esp_mac.h>)
  #include <esp_mac.h>      // IDF 5.x / Arduino core 3.x
#else
  #include <esp_system.h>   // IDF 4.x / Arduino core 2.x
#endif

#ifdef ARDUINO_ARCH_ESP32
    #include "freertos/FreeRTOS.h"
    #include "freertos/portmacro.h"
    #include "freertos/task.h"
#endif

// Define critical section helpers BEFORE class so they are visible where used.
// They reference 'this->mux_' which is a per-instance spinlock.
#ifdef ARDUINO_ARCH_ESP32
  /* Per-instance critical helpers. These macros expect a pointer to an
    instance (e.g. TX_CRITICAL_ENTER(this) or TX_CRITICAL_ENTER(&s)).
    This form works both in non-static member functions and in static
    task functions where you have a local reference `s`. */
  #define TX_CRITICAL_ENTER(inst)  portENTER_CRITICAL(&(inst)->txMux)
  #define TX_CRITICAL_EXIT(inst)   portEXIT_CRITICAL(&(inst)->txMux)
#else
  #define TX_CRITICAL_ENTER(inst)
  #define TX_CRITICAL_EXIT(inst)
#endif

/******************************************************************************************************/
/* Definitions */
/* Constants   */
/******************************************************************************************************/

#define BLE_SERIAL_VERSION_STRING "BLE Serial Library v1.1.0"
#define BLE_SERIAL_APPEARANCE 0x0540 // Generic Sensor

// Log levels: ascending by verbosity for comparisons like (logLevel >= LEVEL)
// This ensures:
//  - NONE (0) disables all logs
//  - DEBUG (4) enables all logs
//  - INFO prints INFO/WARNING/ERROR; 
//  - WARNING prints WARNING/ERROR; 
//  - ERROR prints only errors

#define NONE     0
#define ERROR    1
#define WARNING  2
#define INFO     3
#define DEBUG    4

// Max GATT MTU supported (ESP32 max 517); 
// ATT payload per notify is MTU-3
#define BLE_SERIAL_MAX_MTU        517u
#define BLE_SERIAL_MAX_GATT       512u
#define BLE_SERIAL_DEFAULT_MTU    247u
#define BLE_SERIAL_MIN_MTU         23u
#define BLE_SERIAL_ATT_HDR_BYTES    3u
#define BLE_SERIAL_L2CAP_HDR_BYTES  4u
#define BLE_SERIAL_ENCRYPT_BYTES    4u

// dBm levels similar to Bluedroid's ESP_PWR_LVL_* names:
#define BLE_TX_DBN12  ( -12)
#define BLE_TX_DBN9     (-9)
#define BLE_TX_DBN6     (-6)
#define BLE_TX_DBN3     (-3)
#define BLE_TX_DB0       (0)
#define BLE_TX_DBP3      (3)
#define BLE_TX_DBP6      (6)
#define BLE_TX_DBP9      (9)   // ~max on many ESP32s

// Scopes roughly matching ESP_BLE_PWR_TYPE_*
#define PWR_ALL  NimBLETxPowerType::All
#define PWR_ADV  NimBLETxPowerType::Advertise
#define PWR_SCAN NimBLETxPowerType::Scan
#define PWR_CONN NimBLETxPowerType::Connection

// Nordic UART (NUS) UUIDs
static constexpr const char     BLE_SERIAL_SERVICE_UUID[]           = {"6E400001-B5A3-F393-E0A9-E50E24DCCA9E"};
static constexpr const char     BLE_SERIAL_CHARACTERISTIC_UUID_RX[] = {"6E400002-B5A3-F393-E0A9-E50E24DCCA9E"};
static constexpr const char     BLE_SERIAL_CHARACTERISTIC_UUID_TX[] = {"6E400003-B5A3-F393-E0A9-E50E24DCCA9E"};

// ===== GATT / ATT payload sizing =====
inline constexpr int8_t         RSSI_LOW_THRESHOLD              =  -80;   // low power threshold (increase power if in LOWPOWER mode)
inline constexpr int8_t         RSSI_FAST_THRESHOLD             =  -65;   // Switch back to 2M/1M
inline constexpr int8_t         RSSI_HYSTERESIS                 =    4;   // Prevent oscillation when near threshold
inline constexpr int8_t         RSSI_S8_THRESHOLD               =  -82;   // go S=8 below this
inline constexpr int8_t         RSSI_S2_THRESHOLD               =  -75;   // go S=2 below this
inline constexpr uint32_t       RSSI_INTERVAL_MS                = 500UL;  // 0.5s
inline constexpr uint32_t       RSSI_ACTION_COOLDOWN_MS         = 4000UL; // 4s

// ===== LL (Link-Layer) performance knobs =====
// If MTU is larger than LL size the GATT packets need to be fragmented on the link layer
// default LL size is 27
// maximum LL size is 251
// Common BLE 4.2/5.0 DLE target is 244
inline constexpr uint16_t       LL_MIN_OCTETS                   =    27;    // 27..251
inline constexpr uint16_t       LL_CONS_OCTETS                  =   244;    // 27..251
inline constexpr uint16_t       LL_MAX_OCTETS                   =   251;    // 27..251

inline constexpr uint16_t       MIN_CHUNKSIZE                   =    20;

inline constexpr uint32_t       LL_DEFAULT_TIME_US              =   2120;

// BLE optimizations
static constexpr uint32_t intvl_us(uint16_t intvl)    
    { return ((uint32_t)intvl * 5000) / 4; } // is in units of microseconds    
static constexpr uint16_t us_intvl(uint32_t us)    
    { return (uint16_t)((us * 4) / 5000); } // is in units of 1.25ms
    static constexpr uint16_t tout_ms(uint32_t ms)
    { return (uint16_t)(ms / 10); }         // is in units of 10ms

// aggressive speed
inline constexpr const uint16_t MIN_BLE_INTERVAL_SPEED          = us_intvl( 7500);  // Minimum connection interval in microseconds 7.5ms to 4s
inline constexpr const uint16_t MAX_BLE_INTERVAL_SPEED          = us_intvl(10000);  // Maximum connection interval in µs 7.5ms to 4s
inline constexpr const uint16_t BLE_SLAVE_LATENCY_SPEED         = 0;               // Slave latency: number of connection events that can be skipped
inline constexpr const uint16_t BLE_SUPERVISION_TIMEOUT_SPEED   = tout_ms(4000);   // Supervision timeout in milli seconds 100ms to 32s, needs to be larger than 2 * (latency + 1) * (max_interval_ms)
// low power
inline constexpr const uint16_t MIN_BLE_INTERVAL_LOWPWR         = us_intvl(60000);  // 60ms
inline constexpr const uint16_t MAX_BLE_INTERVAL_LOWPWR         = us_intvl(120000); // 120ms
inline constexpr const uint16_t BLE_SLAVE_LATENCY_LOWPWR        = 8;               // can raise
inline constexpr const uint16_t BLE_SUPERVISION_TIMEOUT_LOWPWR  = tout_ms( 6000);  // 6s
// long range
inline constexpr const uint16_t MIN_BLE_INTERVAL_LONG_RANGE     = us_intvl(30000);  // 30ms
inline constexpr const uint16_t MAX_BLE_INTERVAL_LONG_RANGE     = us_intvl(60000);  // 60ms
inline constexpr const uint16_t BLE_SLAVE_LATENCY_LONG_RANGE    = 2;               // some dozing
inline constexpr const uint16_t BLE_SUPERVISION_TIMEOUT_LONG_RANGE = tout_ms(6000);// 6s
// balanced
inline constexpr const uint16_t MIN_BLE_INTERVAL_BALANCED       = us_intvl(15000);  // 15ms
inline constexpr const uint16_t MAX_BLE_INTERVAL_BALANCED       = us_intvl(30000);  // 30ms
inline constexpr const uint16_t BLE_SLAVE_LATENCY_BALANCED      = 2;               // light dozing
inline constexpr const uint16_t BLE_SUPERVISION_TIMEOUT_BALANCED = tout_ms(5000);  // 5s

// Assume we can push ~N PDUs per effective window; tune N if needed: 2/4/6
inline constexpr uint32_t       PDUS_PER_WINDOW                 = 4;

inline static constexpr uint16_t DISCARD_BACKOFF_AFTER          = 3;   // after these number of discards increase lkg
inline static constexpr uint16_t DISCARD_COOLDOWN_FACTOR_NUM    = 150; // x1.5 or x2, etc.
inline static constexpr uint16_t DISCARD_COOLDOWN_FACTOR_DEN    = 100;

// Optional: make the key distribution explicit (same idea as init_key/rsp_key in Bluedroid)
inline constexpr uint8_t        KEYDIST_ENC                     = 0x01;           // BLE_SPAIR_KEY_DIST_ENC
inline constexpr uint8_t        KEYDIST_ID                      = 0x02;           // BLE_SPAIR_KEY_DIST_ID
inline constexpr uint8_t        KEYDIST_SIGN                    = 0x04;           // BLE_SPAIR_KEY_DIST_SIGN
inline constexpr uint8_t        KEYDIST_LINK                    = 0x08;           // BLE_SPAIR_KEY_DIST_LINK

// ===== Tx backoff/throttle =====
inline constexpr uint16_t       PROBE_AFTER_SUCCESSES           = 32;             // 32-48 wait this many clean sends before probing for faster sends
inline constexpr uint16_t       PROBE_CONFIRM_SUCCESSES         = 12;             // 12-16 accept probe only after this many clean sends
inline constexpr uint32_t       PROBE_STEP_US                   = 5;              // absolute probe step in micro seconds
inline constexpr uint32_t       PROBE_STEP_PCT                  = 1;              // relative probe step of current interval (use the larger of the two)

inline constexpr uint8_t        LKG_ESCALATE_AFTER_FAILS        = 3;              // if LKG last known good fails ,this many times in a row, relax it !can not be Zero!
inline constexpr uint32_t       LKG_ESCALATE_NUM                = 103;            // ×1.03
inline constexpr uint32_t       LKG_ESCALATE_DEN                = 100;

inline constexpr int            COOL_SUCCESS_REQUIRED           = 64;             // successes before probing resumes after a backoff
inline constexpr uint32_t       ESCALATE_COOLDOWN_US            = 1000000;        // 1 s
inline constexpr uint32_t       TIMEOUT_BACKOFF_NUM             = 6;              // ×1.20 on timeout
inline constexpr uint32_t       TIMEOUT_BACKOFF_DEN             = 5;
inline constexpr uint32_t       CONGESTION_RETRY_COOLDOWN_US    = 5000;

// min interval guard
inline constexpr uint32_t       GUARD_NUM                       = 105;    // 105 for time adjustments

// retry behavior: 
inline constexpr uint8_t        EMSGSIZE_RETRY_MAX              = 3;
inline constexpr uint8_t        ETIMEOUT_RETRY_MAX              = 3;
inline constexpr uint8_t        SOFTWARE_RETRY_MAX              = 3;
inline constexpr uint8_t        RECOVER_RETRY_MAX               = 6;

// Notifications failures
// inline constexpr uint8_t        MAX_SETVALUE_FAILS              = 3;
inline constexpr uint8_t        MAX_NOTIFY_FAILS                = 3;

static constexpr uint32_t       FLUSH_MAX_WAIT_MS               = 250;

/******************************************************************************************************/
/* Structures */
/******************************************************************************************************/

enum class Mode {
  Fast,
  LowPower,
  LongRange,
  Balanced
};

enum class Security { 
  None, 
  JustWorks, 
  PasskeyDisplay
};

enum class PumpMode { 
  Polling, 
  Task 
};

enum class TxState{
  Waiting,    // Ring buffer empty or no connection
  Staging,    // Take from buffer and Setvalue/Notify
  Pending,    // Waiting for status Code
  Recovering, // Resend staged data
  Discarding  // Discard staged data
};

enum class TxSuccess{
  NotSet,
  Success,
  MessageSizeTooBig,
  Congestion,
  Timeout,
  Disconnected,
  SoftwareError,
  Unclassified
};

/***************************************************************************************************/
/* Device Driver */
/***************************************************************************************************/

class BLESerial : public Stream {
// ================================================================================================
public:
// ================================================================================================
  // Provide a nested alias so user code can refer to BLESerial::Mode
  using Mode = ::Mode;
  // Provide a nested alias so user code can refer to BLESerial::Security
  using Security = ::Security;
  // Provide a nested alias so user code can refer to BLESerial::PumpMode
  using PumpMode = ::PumpMode;
  // Provide a nested alias so user code can refer to BLESerial::TxState
  using TxState = ::TxState;

  // ----------------------------------------------------------------------------------------------
  // Lifecycle
  // ----------------------------------------------------------------------------------------------
  BLESerial() = default;
  bool     begin(Mode      mode       = Mode::Fast,
              const char*  deviceName = "BLESerialDevice",
              Security     secure     = Security::None);  
           // init stack, create service, start advertising
  void     end();                                       
           // stop advertising, dispose service/server

  // ----------------------------------------------------------------------------------------------
  // Stream API (data I/O)
  // ----------------------------------------------------------------------------------------------
  int      available() override;                        
           // RX bytes ready (Stream contract)
  void     flush() override;                            
           // drain TX ring to link
  int      readAvailable();                             
           // Alias to available()
  int      read() override;                             
           // 1 byte from RX buffer, returns the byte read or -1 if none
  int      read(uint8_t* dst, size_t n);                
           // helper: read up to n bytes, returns number of bytes read
  int      peek() override;                             
           // next byte without consuming (popping from buffer), returns byte or -1 if none
  int      peek(uint8_t* dst, size_t n);                
           // helper: preview up to n bytes without consuming, returns number of bytes peeked
  bool     writeReady() const;
           // Returns true when producers may enqueue more data without violating flow control.
           //   - TX buffer is not locked by high-water ( !txLocked )
           //   - A client is subscribed ( isSubscribed() )
  size_t   write(uint8_t b) override;
           // enqueue single byte to TX, returns bytes written
  size_t   write(const uint8_t* b, size_t n) override;  
           // enqueue block, returns bytes written
  size_t   write(const char* str) { return write(reinterpret_cast<const uint8_t*>(str), strlen(str)); }
           // enqueue C-string, returns bytes written
  size_t   write(const String& s) { return write(reinterpret_cast<const uint8_t*>(s.c_str()), s.length()); }
           // enqueue String, returns bytes written
  size_t   print(const char* str) { return write(reinterpret_cast<const uint8_t*>(str), strlen(str)); }
           // alias to enqueue C-string (alias to write), returns bytes written
  size_t   print(const String& s) { return write(reinterpret_cast<const uint8_t*>(s.c_str()), s.length()); }
           // alias to enqueue String (alias to write), returns bytes written
  size_t   println(const char* str) { size_t r=write(reinterpret_cast<const uint8_t*>(str), strlen(str)); write('\r'); write('\n'); return r+2; }
           // enqueue C-string with CRLF, returns total bytes written
  size_t   println(const String& s) {
              size_t n = write(reinterpret_cast<const uint8_t*>(s.c_str()), s.length());
              n += write(reinterpret_cast<const uint8_t*>("\r\n"), 2);
              return n; }  
  size_t   printf(const char *fmt, ...) __attribute__((format(printf, 2, 3)));
           // formatted print to TX buffer
  size_t   writeTimeout(const uint8_t* p, size_t n, uint32_t timeoutMs = 50);
           // blocking write with timeout, returns number of bytes written
           //   ensure all bytes are queued or timeout expires
  size_t   writeAvailable() const { return txBuf.capacity() - txBuf.available(); }
           // bytes free in TX buffer

  // ----------------------------------------------------------------------------------------------
  // Link / radio adjustments
  // ----------------------------------------------------------------------------------------------
  bool     requestMTU(uint16_t newMtu);                 
           // request a different MTU from client
           // Note: After negotiation, txChunkSize and timing are updated when the MTU change event arrives.
  void     setPower(int8_t dBm, NimBLETxPowerType scope = PWR_ALL); 
           // change radio power for adv/scan/conn/all 3

  // ----------------------------------------------------------------------------------------------
  // Pump / scheduling (Polling vs Task)
  // ----------------------------------------------------------------------------------------------
  void     update();                                    
           // In polling mode we need to update driver regularly in the main loop
  PumpMode getPumpMode() const { return pumpMode; }
  #ifdef ARDUINO_ARCH_ESP32
    void   setPumpMode(PumpMode m); // selects Polling or Task mode
  #else
    void   setPumpMode(PumpMode m) { pumpMode = PumpMode::Polling; } // no Task on non-ESP32
  #endif

  // ----------------------------------------------------------------------------------------------
  // Logging / diagnostics
  // ----------------------------------------------------------------------------------------------
  void     setLogLevel(uint8_t lvl) { logLevel = lvl; }
  uint8_t  getLogLevel() const { return logLevel; }
  void     printStats(Stream &out);
  void     printStats() { printStats(Serial); }
  void     clearStats();
  const char* phyToStr() const;
  inline const char* getPhy() const { return phyToStr(); }

  // ----------------------------------------------------------------------------------------------
  // Event hooks (callbacks) - run in NimBLE context, keep handlers light
  // ----------------------------------------------------------------------------------------------
  void     setOnClientConnect(std::function<void(const std::string& addr)> cb) { onClientConnect = std::move(cb); }
  void     setOnClientDisconnect(std::function<void(const std::string& addr, uint16_t reason)> cb) { onClientDisconnect = std::move(cb); }
  void     setOnMtuChanged(std::function<void(uint16_t mtu)> cb) { onMtuChanged = std::move(cb); }
  void     setOnSubscribeChanged(std::function<void(bool subscribed)> cb) { onSubscribeChanged = std::move(cb); }
  void     setOnDataReceived(std::function<void(const uint8_t* data, size_t len)> cb) { onDataReceived = std::move(cb); }
  void     setOnRxOverflow(std::function<void(size_t lost)> cb) { onRxOverflow = std::move(cb); }

  // ----------------------------------------------------------------------------------------------
  // State Machine
  // ----------------------------------------------------------------------------------------------
  void     advanceTxStateMachine();
  TxState  txWaiting();
  TxState  txStaging();
  TxState  txPending();
  TxState  txRecovering();
  TxState  txDiscarding();

  // Handlers for pending status classification
  void     onNotSet();
  void     onTxSuccess();
  void     onMessageTooBig();
  void     onCongestion();
  void     onTimeout();
  void     onDisconnected();
  void     onSoftwareError();
  void     onUnclassified();

  // ----------------------------------------------------------------------------------------------
  // State / statistics accessors
  // ----------------------------------------------------------------------------------------------
  bool     isConnected()   const { return deviceConnected && connHandle != BLE_HS_CONN_HANDLE_NONE; }
  bool     isSubscribed()  const { return deviceConnected && clientSubscribed; }
  uint16_t getMtu()        const { return mtu; }
  Mode     getMode()       const { return mode; }
  uint32_t getBytesRx()    const { return bytesRx; }
  uint32_t getBytesTx()    const { return bytesTx; }
  uint32_t getRxDrops()    const { return rxDrops; }
  uint32_t getTxDrops()    const { return txDrops; }
  uint32_t getInterval()   const { return sendIntervalUs; }
  uint32_t getLkgInterval() const { return lkgIntervalUs; }
  uint32_t getMinInterval() const { return minSendIntervalUS; }
  int16_t  getRSSI()       const { return rssiAvg; }
  const std::string& getMac() const { return deviceMac; }
  size_t   getTxUsed()     const { return txBuf.available(); }
  size_t   getRxUsed()     const { return rxBuf.available(); }
  size_t   getTxCapacity() const { return txBuf.capacity(); }
  size_t   getRxCapacity() const { return rxBuf.capacity(); }
  size_t   getTxFree()     const { return txBuf.capacity() - txBuf.available(); }
  size_t   getRxFree()     const { return rxBuf.capacity() - rxBuf.available(); }
  bool     isEncrypted()   const { return linkEncrypted; }
  uint16_t getllTxOctets() const { return llTxOctets; }
  uint16_t getllTxTimeUs() const { return llTxTimeUs; }
  uint16_t getllRxOctets() const { return llRxOctets; }
  uint16_t getllRxTimeUs() const { return llRxTimeUs; }
  uint16_t getChunkSize()  const { return txChunkSize; }
  uint16_t getLowWaterMark() const { return lowWater; }
  uint16_t getHighWaterMark() const { return highWater; }
  // Effective window baseline (connection-event spacing considering latency)
  // Returns (connLatency + 1) * connIntervalUs; 0 if not connected or interval unknown.
  uint32_t getWindowBaselineUs() const { return (connIntervalUs == 0) ? 0u : ((uint32_t)(connLatency + 1) * connIntervalUs); }
  // Per-event share (effective connection-event window divided by PDUS_PER_WINDOW)
  // Returns ceil(((connLatency + 1) * connIntervalUs) / PDUS_PER_WINDOW); 0 if interval unknown.
  uint32_t getPerEventShareUs() const { return perEventShareUs; }

// ================================================================================================
private:
// ================================================================================================
  // ===== BLE primitives =====
  NimBLEServer*           server      = nullptr;
  NimBLEService*          service     = nullptr;
  NimBLECharacteristic*   txChar      = nullptr;
  NimBLECharacteristic*   rxChar      = nullptr;
  NimBLEAdvertising*      advertising = nullptr;
  NimBLEAdvertisementData advData;
  NimBLEAdvertisementData scanData;

  // Static GAP handler
  static int gapEventHandler(struct ble_gap_event* ev, void* arg);
  // Active instance used by the static GAP handler
  static BLESerial* active;

  // GAP/GATT state
  volatile bool     deviceConnected   = false;
  volatile bool     clientSubscribed  = false;
  volatile uint16_t connHandle        = BLE_HS_CONN_HANDLE_NONE;
  std::string       peerAddr;
  std::string       deviceMac;
  volatile bool     linkEncrypted     = false;

  // PHY/DLE
  volatile uint8_t  codedScheme       = 2;   // 2 or 8
  volatile uint16_t llTxOctets        = LL_MAX_OCTETS;  // target octets
  volatile uint16_t llRxOctets        = LL_MAX_OCTETS;  // target octets
  volatile uint16_t llTxTimeUs        = 2120; // 1M default
  volatile uint16_t llRxTimeUs        = 2120; // 1M default
  volatile bool     phyIs2M           = false;
  volatile bool     phyIsCoded        = false;

  volatile uint16_t connItvlUnits  = 0;   // 1.25 ms units (HCI units)
  volatile uint16_t connLatency    = 0;   // as seen in GAP
  volatile uint16_t supervisionTimeoutMS = 0; // as seen in GAP
  volatile uint32_t connIntervalUs = 0;   // convenience cache: microseconds

  // Desired link settings that we request from controller/peer
  uint8_t           desiredPhyMask    = BLE_GAP_LE_PHY_1M_MASK; // corrected macro name
  uint8_t           desiredCodedScheme= 0;        // 0, 2, or 8 (desired)
  uint16_t          desiredllTxOctets = LL_MAX_OCTETS;
  uint16_t          desiredllTxTimeUs = LL_DEFAULT_TIME_US;     // conservative time cap
  uint16_t          desiredllRxOctets = LL_MAX_OCTETS;
  uint16_t          desiredllRxTimeUs = LL_DEFAULT_TIME_US;     // conservative time cap

  // MTU / chunking
  volatile uint16_t mtu               = 23;
  volatile uint16_t txChunkSize       = MIN_CHUNKSIZE;
 
  // Recovery Retries
  volatile int      emsgSizeRetries   = 0;
  volatile uint8_t  timeoutRetries    = 0;
  volatile uint8_t  softwareRetries   = 0;
 
  // TX pacing/backoff
  volatile uint32_t sendIntervalUs    = 200;
  volatile uint32_t minSendIntervalUS = 200; // computed floor based on MTU/LL timing
  volatile uint32_t lkgIntervalUs     = 0;
  volatile uint32_t perEventShareUs   = 0;
  // 
  volatile bool     probing           = false;
  volatile uint16_t probeSuccesses    = 0; // number of consecutive successes during probing
  volatile uint8_t  lkgFails          = 0; // number of consecutive LKG failures
  volatile bool     recentlyBackedOff = false;
  volatile uint16_t coolDowns         = 0; // number of cooldowns triggered
  volatile uint16_t successStreak     = 0; // tx successes since last failure
  volatile uint16_t discardStreak     = 0; // consecutive discard events

  volatile uint32_t txSuccessCount    = 0; // number of successful notify/indicate completions
  volatile uint32_t msgSizeCount      = 0; // EMSGSIZE occurrences
  volatile uint32_t congestionCount   = 0; // recoverable congestion events (EAGAIN/EBUSY/...)
  volatile uint32_t timeoutCount      = 0; // timeout events
  volatile uint32_t disconnectCount   = 0; // ENOTCONN/EOS events
  volatile uint32_t unclassifiedCount = 0; // any other non-classified errors
  volatile uint32_t softwareErrorCount= 0; // number of EINVAL, EAPP, EBADDATA, ECONTROLLER, EUNKNOWN events observed
  
  // RX book-keeping
  volatile size_t   bytesRx           = 0;
  volatile size_t   rxDrops           = 0;
  volatile uint32_t lastRxUs          = 0;

  // Notifications
  // size_t         setValueFailedCount = 0;
  size_t            notifyFailedCount   = 0;

  // Buffers and flow control
  RingBuffer<uint8_t, 4096> rxBuf;         // 4k default, 8k for steady high rate, 2k for tight RAM
  RingBuffer<uint8_t, 4096> txBuf;         // 4k default, 8k for steady high rate, 2k for tight RAM
  size_t            highWater         = 0;
  size_t            lowWater          = 0;
  volatile size_t   pendingLen        = 0;
  uint8_t           pending[BLE_SERIAL_MAX_GATT]{};
  volatile bool     txLocked          = false; // prevent producers when high water reached
  // TX book-keeping
  volatile uint32_t lastTxUs          = 0;
  volatile TxSuccess txSuccess        = TxSuccess::NotSet;
  volatile size_t   bytesTx           = 0;
  volatile size_t   txDrops           = 0;

  // On Status Code
  volatile int      onStatusCode      = 0;

  int8_t            powerAdv          = BLE_TX_DB0;
  int8_t            powerScan         = BLE_TX_DB0;
  int8_t            powerConn         = BLE_TX_DB0;

  // Configuration
  Mode              mode              = Mode::Fast;
  TxState           txState           = TxState::Waiting;
  Security          secure            = Security::None;
  uint8_t           logLevel          = INFO;

  // Security
  uint32_t          passkey           = 0; // stores the currently displayed/generated 6-digit passkey

  // RSSI polling
  volatile int8_t   rssiRaw           = 0;
  volatile int8_t   rssiAvg           = 0;
  volatile uint32_t lastRSSIMs        = 0;
  volatile uint32_t lastRSSIActionMs  = 0;

  // Congestion
  volatile uint32_t lastCongestionAtUs= 0; // timestamp of last congestion/backpressure

  // TX pump and helpers
  void              pumpTx();
                    // main TX pump function for polling mode
  static uint16_t   computeTxChunkSize(uint16_t mtu, uint16_t llTxOctets, Mode mode, Security security, size_t txCapacity);
                    // computes chunk size to minimize fragmentation
  uint32_t          computeSendIntervalUs(uint16_t chunkSize);
                    // computes send interval based on chunk size and negotiated link timing
  void              updateWaterMarks(size_t chunkSize);
                    // compute low /  high water mark for TX flow control
  void              updateTxTiming();
                    // recompute TX timing parameters
  uint32_t          estimate_LL_PDUTimeUs(uint16_t llOctets, bool phy2M, bool phyCoded, uint8_t codedScheme);
  inline uint32_t   estimate_LL_PDUTimeUs()                { return estimate_LL_PDUTimeUs(static_cast<uint16_t>(llTxOctets), static_cast<bool>(phyIs2M), static_cast<bool>(phyIsCoded), static_cast<uint8_t>(codedScheme)); }
  inline uint32_t   estimate_LL_PDUTimeUs(uint16_t octets) { return estimate_LL_PDUTimeUs(octets, static_cast<bool>(phyIs2M), static_cast<bool>(phyIsCoded), static_cast<uint8_t>(codedScheme)); }
                    // computes estimted LL PDU time based on current PHY/DLE settings
  void              adjustLink();   
                    // Link adaptation based on RSSI
  static uint16_t   micBytes(Security sec);
                    // returns MIC bytes for given security level
  uint32_t          computePerEventShareUs(uint32_t connInt, uint16_t connLat, uint32_t pdus_per_window);
                    // recompute per-event share based on current conn params

    // Background TX task helpers (ESP32)
  #ifdef ARDUINO_ARCH_ESP32
  // Instance spinlock for TX state
  portMUX_TYPE      txMux = portMUX_INITIALIZER_UNLOCKED;
  static TaskHandle_t rssiTaskHandle;
  static TaskHandle_t txTaskHandle;
  #endif
  // Task functions
  static void       RSSITask(void* arg);
  static void       pumpTxTask(void* arg);

  // Task lifecycle helpers
  void             startTxTask();
  void             stopTxTask();
  void             suspendTxTask();
  void             wakeTxTask();
  void             startRSSITask();
  void             stopRSSITask();
  void             suspendRSSITask();
  void             wakeRSSITask();
  // Sub-ms remainder threshold: below this use microsecond delay and not vTaskDelay
  static constexpr uint32_t TASK_DELAY_THRESHOLD_US = 800;   // tune (500–1500)

  // Current TX pump mode (default: Polling). Defined for all platforms for API consistency.
  volatile PumpMode pumpMode = PumpMode::Polling;

  // Callbacks
  class ServerCallbacks;
  class RxCallbacks;
  class TxCallbacks;
  friend class ServerCallbacks;
  friend class RxCallbacks;
  friend class TxCallbacks;

  std::function<void(const std::string& addr)> onClientConnect;
  std::function<void(const std::string& addr, uint16_t reason)> onClientDisconnect;
  std::function<void(uint16_t mtu)> onMtuChanged;
  std::function<void(bool subscribed)> onSubscribeChanged;
  std::function<void(const uint8_t* data, size_t len)> onDataReceived; // raw RX callback
  std::function<void(size_t lost)> onRxOverflow; // invoked when RX ring overwrites oldest data

};

#endif // BLE_SERIAL_H

