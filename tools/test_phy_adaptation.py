#!/usr/bin/env python3
"""Exercise production PHY decision and completion-state code with a fake clock/controller."""
from pathlib import Path
import subprocess
import tempfile

repo = Path(__file__).resolve().parents[1]
source = (repo / 'src/BLESerial.cpp').read_text()
header = (repo / 'src/BLESerial.h').read_text()
constants = '\n'.join(line for line in header.splitlines() if line.startswith('inline constexpr') and 'RSSI_' in line)
adjust = source.split('void BLESerial::adjustLink() {', 1)[1].split('// ===== State Machine', 1)[0]
reset = source.split('void BLESerial::resetRSSIDecision() {', 1)[1].split('void BLESerial::resetCongestionWindow()', 1)[0]
completion = source.split('const auto &p = ev->phy_updated;', 1)[1].split('// Early return if PHY/coding unchanged', 1)[0]
preamble = r'''
#include <cassert>
#include <cstdint>
#include <cstdio>
#include <functional>
#define BLE_GAP_LE_PHY_1M_MASK 1
#define BLE_GAP_LE_PHY_2M_MASK 2
#define BLE_GAP_LE_PHY_CODED_MASK 4
#define BLE_GAP_LE_PHY_CODED_S2 1
#define BLE_GAP_LE_PHY_CODED_S8 2
#define BLE_HCI_LE_PHY_2M 2
#define BLE_HCI_LE_PHY_CODED 3
#define LOG_LEVEL_INFO 2
#define LOG_LEVEL_WARN 1
uint32_t nowMs = 10000;
uint32_t millis() { return nowMs; }
int calls = 0, result = 268;
std::function<void()> onRequest;
int8_t signal = -40;
int ble_gap_conn_rssi(uint16_t, int8_t* rssi) { *rssi = signal; return 0; }
int ble_gap_set_prefered_le_phy(uint16_t, uint8_t, uint8_t, uint16_t) {
 ++calls;
 if (onRequest) onRequest();
 return result;
}
int logGetLevel() { return 3; }
void diagnostic(int, const char*, const char*, ...) {}
struct BLESerial {
 enum class Mode { Fast, LongRange, LowPower, Balanced };
 Mode mode = Mode::Fast;
 bool connected = true, phyUpdatePending = false, phyIs2M = false, phyIsCoded = false;
 uint16_t connHandle = 1;
 uint8_t codedScheme = 0, desiredPhyMask = 1, desiredCodedScheme = 0;
 uint8_t rssiCandidatePhyMask = 0, rssiCandidateCodedScheme = 0, rssiCandidateCount = 0;
 int8_t rssiRaw = 0, rssiAvg = 0;
 uint32_t lastRSSIMs = 0, lastRSSIActionMs = 0;
 bool isConnected() { return connected; }
 void resetRSSIDecision();
 void adjustLink();
};
struct Completion { uint16_t conn_handle; int status; uint8_t tx_phy, rx_phy; };
'''
tests = r'''
void samples(BLESerial& s, int n) { for(int i = 0; i < n; ++i) { s.adjustLink(); nowMs += 500; } }
int main() {
 BLESerial s;
 samples(s, 6);
 assert(calls == 1 && !s.phyUpdatePending && s.desiredPhyMask == 1);
 auto failedAt = s.lastRSSIActionMs;
 nowMs = failedAt + 3999; s.adjustLink();
 assert(calls == 1 && s.rssiCandidateCount == 0);
 nowMs = failedAt + 4000; samples(s, 5); assert(calls == 1);
 samples(s, 1); assert(calls == 2);
 // Successful submission stays pending beyond the cooldown.
 result = 0; nowMs = s.lastRSSIActionMs + 4000; samples(s, 6);
 assert(calls == 3 && s.phyUpdatePending);
 samples(s, 40); assert(calls == 3 && s.rssiCandidateCount == 0);
 // An unrelated completion cannot release the pending request.
 complete(s, Completion{2, 0, 2, 2}); assert(s.phyUpdatePending);
 // Failed completion releases pending and starts a fresh full cooldown.
 complete(s, Completion{1, 1, 1, 1}); assert(!s.phyUpdatePending);
 auto completedAt = s.lastRSSIActionMs;
 nowMs = completedAt + 3999; s.adjustLink(); assert(calls == 3);
 nowMs = completedAt + 4000; samples(s, 6); assert(calls == 4 && s.phyUpdatePending);
 complete(s, Completion{1, 0, 2, 2}); assert(!s.phyUpdatePending && s.phyIs2M);
 signal = -100; s.rssiAvg = -100;
 nowMs = s.lastRSSIActionMs + 3999; s.adjustLink(); assert(calls == 4);
 nowMs = s.lastRSSIActionMs + 4000; samples(s, 3); assert(calls == 5);
 // Cooldown arithmetic remains correct across millis() wraparound.
 signal = -40;
 BLESerial wrap; wrap.lastRSSIActionMs = UINT32_MAX - 1000;
 nowMs = 1000; wrap.adjustLink(); assert(wrap.rssiCandidateCount == 0);
 nowMs = 3000; wrap.adjustLink(); assert(wrap.rssiCandidateCount == 1);
 // Completion may occur synchronously before the submission returns.
 BLESerial immediate;
 onRequest = [&]() {
   assert(immediate.phyUpdatePending);
   nowMs += 250;
   complete(immediate, Completion{1, 0, 2, 2});
 };
 nowMs = 20000; samples(immediate, 6);
 assert(!immediate.phyUpdatePending && immediate.phyIs2M);
 assert(immediate.lastRSSIActionMs == nowMs - 500);
 onRequest = {};
 // Completion after disconnect must not change negotiated state.
 immediate.connected = false;
 complete(immediate, Completion{1, 0, 1, 1}); assert(immediate.phyIs2M);
 // Coded-PHY downgrades follow the same failure holdoff.
 BLESerial coded; coded.mode = BLESerial::Mode::LongRange;
 coded.phyIsCoded = true; coded.codedScheme = 2;
 coded.desiredPhyMask = 4; coded.desiredCodedScheme = 2;
 signal = -100; result = 268;
 auto beforeCoded = calls;
 samples(coded, 3); assert(calls == beforeCoded + 1);
 assert(!coded.phyUpdatePending && coded.desiredCodedScheme == 2);
 nowMs = coded.lastRSSIActionMs + 3999; coded.adjustLink();
 assert(calls == beforeCoded + 1 && coded.rssiCandidateCount == 0);
 puts("PASS: immediate failure cooldown, fresh confirmation, pending exclusion, connection filtering, completion cooldowns, millis rollover");
}
'''
cpp = preamble + constants + '\nvoid BLESerial::resetRSSIDecision() {' + reset + '\nvoid BLESerial::adjustLink() {' + adjust + '\nvoid complete(BLESerial& s, const Completion& p) {\n' + completion.replace('return 0;', 'return;') + '\n}\n' + tests
with tempfile.TemporaryDirectory(prefix="bleserial-phy-test-") as directory:
    cpp_path = Path(directory) / "test.cpp"
    binary_path = Path(directory) / "test"
    cpp_path.write_text(cpp)
    subprocess.run(['g++', '-std=c++17', '-Wall', '-Wextra', '-Wno-unused-variable', str(cpp_path), '-o', str(binary_path)], check=True)
    subprocess.run([str(binary_path)], check=True)
