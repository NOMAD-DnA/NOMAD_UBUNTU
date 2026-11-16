#include "nomad_control_cpp/vesc_telemetry.hpp"
#include <cmath>
#include <cstring>
#include <iostream>
#include <stdexcept>
using namespace std;
using namespace nomad;
static int checks = 0;
static void expect(bool v, const char* why) { ++checks; if (!v) throw runtime_error(why); }
static bool named(int code, const char* name) { return strcmp(vescFaultName(code), name) == 0; }
int main() {
    try {
        // FW 5.03 mc_fault_code order (bldc datatypes.h at tag 5.03).
        expect(named(0, "NONE") && named(1, "OVER_VOLTAGE") && named(2, "UNDER_VOLTAGE"), "fault names 0..2");
        expect(named(4, "ABS_OVER_CURRENT") && named(5, "OVER_TEMP_FET") && named(6, "OVER_TEMP_MOTOR"), "fault names 4..6");
        expect(named(10, "BOOTING_FROM_WATCHDOG_RESET") && named(18, "UNBALANCED_CURRENTS"), "fault names 10 and 18");
        expect(named(26, "ENCODER_MAGNET_TOO_STRONG"), "last FW 5.03 fault name");
        expect(named(27, "UNKNOWN") && named(1000, "UNKNOWN"), "codes past FW 5.03 are unknown");
        expect(named(-1, "NO_TELEMETRY"), "-1 = no telemetry yet (controller default)");
        expect(abs(inputPowerW(16.4, 1.5) - 24.6) < 1e-9 && inputPowerW(16.4, -2.0) < 0, "power = V x input A, negative on regen");
        VescDisplay d;
        expect(!d.received && d.controller_id == -1 && d.mos_temp_c[2] == 0.0, "display defaults before telemetry");
        cout << checks << " vesc telemetry checks PASS\n";
        return 0;
    } catch (const exception& e) { cerr << e.what() << "\n"; return 1; }
}
