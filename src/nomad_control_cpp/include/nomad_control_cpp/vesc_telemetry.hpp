#pragma once

#include <array>
#include <cstdint>

namespace nomad {

// VESC FW 5.03 mc_fault_code names, in enum order (vedderb/bldc tag 5.03, datatypes.h mc_fault_code).
// The vesc_msgs VescState constants name only 0..6.
inline const char* vescFaultName(int code) {
    static constexpr const char* kNames[] = {
        "NONE", "OVER_VOLTAGE", "UNDER_VOLTAGE", "DRV", "ABS_OVER_CURRENT", "OVER_TEMP_FET", "OVER_TEMP_MOTOR",
        "GATE_DRIVER_OVER_VOLTAGE", "GATE_DRIVER_UNDER_VOLTAGE", "MCU_UNDER_VOLTAGE", "BOOTING_FROM_WATCHDOG_RESET",
        "ENCODER_SPI", "ENCODER_SINCOS_BELOW_MIN_AMPLITUDE", "ENCODER_SINCOS_ABOVE_MAX_AMPLITUDE", "FLASH_CORRUPTION",
        "HIGH_OFFSET_CURRENT_SENSOR_1", "HIGH_OFFSET_CURRENT_SENSOR_2", "HIGH_OFFSET_CURRENT_SENSOR_3",
        "UNBALANCED_CURRENTS", "BRK", "RESOLVER_LOT", "RESOLVER_DOS", "RESOLVER_LOS", "FLASH_CORRUPTION_APP_CFG",
        "FLASH_CORRUPTION_MC_CFG", "ENCODER_NO_MAGNET", "ENCODER_MAGNET_TOO_STRONG"};
    if (code < 0) return "NO_TELEMETRY";
    if (code >= static_cast<int>(sizeof(kNames) / sizeof(kNames[0]))) return "UNKNOWN";
    return kNames[code];
}

// VESC values the controller only reports (status_json, vesc_trace); none of them feeds a control decision.
// input_current_a is the VESC estimate from the phase currents (the 75_300 has no DC-bus shunt). The Ah/Wh
// counters and the tachometer count from VESC power-up; Ah/Wh only while |motor current| > 1 A.
struct VescDisplay {
    bool received = false;
    double input_current_a = 0.0;
    double duty = 0.0;  // signed modulation, -1..1; |duty| near the VESC max duty (0.95) = voltage limit
    double id_a = 0.0, iq_a = 0.0, vd_v = 0.0, vq_v = 0.0;
    std::array<double, 3> mos_temp_c{};
    double charge_drawn_ah = 0.0, charge_regen_ah = 0.0, energy_drawn_wh = 0.0, energy_regen_wh = 0.0;
    std::int32_t tacho = 0, tacho_abs = 0;  // 6 counts per electrical revolution (HFI/observer estimate)
    std::int32_t controller_id = -1;
};

// Battery power from the VESC input voltage and estimated input current (negative while regenerating).
inline double inputPowerW(double voltage_v, double input_current_a) { return voltage_v * input_current_a; }

}  // namespace nomad
