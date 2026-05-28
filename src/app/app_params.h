#pragma once
// app_params.h -- Params struct, EEPROM load/save, threshold publishing.
// QM / non-safety layer. May include safety_defs.h for constants.
#include "../safety/safety_defs.h"

struct Params {
    uint32_t magic;
    uint8_t  version;
    uint32_t window_s;
    float    omega_stable_dps;
    float    omega_instant_dps;
    uint32_t instant_hold_ms;
    float    spike_accel_g;
    uint32_t eval_period_ms;
    uint32_t motion_samples_max;
    uint32_t shock_samples_max;
    uint32_t clean_streak_needed;
    float    diverse_mean_tol_g;
    float    spread_max_deg;
    float    anchor_max_deg;
    float    gyro_range_fault_dps;
    uint32_t stale_fault_ticks;
    float    dual_diverge_deg;
};

extern Params g_params;

// Threshold crossing: app publishes validated params to the safety layer.
// Safety layer reads from g_safety_thresholds (defined in app_params.cpp).
extern volatile SafetyThresholds g_safety_thresholds;

bool params_valid(const Params &p);
bool params_load();
void params_save();
void app_publish_thresholds();   // copies g_params → g_safety_thresholds + refresh CRC
