// safety_stability.cpp -- window buffer, spread metric, FSM, evaluate_stability (ASIL D/SIL 3).
// Independence rule: no app/ includes.

#include "safety_stability.h"
#include "safety_monitor.h"
#include "safety_isr.h"
#include <math.h>
#include <string.h>
#include <Arduino.h>

// --- Window buffers (extern-linked to safety_monitor.cpp's compute_safety_crc) --
static Tilt     s_tilt_buf    [WINDOW_SAMPLES_MAX];
static float    s_accel_mag_buf[WINDOW_SAMPLES_MAX];
uint32_t        s_buf_head          = 0U;
uint32_t        s_buf_count         = 0U;
uint32_t        s_spike_gyro_count  = 0U;
uint32_t        s_spike_accel_count = 0U;
static uint8_t  s_gyro_spike_slot [WINDOW_SAMPLES_MAX];
static uint8_t  s_accel_spike_slot[WINDOW_SAMPLES_MAX];

// --- Safety status (read-only by app layer) -----------------------------------
volatile SafetyStatus g_safety_status;

// --- Stability state ----------------------------------------------------------
StabFSM  s_stab_fsm         = SFST_INIT;
uint8_t  s_fsm_stable_cnt   = 0U;
char     s_fsm_state_str[5] = "INIT";
Tilt     s_stable_anchor    = {0.0f, 0.0f, 0.0f};
bool     s_anchor_valid     = false;
bool     s_imu_b_degraded   = false;
uint8_t  s_b_fail_count     = 0U;
float    s_peak_omega_latch  = 0.0f;
uint32_t s_instant_hold_until= 0U;

// --- Last-value state (written each eval cycle, read by app HTTP layer) ------
float s_last_omega_dps  = 0.0f;
float s_last_roll_deg   = 0.0f;
float s_last_pitch_deg  = 0.0f;
float s_last_roll_b_deg = 0.0f;
float s_last_pitch_b_deg= 0.0f;
float s_last_spread_deg = 0.0f;
char  s_why_str[12]     = "OK";

// --- Clean streak / stale counters -------------------------------------------
uint8_t  s_gyro_clean    = 0U;
uint8_t  s_accel_clean   = 0U;
uint8_t  s_diverse_clean = 0U;
uint8_t  s_timing_clean  = 0U;
uint32_t s_comm_stale_a  = 0U;
uint32_t s_comm_stale_b  = 0U;
uint32_t s_frozen_count  = 0U;

// --- Helpers ------------------------------------------------------------------
float    clamp_f     (float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }
uint32_t safe_min_u32(uint32_t a, uint32_t b)      { return a < b ? a : b; }
uint32_t g_window_samples()                        { return g_safety_thresholds.window_s * STABILITY_HZ; }

// Fault mask with IMU-B faults suppressed while in degraded single-channel mode.
// FAULT_DUAL_DIVERGE also suppressed: fires during ~18 ms intermittent window
// when B first goes stale (corrupted Mahony-B), and cannot self-clear once B
// is absent (can_check gate requires FAULT_IMU_B_COMM clear).
uint16_t effective_faults()
{
    uint16_t m = fault_mask_get();
    if (s_imu_b_degraded) m &= ~(uint16_t)(FAULT_IMU_B_COMM | FAULT_DUAL_DIVERGE);
    return m;
}

// --- Stability algorithm ------------------------------------------------------

bool diverse_stable()
{
    uint32_t ws = g_window_samples();
    if (s_buf_count < ws) return false;
    float sum = 0.0f;
    for (uint32_t i = 0U; i < ws; ++i)
        sum += s_accel_mag_buf[i];
    return fabsf(sum / (float)ws - 1.0f) < g_safety_thresholds.diverse_mean_tol_g;
}

// MA-3: yaw excluded — gyro-integrated yaw conflates rotation with instability.
float tilt_distance_deg(const Tilt &a, const Tilt &b)
{
    float dr = a.roll_deg  - b.roll_deg;
    float dp = a.pitch_deg - b.pitch_deg;
    return sqrtf(dr*dr + dp*dp);
}

// MA-4: centroid-based spread; order-independent, oldest sample no longer biases.
float compute_window_tilt_spread()
{
    if (s_buf_count < 2U) return 0.0f;
    uint32_t ws     = g_window_samples();
    uint32_t count  = safe_min_u32(s_buf_count, ws);
    uint32_t oldest = (s_buf_head + ws - count) % ws;

    float sum_r = 0.0f, sum_p = 0.0f;
    for (uint32_t i = 0U; i < count; ++i) {
        const Tilt &t = s_tilt_buf[(oldest + i) % ws];
        sum_r += t.roll_deg;
        sum_p += t.pitch_deg;
    }
    float mean_r = sum_r / (float)count;
    float mean_p = sum_p / (float)count;

    float mx = 0.0f;
    for (uint32_t i = 0U; i < count; ++i) {
        const Tilt &t = s_tilt_buf[(oldest + i) % ws];
        float dr = t.roll_deg  - mean_r;
        float dp = t.pitch_deg - mean_p;
        float d  = sqrtf(dr*dr + dp*dp);
        if (d > mx) mx = d;
    }
    return mx;
}

void push_sample(const Tilt &t, float omega_dps, float accel_mag_g)
{
    uint32_t slot = s_buf_head;
    s_tilt_buf[slot]      = t;
    s_accel_mag_buf[slot] = accel_mag_g;

    if (s_gyro_spike_slot[slot])  s_spike_gyro_count--;
    s_gyro_spike_slot[slot] = (omega_dps > g_safety_thresholds.omega_stable_dps) ? 1U : 0U;
    s_spike_gyro_count += s_gyro_spike_slot[slot];

    if (s_accel_spike_slot[slot]) s_spike_accel_count--;
    s_accel_spike_slot[slot] = (fabsf(accel_mag_g - 1.0f) > g_safety_thresholds.spike_accel_g) ? 1U : 0U;
    s_spike_accel_count += s_accel_spike_slot[slot];

    uint32_t ws = g_window_samples();
    s_buf_head = (slot + 1U) % ws;
    if (s_buf_count < ws) s_buf_count++;

    refresh_safety_crc();
}

bool evaluate_stability(float tilt_spread)
{
    if (!fault_shadow_ok())                                               { fault_set(FAULT_INTEGRITY); return false; }
    if (!safety_crc_ok())                                                 { fault_set(FAULT_INTEGRITY); return false; }
    if (effective_faults() != 0U)                                         return false;
    // MA-2: signed subtraction wraps correctly on 2s-complement; avoids 49.7-day rollover bug.
    if ((int32_t)(millis() - s_instant_hold_until) < 0)                  return false;
    if (s_buf_count < g_window_samples())                                 return false;
    if (s_spike_gyro_count  > g_safety_thresholds.motion_samples_max)    return false;
    if (s_spike_accel_count > g_safety_thresholds.shock_samples_max)     return false;
    if (tilt_spread         > g_safety_thresholds.spread_max_deg)        return false;
    return true;
}

void reset_stability_state()
{
    s_buf_head          = 0U;
    s_buf_count         = 0U;
    s_spike_gyro_count  = 0U;
    s_spike_accel_count = 0U;
    s_frozen_count      = 0U;
    s_gyro_clean        = 0U;
    s_accel_clean       = 0U;
    s_diverse_clean     = 0U;
    s_timing_clean      = 0U;
    s_isr_cyc_valid     = false;
    s_comm_stale_a      = 0U;
    s_comm_stale_b      = 0U;
    memset(s_gyro_spike_slot,  0, sizeof(s_gyro_spike_slot));
    memset(s_accel_spike_slot, 0, sizeof(s_accel_spike_slot));
    memset(s_accel_mag_buf,    0, sizeof(s_accel_mag_buf));
    s_stab_fsm       = SFST_INIT;
    s_fsm_stable_cnt = 0U;
    strncpy(s_fsm_state_str, "INIT", sizeof(s_fsm_state_str));
    refresh_safety_crc();
}
