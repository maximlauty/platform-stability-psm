#pragma once
// safety_stability.h -- window buffer, stability algorithm, FSM (ASIL D/SIL 3).
// No app/ includes.
#include "safety_defs.h"
#include "safety_types.h"

// Read-only safety status written here, read by app layer
extern volatile SafetyStatus g_safety_status;

// Stability helpers
float    clamp_f       (float v, float lo, float hi);
uint32_t safe_min_u32  (uint32_t a, uint32_t b);
uint32_t g_window_samples();   // = g_safety_thresholds.window_s * STABILITY_HZ

// Fault mask accessor with degraded-mode B-channel masking
uint16_t effective_faults();

// Stability algorithm
bool    diverse_stable          ();
float   tilt_distance_deg       (const Tilt &a, const Tilt &b);
float   compute_window_tilt_spread();
void    push_sample             (const Tilt &t, float omega_dps, float accel_mag_g);
bool    evaluate_stability      (float tilt_spread);
void    reset_stability_state   ();

// Window buffer state (extern-linked to safety_monitor compute_safety_crc)
extern uint32_t s_buf_head;
extern uint32_t s_buf_count;
extern uint32_t s_spike_gyro_count;
extern uint32_t s_spike_accel_count;

// State accessible to ISR / recovery code
extern Tilt    s_stable_anchor;
extern bool    s_anchor_valid;
extern bool    s_imu_b_degraded;
extern uint8_t s_b_fail_count;
extern float   s_peak_omega_latch;
extern uint32_t s_instant_hold_until;
extern float   s_last_omega_dps;
extern float   s_last_roll_deg;
extern float   s_last_pitch_deg;
extern float   s_last_roll_b_deg;
extern float   s_last_pitch_b_deg;
extern float   s_last_spread_deg;
extern char    s_why_str[12];
extern char    s_fsm_state_str[5];
extern uint8_t s_fsm_stable_cnt;
extern StabFSM s_stab_fsm;
extern uint32_t s_comm_stale_a;
extern uint32_t s_comm_stale_b;
extern uint8_t  s_gyro_clean;
extern uint8_t  s_accel_clean;
extern uint8_t  s_diverse_clean;
extern uint8_t  s_timing_clean;
extern uint32_t s_frozen_count;
