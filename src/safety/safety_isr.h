#pragma once
// safety_isr.h -- 833 Hz timer ISR, imu_recover, ISR volatile state (ASIL D/SIL 3).
// No app/ includes.
#include <Arduino.h>
#include "safety_defs.h"
#include "safety_types.h"

// ISR volatile state accessed by safety_tick() and loop()
extern volatile uint8_t s_sample_due;
extern volatile uint8_t s_stability_due;
extern volatile bool    s_forced_unsafe_isr;
extern volatile uint8_t s_hang_cp;
extern          uint32_t s_missed_samples;
extern          uint32_t s_isr_period_cyc;
extern          uint32_t s_isr_tol_cyc;
extern          bool     s_isr_cyc_valid;   // reset by reset_stability_state()

// IntervalTimer instance (started/stopped by imu_recover and setup)
extern IntervalTimer s_sample_timer;

// ISR (registered with IntervalTimer)
void timerISR();

// IMU recovery with exponential backoff. b_only=true: only IMU-B failed.
bool imu_recover(bool b_only = false);

// High-level safety tick (called from loop every iteration)
void safety_tick();

// High-level safety init (called from setup)
void safety_init();

// Starts the 833 Hz interval timer ISR. Called from setup() AFTER app_init()
// so the frozen-loop watchdog doesn't trigger during slow Ethernet PHY init.
void safety_start_isr();
