// app_params.cpp -- EEPROM-backed Params, threshold publishing (QM layer).

#include <Arduino.h>
#include "app_params.h"
#include "../safety/safety_monitor.h"
#include <EEPROM.h>
#include <math.h>

// --- Default parameter values ------------------------------------------------
static const Params PARAMS_DEFAULT = {
    PARAMS_MAGIC, PARAMS_VERSION,
    WINDOW_S,           OMEGA_STABLE_DPS,  OMEGA_INSTANT_DPS,  INSTANT_HOLD_MS,
    SPIKE_ACCEL_G,      EVAL_PERIOD_MS,    MOTION_SAMPLES_MAX, SHOCK_SAMPLES_MAX,
    CLEAN_STREAK_NEEDED,DIVERSE_MEAN_TOL_G,SPREAD_MAX_DEG,
    ANCHOR_MAX_DEG,     GYRO_RANGE_FAULT_DPS, STALE_FAULT_TICKS,
    DUAL_DIVERGE_DEG
};

// --- Global definitions ------------------------------------------------------
Params              g_params           = PARAMS_DEFAULT;
volatile SafetyThresholds g_safety_thresholds = {
    WINDOW_S,           OMEGA_STABLE_DPS,  OMEGA_INSTANT_DPS,  INSTANT_HOLD_MS,
    SPIKE_ACCEL_G,      EVAL_PERIOD_MS,    MOTION_SAMPLES_MAX, SHOCK_SAMPLES_MAX,
    CLEAN_STREAK_NEEDED,DIVERSE_MEAN_TOL_G,SPREAD_MAX_DEG,
    ANCHOR_MAX_DEG,     GYRO_RANGE_FAULT_DPS, STALE_FAULT_TICKS,
    DUAL_DIVERGE_DEG
};

// --- Parameter validation (CR-6: isfinite guards on all float fields) --------
bool params_valid(const Params &p)
{
    if (p.magic   != PARAMS_MAGIC)   return false;
    if (p.version != PARAMS_VERSION) return false;
    if (p.window_s < 1U || p.window_s > 5U) return false;
    if (!isfinite(p.omega_stable_dps)    || p.omega_stable_dps    <   1.0f || p.omega_stable_dps    > 500.0f)  return false;
    if (!isfinite(p.omega_instant_dps)   || p.omega_instant_dps   <   1.0f || p.omega_instant_dps   > 500.0f)  return false;
    if (p.instant_hold_ms > 5000U)                                                                               return false;
    if (!isfinite(p.spike_accel_g)       || p.spike_accel_g       < 0.01f  || p.spike_accel_g       >   2.0f)  return false;
    if (p.eval_period_ms  < 50U          || p.eval_period_ms      > 2000U)                                      return false;
    if (p.motion_samples_max > 500U)                                                                             return false;
    if (p.shock_samples_max  > 100U)                                                                             return false;
    if (p.clean_streak_needed < 1U       || p.clean_streak_needed  >  50U)                                      return false;
    if (!isfinite(p.diverse_mean_tol_g)  || p.diverse_mean_tol_g  < 0.001f || p.diverse_mean_tol_g  >   1.0f)  return false;
    if (!isfinite(p.spread_max_deg)      || p.spread_max_deg       <  0.1f  || p.spread_max_deg      >  90.0f)  return false;
    if (!isfinite(p.anchor_max_deg)      || p.anchor_max_deg       <  0.1f  || p.anchor_max_deg      >  90.0f)  return false;
    if (!isfinite(p.gyro_range_fault_dps)|| p.gyro_range_fault_dps< 10.0f  || p.gyro_range_fault_dps>2000.0f)  return false;
    if (p.stale_fault_ticks < 1U         || p.stale_fault_ticks   > 500U)                                       return false;
    if (!isfinite(p.dual_diverge_deg)    || p.dual_diverge_deg     <  0.5f  || p.dual_diverge_deg    >  90.0f)  return false;
    return true;
}

bool params_load()
{
    Params p;
    EEPROM.get(PARAMS_EEPROM_ADDR, p);
    if (!params_valid(p)) return false;
    g_params = p;
    return true;
}

void params_save()
{
    g_params.magic   = PARAMS_MAGIC;
    g_params.version = PARAMS_VERSION;
    // MA-8: disable interrupts during flash write to prevent ISR preemption.
    noInterrupts();
    wdt_feed();
    EEPROM.put(PARAMS_EEPROM_ADDR, g_params);
    wdt_feed();
    interrupts();
    app_publish_thresholds();   // MA-1: refresh CRC with updated thresholds
}

// Copies validated g_params into g_safety_thresholds then refreshes the CRC.
// Called after every params_load() or params_save().
void app_publish_thresholds()
{
    noInterrupts();
    g_safety_thresholds.window_s            = g_params.window_s;
    g_safety_thresholds.omega_stable_dps    = g_params.omega_stable_dps;
    g_safety_thresholds.omega_instant_dps   = g_params.omega_instant_dps;
    g_safety_thresholds.instant_hold_ms     = g_params.instant_hold_ms;
    g_safety_thresholds.spike_accel_g       = g_params.spike_accel_g;
    g_safety_thresholds.eval_period_ms      = g_params.eval_period_ms;
    g_safety_thresholds.motion_samples_max  = g_params.motion_samples_max;
    g_safety_thresholds.shock_samples_max   = g_params.shock_samples_max;
    g_safety_thresholds.clean_streak_needed = g_params.clean_streak_needed;
    g_safety_thresholds.diverse_mean_tol_g  = g_params.diverse_mean_tol_g;
    g_safety_thresholds.spread_max_deg      = g_params.spread_max_deg;
    g_safety_thresholds.anchor_max_deg      = g_params.anchor_max_deg;
    g_safety_thresholds.gyro_range_fault_dps= g_params.gyro_range_fault_dps;
    g_safety_thresholds.stale_fault_ticks   = g_params.stale_fault_ticks;
    g_safety_thresholds.dual_diverge_deg    = g_params.dual_diverge_deg;
    interrupts();
    refresh_safety_crc();
}
