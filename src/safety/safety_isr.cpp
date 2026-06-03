// safety_isr.cpp -- 833 Hz timer ISR, imu_recover, safety_tick, safety_init (ASIL D/SIL 3).
// Independence rule: no app/ includes.

#include "safety_isr.h"
#include "safety_monitor.h"
#include "safety_imu.h"
#include "safety_ahrs.h"
#include "safety_stability.h"
#include <math.h>
#include <string.h>
#include <stdio.h>
#include <SPI.h>

// --- ISR volatile state (extern in safety_isr.h) -----------------------------
volatile uint8_t s_sample_due       = 0U;
volatile uint8_t s_stability_due    = 0U;
volatile bool    s_forced_unsafe_isr= false;
volatile uint8_t s_hang_cp          = 0U;
uint32_t         s_missed_samples   = 0U;
uint32_t         s_isr_period_cyc   = 0U;
uint32_t         s_isr_tol_cyc      = 0U;
bool             s_isr_cyc_valid    = false;

// --- ISR private state -------------------------------------------------------
static volatile uint8_t  s_stability_div_cnt = 0U;
static          uint32_t s_last_isr_cyc      = 0U;
static volatile bool     s_timing_fault_isr  = false;
static volatile uint8_t  s_loop_alive_token  = 0U;
static volatile uint8_t  s_isr_token_last    = 0U;
static volatile uint8_t  s_isr_frozen_cnt    = 0U;

// --- IntervalTimer instance --------------------------------------------------
IntervalTimer s_sample_timer;

// --- Timer ISR ---------------------------------------------------------------
// Fires at AHRS_HZ (833 Hz). Checks DWT timing drift, drives AHRS and stability
// decimation ticks, and implements frozen-loop watchdog (CR-5).
void timerISR()
{
    if (s_isr_period_cyc != 0U) {
        uint32_t now_cyc = ARM_DWT_CYCCNT;
        if (s_isr_cyc_valid) {
            uint32_t delta = now_cyc - s_last_isr_cyc;
            uint32_t diff  = (delta > s_isr_period_cyc)
                             ? (delta - s_isr_period_cyc)
                             : (s_isr_period_cyc - delta);
            if (diff > s_isr_tol_cyc) s_timing_fault_isr = true;
        }
        s_last_isr_cyc  = now_cyc;
        s_isr_cyc_valid = true;
    }

    if (s_sample_due < 255U) s_sample_due++;
    asm volatile("dsb" ::: "memory");

    if (++s_stability_div_cnt >= (uint8_t)STABILITY_DIV) {
        s_stability_div_cnt = 0U;
        if (s_stability_due < 255U) s_stability_due++;
    }

    // CR-5: frozen-loop watchdog — s_forced_unsafe_isr is sticky;
    // only imu_recover() clears it, preventing loop() from resuming STABLE
    // output after a transient freeze without a verified recovery sequence.
    {
        uint8_t tok = s_loop_alive_token;
        if (tok == s_isr_token_last) {
            if (s_isr_frozen_cnt < 255U) s_isr_frozen_cnt++;
            if (s_isr_frozen_cnt >= FROZEN_TICKS_MAX) {
                digitalWriteFast(LED_PIN,      LOW);
                digitalWriteFast(SAFE_OUT_PIN, LOW);
                s_forced_unsafe_isr = true;
            }
        } else {
            s_isr_token_last = tok;
            s_isr_frozen_cnt = 0U;
        }
    }
}

// --- IMU recovery ------------------------------------------------------------
// b_only=true: only IMU-B failed; skip disrupting working IMU-A.
// Returns true only when target IMU(s) pass init, self-test, and return data.
bool imu_recover(bool b_only)
{
    if (s_peak_omega_latch > g_safety_thresholds.omega_instant_dps) {
        s_peak_omega_latch *= 0.9f;
        return false;
    }

    s_sample_timer.end();

    noInterrupts();
    s_sample_due        = 0U;
    s_stability_due     = 0U;
    s_stability_div_cnt = 0U;
    interrupts();

    set_safe_output(false);

    if (!b_only) asm_frozen_burst_reset(IMU_A_CS_PIN);
    asm_frozen_burst_reset(IMU_B_CS_PIN);

    // asm_init() calls asm_sw_reset() internally (SPI flush + SW_RST + settle).
    // Do NOT call asm_sw_reset() here first -- the double-reset causes IMU to
    // fail WHO_AM_I when running at 833 Hz (second reset interrupts recovery
    // from first). safety_init() calls asm_init() directly with no pre-reset.
    wdt_feed();
    bool ok_a = b_only ? true : asm_init(IMU_A_CS_PIN);
    bool ok_b = asm_init(IMU_B_CS_PIN);
    wdt_feed();

    if (!b_only && !ok_a) fault_set(FAULT_IMU_A_COMM);
    if (!ok_b)            fault_set(FAULT_IMU_B_COMM);

    if (!ok_a || !ok_b) {
        if (Serial.availableForWrite() > 0) {
            char ibuf[56];
            snprintf(ibuf, sizeof(ibuf), "[RECOVER] asm_init failed  A=%s  B=%s",
                     ok_a ? "OK" : "SKIP", ok_b ? "OK" : "NO-RESP");
            Serial.println(ibuf);
        }
        if (!b_only && ok_a) asm_fsm_configure(IMU_A_CS_PIN);
        if (ok_b)            asm_fsm_configure(IMU_B_CS_PIN);
        s_sample_timer.begin(timerISR, 1000000U / AHRS_HZ);
        return false;
    }

    bool st_a = b_only ? true : asm_self_test(IMU_A_CS_PIN);
    bool st_b = asm_self_test(IMU_B_CS_PIN);
    if (!st_a || !st_b) {
        fault_set(FAULT_SELFTEST);
        if (Serial.availableForWrite() > 0) {
            char stbuf[56];
            snprintf(stbuf, sizeof(stbuf), "[RECOVER] self-test failed  A=%s  B=%s",
                     st_a ? "PASS" : "FAIL", st_b ? "PASS" : "FAIL");
            Serial.println(stbuf);
        }
        if (!b_only && st_a) asm_fsm_configure(IMU_A_CS_PIN);
        if (st_b)            asm_fsm_configure(IMU_B_CS_PIN);
        s_sample_timer.begin(timerISR, 1000000U / AHRS_HZ);
        return false;
    }
    wdt_feed();

    {
        float ax, ay, az, gx, gy, gz;
        bool got_a = b_only;
        bool got_b = false;
        uint32_t t_wait = millis();
        while (!got_a || !got_b) {
            wdt_feed();
            if (!got_a) got_a = asm_read_sensors(IMU_A_CS_PIN, ax, ay, az, gx, gy, gz);
            if (!got_b) got_b = asm_read_sensors(IMU_B_CS_PIN, ax, ay, az, gx, gy, gz);
            if (millis() - t_wait > 3000U) {
                if (Serial.availableForWrite() > 0)
                    Serial.println("[RECOVER] timeout waiting for IMU data");
                s_sample_timer.begin(timerISR, 1000000U / AHRS_HZ);
                return false;
            }
            delay(2);
        }
    }

    if (!b_only) {
        mahony_init(s_imu_a);
        mahony_init(s_imu_b);
    } else {
        // Seed B from A's quaternion so Mahony-B starts at correct orientation;
        // prevents FAULT_DUAL_DIVERGE transient from identity-seeded convergence.
        s_imu_b.qw = s_imu_a.qw; s_imu_b.qx = s_imu_a.qx;
        s_imu_b.qy = s_imu_a.qy; s_imu_b.qz = s_imu_a.qz;
        s_imu_b.exi = 0.0f; s_imu_b.eyi = 0.0f; s_imu_b.ezi = 0.0f;
    }

    if (!b_only) asm_fsm_configure(IMU_A_CS_PIN);
    asm_fsm_configure(IMU_B_CS_PIN);

    fault_reset_all();
    reset_stability_state();
    s_peak_omega_latch  = 0.0f;
    s_forced_unsafe_isr = false;

    s_sample_timer.begin(timerISR, 1000000U / AHRS_HZ);
    return true;
}

// --- safety_tick() -----------------------------------------------------------
// Called from loop() on every iteration. Contains all safety processing:
// AHRS integration, stability evaluation, fault management, output control.
void safety_tick()
{
    static float    last_omega        = 0.0f, last_amag    = 1.0f;
    static float    last_roll         = 0.0f, last_pitch   = 0.0f;
    static float    last_roll_b       = 0.0f, last_pitch_b = 0.0f;
    static float    max_omega_period  = 0.0f;
    static uint32_t s_spi_ms_max      = 0U;
    static uint32_t s_loop_count      = 0U;
    static uint32_t s_hbeat_ms        = 0U;
    static uint32_t s_status_ms       = 0U;

    wdt_feed();
    s_loop_alive_token++;
    ++s_loop_count;

    // CR-5: ISR forced-unsafe flag -- sticky, cleared only by imu_recover().
    if (s_forced_unsafe_isr) {
        g_platform_stable = false;
        fault_set(FAULT_FROZEN);
        set_safe_output(false);
    }

    // ── Heartbeat (1 Hz) ─────────────────────────────────────────────────────
    {
        uint32_t now_hb = millis();
        if (now_hb - s_hbeat_ms >= 1000U) {
            s_hbeat_ms = now_hb;
            char hb[128];
            snprintf(hb, sizeof(hb),
                "[HBEAT] loops=%lu  fault=0x%04X  staleA=%lu  staleB=%lu  spi=%lums  ms=%lu",
                (unsigned long)s_loop_count, (unsigned)fault_mask_get(),
                (unsigned long)s_comm_stale_a, (unsigned long)s_comm_stale_b,
                (unsigned long)s_spi_ms_max,   (unsigned long)now_hb);
            if (Serial.availableForWrite() >= 128) Serial.println(hb);
            s_loop_count = 0U;
        }
    }

    // ── Timing fault (CR-1: atomic read-clear) ────────────────────────────────
    noInterrupts();
    bool timing_fault   = s_timing_fault_isr;
    s_timing_fault_isr  = false;
    interrupts();
    if (timing_fault) {
        fault_set(FAULT_TIMING);
        s_timing_clean = 0U;
    } else if (fault_mask_get() & FAULT_TIMING) {
        if (++s_timing_clean >= g_safety_thresholds.clean_streak_needed)
            fault_clr(FAULT_TIMING);
    }

    // ── AHRS sample tick (833 Hz) ─────────────────────────────────────────────
    // CR-1: atomic read-clear prevents the ISR increment from being lost.
    noInterrupts();
    uint8_t s_due  = s_sample_due;
    s_sample_due   = 0U;
    interrupts();

    if (s_due > 0U) {
        uint8_t replay = s_due - 1U;
        if (replay > 0U) s_missed_samples += (uint32_t)replay;

        float ax_a, ay_a, az_a, gx_a, gy_a, gz_a;
        float ax_b, ay_b, az_b, gx_b, gy_b, gz_b;

        s_hang_cp = 1U;
        uint32_t t_spi = millis();
        bool fresh_a = asm_read_sensors(IMU_A_CS_PIN, ax_a, ay_a, az_a, gx_a, gy_a, gz_a);
        bool fresh_b = asm_read_sensors(IMU_B_CS_PIN, ax_b, ay_b, az_b, gx_b, gy_b, gz_b);
        { uint32_t dt = millis() - t_spi; if (dt > s_spi_ms_max) s_spi_ms_max = dt; }
        s_hang_cp = 0U;

        // IMU-B axis remap: board reverse side + 90-degree rotation relative to IMU-A.
        // Derived from live gravity vectors: B.x=A.y, B.y=A.x, B.z=-A.z.
        if (fresh_b) {
            float tmp;
            tmp = ax_b; ax_b = ay_b; ay_b = tmp; az_b = -az_b;
            tmp = gx_b; gx_b = gy_b; gy_b = tmp; gz_b = -gz_b;
        }

        if (fresh_a) {
            s_comm_stale_a = 0U;
        } else {
            if (s_comm_stale_a < g_safety_thresholds.stale_fault_ticks) s_comm_stale_a++;
            if (s_comm_stale_a >= g_safety_thresholds.stale_fault_ticks) fault_set(FAULT_IMU_A_COMM);
        }
        if (fresh_b) {
            s_comm_stale_b = 0U;
        } else {
            if (s_comm_stale_b < g_safety_thresholds.stale_fault_ticks) s_comm_stale_b++;
            if (s_comm_stale_b >= g_safety_thresholds.stale_fault_ticks) fault_set(FAULT_IMU_B_COMM);
        }

        if (fresh_a) { for (uint8_t r = 0U; r <= replay; ++r) mahony_update(s_imu_a, gx_a, gy_a, gz_a, ax_a, ay_a, az_a); }
        if (fresh_b) { for (uint8_t r = 0U; r <= replay; ++r) mahony_update(s_imu_b, gx_b, gy_b, gz_b, ax_b, ay_b, az_b); }

        float roll_a, pitch_a, roll_b, pitch_b;
        mahony_to_tilt(s_imu_a, roll_a, pitch_a);
        mahony_to_tilt(s_imu_b, roll_b, pitch_b);

        float omega_a = 0.0f, omega_b = 0.0f;
        float amag_a  = 1.0f, amag_b  = 1.0f;
        if (fresh_a) {
            omega_a = sqrtf(gx_a*gx_a + gy_a*gy_a + gz_a*gz_a) * RAD_TO_DEG;
            amag_a  = sqrtf(ax_a*ax_a  + ay_a*ay_a  + az_a*az_a);
        }
        if (fresh_b) {
            omega_b = sqrtf(gx_b*gx_b + gy_b*gy_b + gz_b*gz_b) * RAD_TO_DEG;
            amag_b  = sqrtf(ax_b*ax_b  + ay_b*ay_b  + az_b*az_b);
        }

        float omega_dps = (omega_a > omega_b) ? omega_a : omega_b;
        float amag = (fresh_a && fresh_b) ? (amag_a + amag_b) * 0.5f
                   : (fresh_a ? amag_a : amag_b);

        if (isnan(omega_dps) || omega_dps > g_safety_thresholds.gyro_range_fault_dps) {
            fault_set(FAULT_GYRO_RANGE); s_gyro_clean = 0U;
        } else if (fault_mask_get() & FAULT_GYRO_RANGE) {
            if (++s_gyro_clean >= g_safety_thresholds.clean_streak_needed)
                fault_clr(FAULT_GYRO_RANGE);
        }

        if (isnan(amag) || amag < 0.05f || amag > 4.4f) {
            fault_set(FAULT_ACCEL_RANGE); s_accel_clean = 0U;
        } else if (fault_mask_get() & FAULT_ACCEL_RANGE) {
            if (++s_accel_clean >= g_safety_thresholds.clean_streak_needed)
                fault_clr(FAULT_ACCEL_RANGE);
        }

        // MA-2: overflow-safe instant hold arm
        if (omega_dps > g_safety_thresholds.omega_instant_dps)
            s_instant_hold_until = millis() + g_safety_thresholds.instant_hold_ms;

        if (fresh_a && fresh_b) {
            float dr = fabsf(roll_a  - roll_b);
            float dp = fabsf(pitch_a - pitch_b);
            if (dr > g_safety_thresholds.dual_diverge_deg || dp > g_safety_thresholds.dual_diverge_deg)
                fault_set(FAULT_DUAL_DIVERGE);
        }

        last_omega        = omega_dps;
        last_amag         = amag;
        last_roll         = roll_a;
        last_pitch        = pitch_a;
        s_last_omega_dps  = omega_dps;
        s_last_roll_deg   = roll_a;
        s_last_pitch_deg  = pitch_a;
        if (fresh_b) {
            last_roll_b        = roll_b;
            last_pitch_b       = pitch_b;
            s_last_roll_b_deg  = roll_b;
            s_last_pitch_b_deg = pitch_b;
        }
        if (omega_dps > max_omega_period) max_omega_period = omega_dps;
        if ((fresh_a || fresh_b) && omega_dps > s_peak_omega_latch)
            s_peak_omega_latch = omega_dps;

        if (fresh_a && fresh_b) s_frozen_count = 0U;
    }

    // ── Stability sample tick (~104 Hz) ───────────────────────────────────────
    // CR-1: atomic read-clear.
    noInterrupts();
    uint8_t stab_due = s_stability_due;
    s_stability_due  = 0U;
    interrupts();

    if (stab_due > 0U) {
        uint16_t ef = effective_faults();
        bool both_ok = !(ef & (FAULT_IMU_A_COMM | FAULT_IMU_B_COMM | FAULT_DUAL_DIVERGE));
        if (both_ok) {
            Tilt t;
            if (s_imu_b_degraded) {
                t.roll_deg  = s_last_roll_deg;
                t.pitch_deg = s_last_pitch_deg;
            } else {
                t.roll_deg  = (s_last_roll_deg  + s_last_roll_b_deg)  * 0.5f;
                t.pitch_deg = (s_last_pitch_deg + s_last_pitch_b_deg) * 0.5f;
            }
            t.yaw_deg = 0.0f;   // MA-3: yaw excluded from safety calculations
            push_sample(t, s_last_omega_dps, last_amag);
        } else {
            refresh_safety_crc();
            g_platform_stable = false;
            set_safe_output(false);
        }
    }

    // ── Periodic evaluation (200 ms) ─────────────────────────────────────────
    static uint32_t last_eval_ms = 0U;
    uint32_t now = millis();
    if (now - last_eval_ms >= g_safety_thresholds.eval_period_ms) {
        last_eval_ms = now;

        static uint8_t  s_whoami_fail_a  = 0U;
        static uint8_t  s_whoami_fail_b  = 0U;
        if (asm_reg_read(IMU_A_CS_PIN, ASM_REG_WHO_AM_I) != ASM_WHOAMI_VAL) {
            if (++s_whoami_fail_a >= 3U) fault_set(FAULT_IMU_A_COMM);
        } else {
            s_whoami_fail_a = 0U;
        }
        if (asm_reg_read(IMU_B_CS_PIN, ASM_REG_WHO_AM_I) != ASM_WHOAMI_VAL) {
            if (++s_whoami_fail_b >= 3U) fault_set(FAULT_IMU_B_COMM);
        } else {
            s_whoami_fail_b = 0U;
        }

        // Config register CRC-8 shadow check -- detects SPI corruption of ODR/range settings.
        // Shadow set by asm_init(); refreshed automatically on imu_recover() → asm_init().
        static uint8_t s_cfgcrc_fail_a = 0U;
        static uint8_t s_cfgcrc_fail_b = 0U;
        if (!asm_verify_config_crc(IMU_A_CS_PIN)) {
            if (++s_cfgcrc_fail_a >= 3U) fault_set(FAULT_IMU_A_COMM);
        } else {
            s_cfgcrc_fail_a = 0U;
        }
        if (!asm_verify_config_crc(IMU_B_CS_PIN)) {
            if (++s_cfgcrc_fail_b >= 3U) fault_set(FAULT_IMU_B_COMM);
        } else {
            s_cfgcrc_fail_b = 0U;
        }

        // IMU recovery with exponential backoff
        static uint32_t last_recover_ms  = 0U;
        static uint32_t recover_attempts = 0U;
        const uint16_t  recov_mask = FAULT_IMU_A_COMM | FAULT_IMU_B_COMM | FAULT_FROZEN | FAULT_SELFTEST;
        const uint16_t  hw_faults  = FAULT_IMU_A_COMM | FAULT_IMU_B_COMM | FAULT_SELFTEST;
        uint16_t fm = fault_mask_get();
        if (fm & recov_mask) {
            // Fast path: FAULT_FROZEN with no IMU hw fault.
            // Execution here proves loop() is alive -- clear flag directly, skip imu_recover().
            if (s_forced_unsafe_isr && !(fm & hw_faults)) {
                noInterrupts();
                s_forced_unsafe_isr = false;
                s_isr_frozen_cnt    = 0U;
                interrupts();
                fault_clr(FAULT_FROZEN);
                recover_attempts = 0U;
                if (Serial.availableForWrite() > 0)
                    Serial.println("[RECOVER] FAULT_FROZEN cleared -- loop alive, IMUs OK.");
            } else {
                uint32_t backoff = safe_min_u32(
                    3000U * (1U << safe_min_u32(recover_attempts, 4U)), 30000U);
                if (now - last_recover_ms >= backoff) {
                    last_recover_ms = now;
                    ++recover_attempts;
                    char rbuf[72];
                    snprintf(rbuf, sizeof(rbuf),
                        "[RECOVER] Attempt #%lu  fault=0x%04X  backoff=%lus",
                        (unsigned long)recover_attempts, (unsigned)fm,
                        (unsigned long)(backoff / 1000U));
                    if (Serial.availableForWrite() > 0) Serial.println(rbuf);
                    bool b_only = (fm & FAULT_IMU_B_COMM) &&
                                  !(fm & (FAULT_IMU_A_COMM | FAULT_FROZEN));
                    if (imu_recover(b_only)) {
                        if (Serial.availableForWrite() > 0)
                            Serial.println("[RECOVER] IMUs restored.");
                        recover_attempts = 0U;
                        s_b_fail_count   = 0U;
                        s_imu_b_degraded = false;
                    } else {
                        if (Serial.availableForWrite() > 0)
                            Serial.println("[RECOVER] Not responding, will retry.");
                        if (b_only) {
                            if (s_b_fail_count < 255U) s_b_fail_count++;
                            if (!s_imu_b_degraded && s_b_fail_count >= IMU_B_DEGRADE_AFTER) {
                                s_imu_b_degraded = true;
                                if (Serial.availableForWrite() > 0)
                                    Serial.println("[DEGRADE] IMU-B absent -- single-channel mode (IMU-A only)");
                            }
                        }
                    }
                }
            }
        } else {
            recover_attempts = 0U;
        }

        // Stack depth watermark (MA-9)
        stack_watermark_update();
        if (stack_used_bytes() > STACK_WARN_BYTES) {
            fault_set(FAULT_INTEGRITY);
            if (Serial.availableForWrite() > 0) {
                char sb[56];
                snprintf(sb, sizeof(sb), "[STACK] deep=%lu bytes -- FAULT_INTEGRITY",
                         (unsigned long)stack_used_bytes());
                Serial.println(sb);
            }
        }

        float spread = compute_window_tilt_spread();
        s_last_spread_deg = spread;

        // MI-4: diverse_stable() MUST precede evaluate_stability()
        if (s_buf_count >= g_window_samples()) {
            if (diverse_stable()) {
                if (fault_mask_get() & FAULT_DIVERSE) {
                    if (++s_diverse_clean >= g_safety_thresholds.clean_streak_needed)
                        fault_clr(FAULT_DIVERSE);
                } else {
                    s_diverse_clean = 0U;
                }
            } else {
                fault_set(FAULT_DIVERSE);
                s_diverse_clean = 0U;
            }
        }

        static uint8_t s_dual_diverge_clean = 0U;
        if (fault_mask_get() & FAULT_DUAL_DIVERGE) {
            uint16_t ef_div = effective_faults();
            bool can_check = !(ef_div & (FAULT_IMU_A_COMM | FAULT_IMU_B_COMM));
            if (can_check) {
                float dr = fabsf(s_last_roll_deg  - s_last_roll_b_deg);
                float dp = fabsf(s_last_pitch_deg - s_last_pitch_b_deg);
                if (dr <= g_safety_thresholds.dual_diverge_deg && dp <= g_safety_thresholds.dual_diverge_deg) {
                    if (++s_dual_diverge_clean >= g_safety_thresholds.clean_streak_needed)
                        fault_clr(FAULT_DUAL_DIVERGE);
                } else {
                    s_dual_diverge_clean = 0U;
                }
            } else {
                s_dual_diverge_clean = 0U;
            }
        } else {
            s_dual_diverge_clean = 0U;
        }

        // HW FSM gate -- skip read on COMM fault (MISO floats HIGH → false stable)
        bool comm_a_ok = !(fault_mask_get() & FAULT_IMU_A_COMM);
        bool comm_b_ok = !(fault_mask_get() & FAULT_IMU_B_COMM);
        bool hw_a = comm_a_ok && asm_fsm_read_stable(IMU_A_CS_PIN);
        bool hw_b = s_imu_b_degraded ? true : (comm_b_ok && asm_fsm_read_stable(IMU_B_CS_PIN));
        if (hw_a && hw_b) {
            if (s_fsm_stable_cnt < 255U) s_fsm_stable_cnt++;
        } else {
            s_fsm_stable_cnt = 0U;
        }
        bool hw_gate = (s_fsm_stable_cnt >= FSM_STABLE_NEED);

        bool sw_ok = evaluate_stability(spread);

        {
            uint16_t ef = effective_faults();
            switch (s_stab_fsm) {
                case SFST_INIT:
                    s_stab_fsm = SFST_FILLING;
                    break;
                case SFST_FILLING:
                    if (ef != 0U || s_forced_unsafe_isr)
                        { s_stab_fsm = SFST_FAULT; break; }
                    if (s_buf_count >= g_window_samples())
                        s_stab_fsm = (sw_ok && hw_gate) ? SFST_STABLE : SFST_UNSTABLE;
                    break;
                case SFST_STABLE:
                    if (ef != 0U || s_forced_unsafe_isr)
                        { s_stab_fsm = SFST_FAULT; break; }
                    if (!sw_ok || !hw_gate)
                        s_stab_fsm = SFST_UNSTABLE;
                    break;
                case SFST_UNSTABLE:
                    if (ef != 0U || s_forced_unsafe_isr)
                        { s_stab_fsm = SFST_FAULT; break; }
                    if (sw_ok && hw_gate)
                        s_stab_fsm = SFST_STABLE;
                    break;
                case SFST_FAULT:
                    if (ef == 0U && !s_forced_unsafe_isr)
                        s_stab_fsm = SFST_INIT;
                    break;
            }
        }
        switch (s_stab_fsm) {
            case SFST_FILLING:  strncpy(s_fsm_state_str, "FILL", 5); break;
            case SFST_STABLE:   strncpy(s_fsm_state_str, "STBL", 5); break;
            case SFST_UNSTABLE: strncpy(s_fsm_state_str, "MOVE", 5); break;
            case SFST_FAULT:    strncpy(s_fsm_state_str, "FALT", 5); break;
            default:            strncpy(s_fsm_state_str, "INIT", 5); break;
        }
        g_platform_stable = !s_forced_unsafe_isr && (s_stab_fsm == SFST_STABLE);

        bool anchor_tripped = false;
        {
            Tilt cur = { last_roll, last_pitch, 0.0f };
            if (g_platform_stable) {
                if (!s_anchor_valid) {
                    s_stable_anchor = cur;
                    s_anchor_valid  = true;
                } else if (tilt_distance_deg(cur, s_stable_anchor) > g_safety_thresholds.anchor_max_deg) {
                    g_platform_stable = false;
                    s_anchor_valid    = false;
                    anchor_tripped    = true;
                }
            } else {
                s_anchor_valid = false;
            }
        }

        set_safe_output(g_platform_stable);

        // ── Status line (0.2 Hz) ──────────────────────────────────────────────
        uint32_t ws    = g_window_samples();
        uint32_t count = safe_min_u32(s_buf_count, ws);
        float fill     = 100.0f * (float)count / (float)ws;

        char fault_str[96] = "none";
        uint16_t cur_fm = fault_mask_get();
        if (cur_fm || s_imu_b_degraded) {
            int pos = 0; fault_str[0] = '\0';
            if (cur_fm & FAULT_IMU_A_COMM)  pos += snprintf(fault_str+pos, 96-pos, "COMM_A ");
            if (cur_fm & FAULT_IMU_B_COMM)  pos += snprintf(fault_str+pos, 96-pos, "COMM_B ");
            if (s_imu_b_degraded)            pos += snprintf(fault_str+pos, 96-pos, "DGRADE_B ");
            if (cur_fm & FAULT_GYRO_RANGE)  pos += snprintf(fault_str+pos, 96-pos, "GYRO ");
            if (cur_fm & FAULT_ACCEL_RANGE) pos += snprintf(fault_str+pos, 96-pos, "ACCEL ");
            if (cur_fm & FAULT_OUTPUT)       pos += snprintf(fault_str+pos, 96-pos, "OUTPUT ");
            if (cur_fm & FAULT_FROZEN)       pos += snprintf(fault_str+pos, 96-pos, "FROZEN ");
            if (cur_fm & FAULT_TIMING)       pos += snprintf(fault_str+pos, 96-pos, "TIMING ");
            if (cur_fm & FAULT_DIVERSE)      pos += snprintf(fault_str+pos, 96-pos, "DIVERSE ");
            if (cur_fm & FAULT_DUAL_DIVERGE) pos += snprintf(fault_str+pos, 96-pos, "DIVERGE ");
            if (cur_fm & FAULT_SELFTEST)     pos += snprintf(fault_str+pos, 96-pos, "SELFTEST ");
            if (cur_fm & FAULT_INTEGRITY)    pos += snprintf(fault_str+pos, 96-pos, "INTEGRITY ");
            (void)pos;
        }

        bool instant_active = ((int32_t)(millis() - s_instant_hold_until) < 0);
        const char *why_str = "OK";
        if (!g_platform_stable) {
            if      (s_stab_fsm == SFST_FAULT)                                     why_str = "FAULT";
            else if (anchor_tripped)                                               why_str = "DRIFT";
            else if (!fault_shadow_ok())                                           why_str = "SHADOW";
            else if (!safety_crc_ok())                                             why_str = "CRC";
            else if (cur_fm != 0U)                                                 why_str = "FAULT";
            else if (instant_active)                                               why_str = "HOLD";
            else if (s_stab_fsm == SFST_FILLING)                                   why_str = "FILL";
            else if (s_buf_count < g_window_samples())                             why_str = "FILL";
            else if (s_spike_gyro_count  > g_safety_thresholds.motion_samples_max) why_str = "GYRO_SPK";
            else if (s_spike_accel_count > g_safety_thresholds.shock_samples_max)  why_str = "ACCEL_SPK";
            else if (spread > g_safety_thresholds.spread_max_deg)                  why_str = "SPREAD";
            else if (!hw_gate)                                                     why_str = "HW_FSM";
            else                                                                   why_str = "UNKNOWN";
        }
        strncpy(s_why_str, why_str, sizeof(s_why_str) - 1);
        s_why_str[sizeof(s_why_str) - 1] = '\0';

        // MI-7: capture hang checkpoint BEFORE overwriting with status-print marker
        uint8_t hang_cp_snap = s_hang_cp;
        char line[512];
        s_hang_cp = 3U;
        snprintf(line, sizeof(line),
            "[STATUS] stable=%s%s  why=%-9s  fsm=%s  hw=%u"
            "  fault=%s  inhib=%d  fill=%d%%"
            "  staleA=%lu  staleB=%lu"
            "  gyro_spk=%lu/%lu  accel_spk=%lu/%lu"
            "  omega=%.2fdps  pk=%.2fdps"
            "  RA=%.2f PA=%.2f  RB=%.2f PB=%.2f  dR=%.2f dP=%.2f"
            "  spread=%.2fdeg  miss=%lu  stk=%lu  cp=%u  spi=%lums",
            g_platform_stable ? "TRUE " : "FALSE",
            instant_active    ? "*"     : " ",
            why_str, s_fsm_state_str, (unsigned)s_fsm_stable_cnt,
            fault_str, (int)g_output_inhibit, (int)fill,
            (unsigned long)s_comm_stale_a,      (unsigned long)s_comm_stale_b,
            (unsigned long)s_spike_gyro_count,  (unsigned long)g_safety_thresholds.motion_samples_max,
            (unsigned long)s_spike_accel_count, (unsigned long)g_safety_thresholds.shock_samples_max,
            (double)last_omega,     (double)max_omega_period,
            (double)last_roll,      (double)last_pitch,
            (double)last_roll_b,    (double)last_pitch_b,
            (double)fabsf(last_roll - last_roll_b),
            (double)fabsf(last_pitch - last_pitch_b),
            (double)spread,
            (unsigned long)s_missed_samples,
            (unsigned long)stack_used_bytes(),
            (unsigned)hang_cp_snap,
            (unsigned long)s_spi_ms_max);

        uint32_t now_st = millis();
        if (now_st - s_status_ms >= 5000U) {
            if (Serial.availableForWrite() >= 320) {
                s_status_ms = now_st;
                Serial.println(line);
            }
        }
        s_hang_cp        = 0U;
        s_spi_ms_max     = 0U;
        max_omega_period = 0.0f;

        // Update g_safety_status for app-layer HTTP reads
        g_safety_status.platform_stable  = g_platform_stable;
        g_safety_status.output_inhibit   = g_output_inhibit;
        g_safety_status.fault_mask       = cur_fm;
        memcpy((char*)g_safety_status.why_str,       s_why_str,       sizeof(g_safety_status.why_str) - 1);
        ((char*)g_safety_status.why_str)[sizeof(g_safety_status.why_str) - 1] = '\0';
        memcpy((char*)g_safety_status.fsm_state_str, s_fsm_state_str, sizeof(g_safety_status.fsm_state_str) - 1);
        ((char*)g_safety_status.fsm_state_str)[sizeof(g_safety_status.fsm_state_str) - 1] = '\0';
        g_safety_status.last_omega_dps   = s_last_omega_dps;
        g_safety_status.last_roll_deg    = s_last_roll_deg;
        g_safety_status.last_pitch_deg   = s_last_pitch_deg;
        g_safety_status.last_roll_b_deg  = s_last_roll_b_deg;
        g_safety_status.last_pitch_b_deg = s_last_pitch_b_deg;
        g_safety_status.last_spread_deg  = s_last_spread_deg;
        g_safety_status.spike_gyro_count = s_spike_gyro_count;
        g_safety_status.spike_accel_count= s_spike_accel_count;
        g_safety_status.comm_stale_a     = s_comm_stale_a;
        g_safety_status.comm_stale_b     = s_comm_stale_b;
        g_safety_status.fsm_stable_cnt   = s_fsm_stable_cnt;
        g_safety_status.imu_b_degraded   = s_imu_b_degraded;
        g_safety_status.buf_count        = s_buf_count;
        g_safety_status.window_samples   = ws;
    }
}

// --- safety_init() -----------------------------------------------------------
// Called from setup() after app_params_init() has loaded thresholds.
// Handles all safety-layer hardware and software initialization.
void safety_init()
{
    pinMode(LED_PIN,         OUTPUT); digitalWrite(LED_PIN,         LOW);
    pinMode(SAFE_OUT_PIN,    OUTPUT); digitalWrite(SAFE_OUT_PIN,    LOW);
    // INPUT_PULLDOWN: unwired SAFE_MON_PIN_HW reads LOW (not floating HIGH),
    // preventing a false FAULT_INTEGRITY at startup before the first sample.
    pinMode(SAFE_MON_PIN_HW, INPUT_PULLDOWN);

    pinMode(IMU_A_CS_PIN,   OUTPUT); digitalWrite(IMU_A_CS_PIN, HIGH);
    pinMode(IMU_B_CS_PIN,   OUTPUT); digitalWrite(IMU_B_CS_PIN, HIGH);
    pinMode(IMU_A_INT_PIN,  INPUT);
    pinMode(IMU_B_INT_PIN,  INPUT);

    if (!ram_march_test()) {
        for (;;) {
            digitalWrite(LED_PIN, HIGH); delay(100);
            digitalWrite(LED_PIN, LOW);  delay(100);
        }
    }

    ARM_DWT_CTRL |= ARM_DWT_CTRL_CYCCNTENA;
    ARM_DWT_CYCCNT = 0U;

    Serial.begin(115200);
    while (!Serial && millis() < 3000U) {}
    Serial.println("Platform Stability Monitor v2  Teensy4.1+ASM330LHBx2  ASIL-D");

    {
        uint32_t srsr = SRC_SRSR;
        SRC_SRSR = srsr;
        if      (srsr & (1u<<7)) Serial.println("[RESET_CAUSE] WDOG3 (app watchdog)");
        else if (srsr & (1u<<4)) Serial.println("[RESET_CAUSE] WDOG1 watchdog");
        else if (srsr & (1u<<3)) Serial.println("[RESET_CAUSE] IPP user reset");
        else if (srsr & (1u<<0)) Serial.println("[RESET_CAUSE] POR");
        else {
            char rc[48];
            snprintf(rc, sizeof(rc), "[RESET_CAUSE] other SRC_SRSR=0x%08lX", (unsigned long)srsr);
            Serial.println(rc);
        }
    }

    s_isr_period_cyc = (uint32_t)((uint64_t)F_CPU / AHRS_HZ);
    s_isr_tol_cyc    = s_isr_period_cyc * TIMER_TOLERANCE_PCT / 100U;

    SPI1.begin();

    Serial.println("[INIT] IMU-A...");
    if (!asm_init(IMU_A_CS_PIN))
        fatal("IMU-A WHO_AM_I mismatch -- check CS wiring and SPI mode");
    Serial.println("[CHECK] IMU-A WHO_AM_I OK");

    Serial.println("[INIT] IMU-B...");
    if (!asm_init(IMU_B_CS_PIN))
        fatal("IMU-B WHO_AM_I mismatch -- check CS wiring and SPI mode");
    Serial.println("[CHECK] IMU-B WHO_AM_I OK");

    Serial.println("[INIT] Self-test IMU-A (platform must be stationary)...");
    bool st_a = asm_self_test(IMU_A_CS_PIN);
    Serial.println("[INIT] Self-test IMU-B...");
    bool st_b = asm_self_test(IMU_B_CS_PIN);
    if (!st_a || !st_b) {
        fault_set(FAULT_SELFTEST);
        char stbuf[96];
        snprintf(stbuf, sizeof(stbuf),
            "[SELFTEST] FAILED  A=%s  B=%s  -- FAULT_SELFTEST set",
            st_a ? "PASS" : "FAIL", st_b ? "PASS" : "FAIL");
        Serial.println(stbuf);
    } else {
        Serial.println("[CHECK] Self-test PASS A+B");
    }

    Serial.println("[INIT] Configuring FSM A...");
    asm_fsm_configure(IMU_A_CS_PIN);
    Serial.println("[INIT] Configuring FSM B...");
    asm_fsm_configure(IMU_B_CS_PIN);
    Serial.println("[CHECK] FSM configured A+B");

    {
        float ax, ay, az, gx, gy, gz;
        bool got_a = false, got_b = false;
        uint32_t t0 = millis();
        Serial.println("[INIT] Waiting for first IMU data...");
        while (!got_a || !got_b) {
            if (!got_a) got_a = asm_read_sensors(IMU_A_CS_PIN, ax, ay, az, gx, gy, gz);
            if (!got_b) got_b = asm_read_sensors(IMU_B_CS_PIN, ax, ay, az, gx, gy, gz);
            if (millis() - t0 > 5000U)
                fatal("IMU data timeout during init -- check ODR config");
        }
    }

    mahony_init(s_imu_a);
    mahony_init(s_imu_b);
    refresh_safety_crc();

    wdt_begin(1500);
    stack_base_capture();
    Serial.println("[INIT] Running.");
    wdt_feed();
}

// Called from setup() after app_init() so the ISR only starts when loop() is ready.
// Prevents false FAULT_FROZEN detection during slow Ethernet init.
void safety_start_isr()
{
    s_sample_timer.begin(timerISR, 1000000U / AHRS_HZ);
}
