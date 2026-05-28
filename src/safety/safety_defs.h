#pragma once
// safety_defs.h -- all compile-time constants for the safety layer.
// NO includes from app/ permitted here.
#include <Arduino.h>
#include <stdint.h>

// --- Pin definitions ---------------------------------------------------------
#define LED_PIN         13
#define SAFE_OUT_PIN     2
#define SAFE_MON_PIN_HW  3     // independent INPUT_PULLDOWN; wired to SAFE_OUT_PIN net via 100 ohm (CR-4)
#define IMU_A_CS_PIN    10
#define IMU_B_CS_PIN     9
#define IMU_A_INT_PIN    8
#define IMU_B_INT_PIN    7

// --- ASM330LHB register map --------------------------------------------------
#define ASM_REG_WHO_AM_I        0x0FU
#define ASM_REG_INT1_CTRL       0x0DU
#define ASM_REG_CTRL1_XL        0x10U
#define ASM_REG_CTRL2_G         0x11U
#define ASM_REG_CTRL3_C         0x12U
#define ASM_REG_CTRL4_C         0x13U
#define ASM_REG_CTRL5_C         0x14U
#define ASM_REG_FUNC_CFG_ACCESS 0x01U
#define ASM_REG_PAGE_SEL        0x02U
#define ASM_REG_PAGE_ADDRESS    0x08U
#define ASM_REG_PAGE_VALUE      0x09U
#define ASM_REG_PAGE_RW         0x17U
#define ASM_EMB_FUNC_EN_B       0x05U
#define ASM_EMB_FSM_INT1_A      0x0BU
#define ASM_EMB_FSM_INT1_B      0x0CU
#define ASM_EMB_FSM_INT2_A      0x0FU
#define ASM_EMB_FSM_INT2_B      0x10U
#define ASM_EMB_FSM_ENABLE_A    0x46U
#define ASM_EMB_FSM_ENABLE_B    0x47U
#define ASM_EMB_ODR_CFG_B       0x5FU
#define ASM_REG_FSM_STATUS_A    0x36U
#define ASM_REG_STATUS_REG      0x1EU
#define ASM_REG_OUTX_L_G        0x22U
#define ASM_REG_OUTX_L_A        0x28U
#define ASM_WHOAMI_VAL          0x6BU

// --- Control register values -------------------------------------------------
#define ASM_CTRL1_XL_VAL  0x70U   // 833 Hz, ±2g
#define ASM_CTRL2_G_VAL   0x74U   // 833 Hz, ±500dps
#define ASM_CTRL3_C_VAL   0x44U   // BDU + IF_INC
#define ASM_CTRL4_C_VAL   0x08U   // DRDY_MASK

#define ASM_ACCEL_SENS_G   0.000061f
#define ASM_GYRO_SENS_DPS  0.01750f

#ifndef DEG_TO_RAD
#define DEG_TO_RAD  0.017453293f
#endif
#ifndef RAD_TO_DEG
#define RAD_TO_DEG  57.29577951f
#endif

#define ASM_SPI_CLOCK    4000000U
#define ASM_SPI_READ_BIT 0x80U

// --- Fault codes -------------------------------------------------------------
static constexpr uint16_t FAULT_IMU_A_COMM   = 0x0001U;
static constexpr uint16_t FAULT_IMU_B_COMM   = 0x0002U;
static constexpr uint16_t FAULT_GYRO_RANGE   = 0x0004U;
static constexpr uint16_t FAULT_ACCEL_RANGE  = 0x0008U;
static constexpr uint16_t FAULT_OUTPUT       = 0x0010U;
static constexpr uint16_t FAULT_FROZEN       = 0x0020U;
static constexpr uint16_t FAULT_TIMING       = 0x0040U;
static constexpr uint16_t FAULT_DIVERSE      = 0x0080U;
static constexpr uint16_t FAULT_DUAL_DIVERGE = 0x0100U;
static constexpr uint16_t FAULT_SELFTEST     = 0x0200U;
static constexpr uint16_t FAULT_INTEGRITY    = 0x0400U;

// --- Sampling & stability constants ------------------------------------------
static constexpr uint32_t AHRS_HZ            = 833U;
static constexpr uint32_t STABILITY_DIV      = 8U;
static constexpr uint32_t STABILITY_HZ       = AHRS_HZ / STABILITY_DIV;
static constexpr uint32_t WINDOW_SAMPLES_MAX = 5U * STABILITY_HZ;

static constexpr float    OMEGA_STABLE_DPS   =  20.0f;
static constexpr float    OMEGA_INSTANT_DPS  =  45.0f;
static constexpr uint32_t INSTANT_HOLD_MS    = 250U;
static constexpr float    SPIKE_ACCEL_G      =   0.10f;
static constexpr uint32_t EVAL_PERIOD_MS     = 200U;
static constexpr uint32_t WINDOW_S           =   1U;
static constexpr uint32_t MOTION_SAMPLES_MAX =  56U;
static constexpr uint32_t SHOCK_SAMPLES_MAX  =   3U;
static constexpr uint32_t CLEAN_STREAK_NEEDED=   5U;
static constexpr float    DIVERSE_MEAN_TOL_G =   0.1f;
static constexpr float    SPREAD_MAX_DEG     =   7.5f;
static constexpr float    ANCHOR_MAX_DEG     =   8.0f;
static constexpr float    GYRO_RANGE_FAULT_DPS= 248.0f;
static constexpr uint32_t STALE_FAULT_TICKS  =  15U;
static constexpr float    DUAL_DIVERGE_DEG   =   5.0f;

// --- Safety monitoring constants ---------------------------------------------
static constexpr uint16_t MARCH_BUF_BYTES     = 128U;
static constexpr uint32_t TIMER_TOLERANCE_PCT =   5U;
static constexpr uint8_t  FROZEN_TICKS_MAX    =  26U;
static constexpr uint8_t  FROZEN_BURST_TICKS  =   5U;
static constexpr float    MAHONY_KP           =   2.0f;
static constexpr float    MAHONY_KI           =   0.005f;
static constexpr uint32_t STACK_WARN_BYTES    =  6144U;
static constexpr uint8_t  FSM_STABLE_NEED     =   3U;
static constexpr uint8_t  IMU_B_DEGRADE_AFTER =   4U;

// --- EEPROM parameters -------------------------------------------------------
static constexpr uint32_t PARAMS_MAGIC       = 0xBE4F0002UL;
static constexpr uint8_t  PARAMS_VERSION     = 3U;
static constexpr int      PARAMS_EEPROM_ADDR = 0;

// --- Shared structs (app writes → safety reads) ------------------------------
// SafetyThresholds: populated by app layer after params_valid(); read by safety layer.
struct SafetyThresholds {
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

// g_safety_thresholds: defined in app_params.cpp, declared here so safety code
// can read thresholds without including any app/ header.
extern volatile SafetyThresholds g_safety_thresholds;

// SafetyStatus: written by safety layer; read (const volatile) by app layer.
struct SafetyStatus {
    bool     platform_stable;
    bool     output_inhibit;
    uint16_t fault_mask;
    char     why_str[12];
    char     fsm_state_str[5];
    float    last_omega_dps;
    float    last_roll_deg;
    float    last_pitch_deg;
    float    last_roll_b_deg;
    float    last_pitch_b_deg;
    float    last_spread_deg;
    uint32_t spike_gyro_count;
    uint32_t spike_accel_count;
    uint32_t comm_stale_a;
    uint32_t comm_stale_b;
    uint8_t  fsm_stable_cnt;
    bool     imu_b_degraded;
    uint32_t buf_count;
    uint32_t window_samples;
};
