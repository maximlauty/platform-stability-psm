// safety_monitor.cpp -- ASIL D / SIL 3 safety monitor core.
// WDT, fault register (shadow pair), CRC-16, RAM march, stack watermark,
// safe output control, fatal handler.
// Independence rule: no app/ includes.

#include "safety_monitor.h"
#include <Arduino.h>
#include <string.h>

// --- Watchdog (iMXRT1062 RTWDOG3) -------------------------------------------
void wdt_begin(uint16_t timeout_ms)
{
    __disable_irq();
    WDOG3_CNT = 0xD928C520UL;
    WDOG3_CNT = 0xB480A602UL;
    __enable_irq();
    while (!(WDOG3_CS & WDOG_CS_ULK)) {}
    WDOG3_TOVAL = timeout_ms;
    WDOG3_CS = WDOG_CS_EN | WDOG_CS_CLK(1) | WDOG_CS_UPDATE | WDOG_CS_CMD32EN;
    while (!(WDOG3_CS & WDOG_CS_RCS)) {}
}

void wdt_feed()
{
    WDOG3_CNT = 0xB480A602UL;
}

// --- Fault register (shadow pair) -------------------------------------------
// Both registers are declared here (file-scope, not exported); all access goes
// through the public API functions in this file.
static uint16_t s_fault_mask     = 0U;
static uint16_t s_fault_mask_inv = 0xFFFFU;

// CRC is also declared here (compute_safety_crc reads s_fault_mask from this file)
static uint16_t s_safety_crc = 0U;

// Forward declaration: fault helpers call refresh_safety_crc
static void refresh_safety_crc_internal();

void fault_set(uint16_t bits)
{
    noInterrupts();
    s_fault_mask     |=  bits;
    s_fault_mask_inv &= (uint16_t)(~bits);
    interrupts();
    refresh_safety_crc_internal();
}

void fault_clr(uint16_t bits)
{
    noInterrupts();
    s_fault_mask     &= (uint16_t)(~bits);
    s_fault_mask_inv |=  bits;
    interrupts();
    refresh_safety_crc_internal();
}

void fault_reset_all()
{
    noInterrupts();
    s_fault_mask     = 0U;
    s_fault_mask_inv = 0xFFFFU;
    interrupts();
    refresh_safety_crc_internal();
}

bool fault_shadow_ok()
{
    return (uint16_t)(s_fault_mask ^ s_fault_mask_inv) == 0xFFFFU;
}

void fault_set_from_isr(uint16_t bits)
{
    s_fault_mask     |=  bits;
    s_fault_mask_inv &= (uint16_t)(~bits);
}

void fault_clr_from_isr(uint16_t bits)
{
    s_fault_mask     &= (uint16_t)(~bits);
    s_fault_mask_inv |=  bits;
}

uint16_t fault_mask_get()
{
    return s_fault_mask;
}

// --- CRC-16 (CCITT-FALSE, poly 0x1021, init 0xFFFF) -------------------------
uint16_t crc16_ccitt(uint16_t crc, uint8_t byte)
{
    crc ^= (uint16_t)byte << 8;
    for (uint8_t i = 0U; i < 8U; ++i)
        crc = (crc & 0x8000U) ? (uint16_t)((crc << 1U) ^ 0x1021U) : (uint16_t)(crc << 1U);
    return crc;
}

// External state read by compute_safety_crc (defined in safety_stability.cpp and safety_isr.cpp)
extern uint32_t s_spike_gyro_count;
extern uint32_t s_spike_accel_count;
extern uint32_t s_buf_count;
extern uint32_t s_buf_head;

uint16_t compute_safety_crc()
{
    uint16_t crc = 0xFFFFU;
    crc = crc16_ccitt(crc, (uint8_t)(s_spike_gyro_count));
    crc = crc16_ccitt(crc, (uint8_t)(s_spike_gyro_count  >>  8));
    crc = crc16_ccitt(crc, (uint8_t)(s_spike_gyro_count  >> 16));
    crc = crc16_ccitt(crc, (uint8_t)(s_spike_gyro_count  >> 24));
    crc = crc16_ccitt(crc, (uint8_t)(s_spike_accel_count));
    crc = crc16_ccitt(crc, (uint8_t)(s_spike_accel_count >>  8));
    crc = crc16_ccitt(crc, (uint8_t)(s_spike_accel_count >> 16));
    crc = crc16_ccitt(crc, (uint8_t)(s_spike_accel_count >> 24));
    crc = crc16_ccitt(crc, (uint8_t)(s_buf_count));
    crc = crc16_ccitt(crc, (uint8_t)(s_buf_count         >>  8));
    crc = crc16_ccitt(crc, (uint8_t)(s_buf_count         >> 16));
    crc = crc16_ccitt(crc, (uint8_t)(s_buf_count         >> 24));
    crc = crc16_ccitt(crc, (uint8_t)(s_buf_head));
    crc = crc16_ccitt(crc, (uint8_t)(s_buf_head          >>  8));
    crc = crc16_ccitt(crc, (uint8_t)(s_buf_head          >> 16));
    crc = crc16_ccitt(crc, (uint8_t)(s_buf_head          >> 24));
    // MA-1: fault shadow pair
    crc = crc16_ccitt(crc, (uint8_t)(s_fault_mask));
    crc = crc16_ccitt(crc, (uint8_t)(s_fault_mask        >>  8));
    crc = crc16_ccitt(crc, (uint8_t)(s_fault_mask_inv));
    crc = crc16_ccitt(crc, (uint8_t)(s_fault_mask_inv    >>  8));
    // MA-1: key threshold bytes (36 B total: spike counts 8B + buf state 8B + fault shadow 4B
    //        + omega_stable_dps 4B + spike_accel_g 4B + spread_max_deg 4B + anchor_max_deg 4B)
    uint32_t tmp;
    memcpy(&tmp, (const void*)&g_safety_thresholds.omega_stable_dps, 4U);
    crc = crc16_ccitt(crc, (uint8_t)(tmp));
    crc = crc16_ccitt(crc, (uint8_t)(tmp >>  8));
    crc = crc16_ccitt(crc, (uint8_t)(tmp >> 16));
    crc = crc16_ccitt(crc, (uint8_t)(tmp >> 24));
    memcpy(&tmp, (const void*)&g_safety_thresholds.spike_accel_g, 4U);
    crc = crc16_ccitt(crc, (uint8_t)(tmp));
    crc = crc16_ccitt(crc, (uint8_t)(tmp >>  8));
    crc = crc16_ccitt(crc, (uint8_t)(tmp >> 16));
    crc = crc16_ccitt(crc, (uint8_t)(tmp >> 24));
    memcpy(&tmp, (const void*)&g_safety_thresholds.spread_max_deg, 4U);
    crc = crc16_ccitt(crc, (uint8_t)(tmp));
    crc = crc16_ccitt(crc, (uint8_t)(tmp >>  8));
    crc = crc16_ccitt(crc, (uint8_t)(tmp >> 16));
    crc = crc16_ccitt(crc, (uint8_t)(tmp >> 24));
    memcpy(&tmp, (const void*)&g_safety_thresholds.anchor_max_deg, 4U);
    crc = crc16_ccitt(crc, (uint8_t)(tmp));
    crc = crc16_ccitt(crc, (uint8_t)(tmp >>  8));
    crc = crc16_ccitt(crc, (uint8_t)(tmp >> 16));
    crc = crc16_ccitt(crc, (uint8_t)(tmp >> 24));
    return crc;
}

static void refresh_safety_crc_internal() { s_safety_crc = compute_safety_crc(); }
void        refresh_safety_crc()          { s_safety_crc = compute_safety_crc(); }
bool        safety_crc_ok()               { return compute_safety_crc() == s_safety_crc; }

// --- RAM march test (March C-) -----------------------------------------------
bool ram_march_test()
{
    static volatile uint8_t march_buf[MARCH_BUF_BYTES];
    const uint16_t N = MARCH_BUF_BYTES;
    uint16_t i;
    for (i = 0U; i < N; ++i) march_buf[i] = 0x00U;
    for (i = 0U; i < N; ++i) { if (march_buf[i] != 0x00U) return false; march_buf[i] = 0xFFU; }
    for (i = 0U; i < N; ++i) { if (march_buf[i] != 0xFFU) return false; march_buf[i] = 0x00U; }
    for (i = N; i-- > 0U; )  { if (march_buf[i] != 0x00U) return false; march_buf[i] = 0xFFU; }
    for (i = N; i-- > 0U; )  { if (march_buf[i] != 0xFFU) return false; march_buf[i] = 0x00U; }
    for (i = 0U; i < N; ++i) if (march_buf[i] != 0x00U) return false;
    return true;
}

// --- Stack watermark ---------------------------------------------------------
static uint32_t s_stack_base   = 0U;
static uint32_t s_stack_min_sp = 0xFFFFFFFFUL;

void stack_watermark_update()
{
    uint32_t sp;
    asm volatile("mov %0, sp" : "=r"(sp));
    if (sp < s_stack_min_sp) s_stack_min_sp = sp;
}

uint32_t stack_used_bytes()
{
    return (s_stack_base > s_stack_min_sp) ? (s_stack_base - s_stack_min_sp) : 0U;
}

void stack_base_capture()
{
    uint32_t sp;
    asm volatile("mov %0, sp" : "=r"(sp));
    s_stack_base = sp;
}

// --- Shared volatile state ---------------------------------------------------
volatile bool g_output_inhibit = false;
volatile bool g_platform_stable = false;

// --- Output control (CR-4: independent GPIO readback via SAFE_MON_PIN_HW) ----
void set_safe_output(bool stable)
{
    if (g_output_inhibit) {
        digitalWrite(LED_PIN,      LOW);
        digitalWrite(SAFE_OUT_PIN, LOW);
        fault_clr(FAULT_OUTPUT);
        return;
    }

    digitalWrite(LED_PIN,      stable ? HIGH : LOW);
    digitalWrite(SAFE_OUT_PIN, stable ? HIGH : LOW);
    delayMicroseconds(10U);

    bool mon_high    = (digitalRead(SAFE_MON_PIN_HW) == HIGH);
    bool readback_ok = (mon_high == stable);

    if (!readback_ok) {
        if (!stable) {
            fault_set(FAULT_INTEGRITY);
        } else {
            fault_set(FAULT_OUTPUT);
        }
    } else if (stable) {
        fault_clr(FAULT_OUTPUT);
    }
}

// --- Fatal handler -----------------------------------------------------------
void fatal(const char *msg)
{
    digitalWrite(LED_PIN,      LOW);
    digitalWrite(SAFE_OUT_PIN, LOW);
    Serial.print("[FATAL] "); Serial.println(msg);
    for (;;) { __asm__ volatile ("nop" ::: "memory"); }
}
