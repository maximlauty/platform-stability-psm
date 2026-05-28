#pragma once
// safety_monitor.h -- WDT, fault register, CRC, RAM march, stack, output.
// ASIL D / SIL 3 layer. No app/ includes.
#include "safety_defs.h"

// --- Shared volatile state (read by app layer via SafetyStatus, written here) --
extern volatile bool   g_output_inhibit;
extern volatile bool   g_platform_stable;

// --- Watchdog ---
void wdt_begin(uint16_t timeout_ms);
void wdt_feed();

// --- Fault register (shadow pair) ---
void fault_set(uint16_t bits);
void fault_clr(uint16_t bits);
void fault_reset_all();
bool fault_shadow_ok();
// ISR-safe variants (no noInterrupts guard):
void fault_set_from_isr(uint16_t bits);
void fault_clr_from_isr(uint16_t bits);
// Direct read (used by stability + ISR code):
uint16_t fault_mask_get();

// --- CRC integrity ---
uint16_t crc16_ccitt(uint16_t crc, uint8_t byte);
uint16_t compute_safety_crc();
void     refresh_safety_crc();
bool     safety_crc_ok();

// --- RAM march test ---
bool ram_march_test();

// --- Stack watermark ---
void     stack_watermark_update();
uint32_t stack_used_bytes();
void     stack_base_capture();   // called once in setup() to record baseline SP

// --- Output control ---
void set_safe_output(bool stable);

// --- Fatal handler ---
void fatal(const char *msg);
