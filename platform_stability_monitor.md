# platform_stability_monitor.md

Platform Stability Monitor — Teensy 4.1 + BNO085  
Technical Reference — Rev 1.0 (post-remediation, SIL 2 audit applied)

---

## Table of Contents

1. [System Overview](#1-system-overview)
2. [Hardware Components](#2-hardware-components)
3. [Software Architecture](#3-software-architecture)
4. [Safety Mechanisms](#4-safety-mechanisms)
5. [Data Flow](#5-data-flow)
6. [Failure Modes & Risks](#6-failure-modes--risks)
7. [Limitations / Known Issues](#7-limitations--known-issues)

---

## 1. System Overview

The Platform Stability Monitor (PSM) determines whether a physical platform is stationary and level. It samples a BNO085 IMU at 100 Hz, applies a multi-criterion stability algorithm over a rolling 1–5 s window, and asserts a single digital output (`SAFE_OUT_PIN`) HIGH when the platform is confirmed stable.

The system is intended to approach **IEC 61508 SIL 2**, meaning it must tolerate random hardware faults and systematic software failures with a probability of dangerous failure per hour (PFH) ≤ 10⁻⁶. It does **not** currently achieve certified SIL 2 — specific gaps are documented in §7.

**Primary decision boundary:** `g_platform_stable` is set TRUE only when all of the following hold simultaneously:

- No active fault flags (`s_fault_mask == 0`)
- Fault shadow register is consistent (`fault_shadow_ok()`)
- CRC-16 over critical counters passes (`safety_crc_ok()`)
- Window buffer is fully populated
- Gyro spike count ≤ `motion_samples_max`
- Accel spike count ≤ `shock_samples_max`
- Tilt spread (centroid radius) ≤ `spread_max_deg`
- Instant-hold timer expired
- ISR frozen-watchdog flag clear
- Absolute tilt anchor within `anchor_max_deg`

**Fail-safe direction:** any fault, communications failure, hardware error, or software integrity violation drives the output LOW (not stable). The system is fail-safe by default.

```
┌─────────────────────────────────────────────────────────────┐
│                    Platform Stability Monitor                │
│                                                             │
│   ┌──────────┐   I2C 400kHz   ┌──────────────────────────┐ │
│   │  BNO085  │◄──────────────►│       Teensy 4.1          │ │
│   │  IMU     │                │   (iMXRT1062 Cortex-M7)  │ │
│   └──────────┘                │                          │ │
│                               │  ┌────────┐ ┌─────────┐ │ │
│   ┌──────────┐                │  │IntervalTimer ISR    │ │ │
│   │SAFE_OUT  │◄───────────────│  │100 Hz  │ │WDT 1.5s │ │ │
│   │PIN 2     │                │  └────────┘ └─────────┘ │ │
│   └─────┬────┘                │                          │ │
│         │ 100Ω ESD            │  ┌──────────────────────┐│ │
│   ┌─────▼────┐                │  │ Stability Algorithm  ││ │
│   │SAFE_MON  │────────────────│  │ + Fault Monitor      ││ │
│   │PIN 4     │  readback      │  └──────────────────────┘│ │
│   └──────────┘                │                          │ │
│                               │  ┌──────────────────────┐│ │
│   ┌──────────┐                │  │  HTTP/1.1 API        ││ │
│   │ Ethernet │◄───────────────│  │  (QNEthernet)        ││ │
│   │ (ENET)   │                │  └──────────────────────┘│ │
│   └──────────┘                └──────────────────────────┘ │
└─────────────────────────────────────────────────────────────┘
```

---

## 2. Hardware Components

### 2.1 Teensy 4.1 — MCU (iMXRT1062)

**What it is:** NXP iMXRT1062 Cortex-M7 at 600 MHz. 1 MB SRAM (DTCM + OCRAM), 8 MB flash (QSPI), hardware FPU (double-precision), DWT cycle counter.

**Why used:** High clock speed enables floating-point stability math in the 100 Hz sample loop without timing margin issues. Teensyduino provides a mature Arduino-compatible HAL covering I2C, timers, interrupts, and EEPROM emulation. Built-in ENET MAC eliminates an external Ethernet chip.

**Internal features used:**

| Feature | Usage |
|---|---|
| `IntervalTimer` | 100 Hz ISR via FlexTimer/PIT peripheral |
| `RTWDOG3` (WDOG3) | Watchdog — 1500 ms timeout, LPO 1 kHz clock |
| `SRC_SRSR` | System Reset Controller — reset-cause register, W1C |
| `ARM_DWT_CYCCNT` | DWT cycle counter — ISR period measurement |
| `EEPROM` (emulated) | Persistent parameter storage (FlexNVM) |
| `Wire` (I2C1) | SDA=18, SCL=19, 400 kHz |
| ENET MAC | Built-in Ethernet controller (QNEthernet driver) |
| GPIO | `SAFE_OUT_PIN` (2), `SAFE_MON_PIN` (4), `BNO_RST_PIN` (3), LED (13) |

**Stack:** Default Teensyduino stack is 8 KB. PSM monitors usage via a watermark; warns (FAULT_INTEGRITY) if > 6 KB consumed.

**Reset-cause logging:** At `setup()`, `SRC_SRSR` is read and cleared (W1C). The decoded cause (POR, WDOG3, WDOG1/2, user reset, ENET WDOG2) is printed to Serial. This is the only persistent reset-cause record; no EEPROM logging is implemented.

---

### 2.2 BNO085 IMU

**What it is:** Bosch BNO085 (SH-2 sensor hub). Contains a 3-axis accelerometer, 3-axis gyroscope, and an internal ARM Cortex-M0+ running Bosch's SHTP sensor fusion firmware.

**Why used:** The on-chip fusion firmware provides calibrated gyro/accel reports and a Game Rotation Vector (quaternion) without a separate AHRS implementation on the host. The calibration engine continuously corrects gyro bias.

**Communication:** I2C at 400 kHz. Address `0x4A` (SA0=LOW) or `0x4B` (SA0=HIGH). SparkFun BNO08x v2.x library (SH-2 SHTP protocol). Three reports subscribed at 100 Hz (10 ms period):

| Report ID | Data | Conversion |
|---|---|---|
| `SENSOR_REPORTID_GYROSCOPE_CALIBRATED` | ωx, ωy, ωz rad/s | × (180/π) → dps |
| `SENSOR_REPORTID_ACCELEROMETER` | ax, ay, az m/s² | ÷ 9.80665 → g |
| `SENSOR_REPORTID_GAME_ROTATION_VECTOR` | quaternion (w, i, j, k) | ZYX Euler → roll/pitch/yaw |

**DCD calibration pause:** The BNO085 internal watchdog periodically writes dynamic calibration data (DCD) to internal flash. During this ~400 ms save the Game Rotation Vector report can pause. To avoid false COMM faults, COMM fault is gated only on gyro+accel staleness, not GRV. If GRV is stale, tilt falls back to raw accelerometer gravity decomposition.

**Hardware reset:** `BNO_RST_PIN` (pin 3) is driven LOW for 10 ms to hardware-reset the BNO085. Used at startup and in `imu_recover()`. Boot time after RST de-assertion is ~300 ms.

**I2C recovery:** If the I2C bus hangs (SDA stuck low), `wire_reset()` bit-bangs 9 SCL pulses to release the device, then generates a STOP condition and re-initialises `Wire`. This is sufficient for most stuck-device scenarios but is not a guaranteed bus recovery for all fault modes.

---

### 2.3 SAFE_OUT_PIN / SAFE_MON_PIN — Safety Output Circuit

**What it is:** A GPIO output (`SAFE_OUT_PIN`, pin 2) asserts HIGH when `g_platform_stable == true`. A separate GPIO input (`SAFE_MON_PIN`, pin 4, `INPUT_PULLDOWN`) is wired to the same net and reads back the actual voltage on that net.

**Why used:** Detects output driver faults (stuck-at-HIGH, stuck-at-LOW) that a pure software flag cannot detect. Required for SIL 2 — output integrity must be independently verified.

**Wiring requirement:**  
`SAFE_MON_PIN` must be wired directly to the `SAFE_OUT_PIN` net. Use ≤100 Ω series (ESD only). Do **not** use 10 kΩ — the internal PULLDOWN (~10 kΩ) creates a voltage divider that may not cross the logic-HIGH threshold.

**Fault policy:**

```
set_safe_output(stable):
  drive SAFE_OUT_PIN = stable
  read SAFE_MON_PIN → mon_high

  if (stable == true) && (mon_high == false):
      fault_set(FAULT_OUTPUT)      ← net LOW when HIGH commanded
                                      (wire missing or driver stuck LOW)
  if (stable == false) && (mon_high == true):
      fault_set(FAULT_INTEGRITY)   ← net HIGH when LOW commanded — CRITICAL
                                      (driver or external load stuck HIGH)
  if (stable == true) && (mon_high == true):
      fault_clr(FAULT_OUTPUT)      ← confirm wire functional

  NOTE: FALSE readback pass does NOT clear FAULT_OUTPUT.
        Only a TRUE readback pass clears it. Prevents TRUE→FALSE
        oscillation from masking a persistent wiring fault.
```

---

### 2.4 Ethernet Interface (QNEthernet)

**What it is:** Teensy 4.1's built-in ENET MAC + external PHY, driven by the QNEthernet library (lwIP-based). Static IP: `192.168.168.71/24`.

**Why used:** Provides a web-based dashboard and REST API for monitoring and parameter tuning without a USB or serial connection.

**Safety note:** Ethernet MAC bring-up can take >1500 ms. It is initialised *before* the watchdog is armed to prevent a false WDT reset during startup.

---

## 3. Software Architecture

### 3.1 Single-File Structure

The entire application is one `.ino` file (`platform_stability.ino`). No RTOS. Execution model is `setup()` + bare-metal `loop()` with one ISR.

```
setup():
  GPIO init → RAM march test → DWT enable → EEPROM params load
  → reset-cause log → I2C init → BNO085 init+enable → wait first reports
  → Ethernet init → WDT arm → stack base capture → timer ISR start

loop() [runs continuously, ~>1000 Hz unloaded]:
  wdt_feed()
  s_loop_alive_token++          ← ISR frozen-watchdog keepalive
  check s_forced_unsafe_isr
  web server (non-blocking)
  heartbeat (1 Hz Serial)
  atomic read s_timing_fault_isr
  if s_sample_due > 0:
      bno_poll()                ← drain SH2 events
      data freshness check → COMM fault
      sensor range checks → GYRO_RANGE / ACCEL_RANGE faults
      push_sample()             ← buffer + CRC refresh
  if eval period (200 ms):
      imu_recover() if needed
      stack watermark update
      compute_window_tilt_spread()
      diverse_stable() → FAULT_DIVERSE
      evaluate_stability()
      anchor check
      set_safe_output()
      Serial status line
```

### 3.2 ISR (`timerISR`)

Fires at exactly 100 Hz via `IntervalTimer` (Teensyduino FlexTimer/PIT). Two responsibilities:

**1. Sample tick:**  
Increments `s_sample_due` (capped at 255). `loop()` atomically reads and clears it. If `s_sample_due > 1` on read, samples were missed and `s_missed_samples` is incremented.

**2. ISR-period timing check:**  
Uses `ARM_DWT_CYCCNT` (600 MHz cycle counter) to measure the interval between consecutive ISR firings. Expected: 600,000,000 / 100 = 6,000,000 cycles. Tolerance: ±5% = ±300,000 cycles. Jitter beyond this sets `s_timing_fault_isr` (atomic: cleared by `loop()` with `noInterrupts()`).

**3. Frozen-loop watchdog:**  
Every ISR tick checks whether `s_loop_alive_token` changed since last tick. If unchanged for `FROZEN_TICKS_MAX` = 26 consecutive ticks (~260 ms), the ISR directly drives `SAFE_OUT_PIN` and LED LOW using `digitalWriteFast()`, and sets `s_forced_unsafe_isr = true`. This forces unsafe output well before the 1500 ms WDT fires.

```
timerISR() [@ 100 Hz]:
  DWT delta check → s_timing_fault_isr
  s_sample_due++
  __DSB()                     ← memory barrier before token check
  if token unchanged for FROZEN_TICKS_MAX ticks:
      digitalWriteFast(SAFE_OUT_PIN, LOW)
      s_forced_unsafe_isr = true   ← sticky, cleared only by imu_recover()
```

**ISR constraints:** No heap allocation, no blocking calls, no I2C. `digitalWriteFast()` is used (register-direct, no locking). Shared variables accessed from `loop()` are wrapped in `noInterrupts()/interrupts()`.

### 3.3 Stability Algorithm

Executed every `eval_period_ms` = 200 ms after the sample buffer is sufficiently populated.

**Step 1 — Buffer fill:**  
100 Hz samples accumulate in a circular buffer (`s_tilt_buf`, `s_accel_mag_buf`), `WINDOW_SAMPLES_MAX` = 500 entries. Default window = 1 s = 100 samples. Window is tunable 1–5 s via EEPROM.

**Step 2 — Spike counting:**  
Per-sample slot arrays (`s_gyro_spike_slot`, `s_accel_spike_slot`) track whether each slot contained a spike. When a slot is overwritten, its old spike contribution is decremented before writing the new one, keeping `s_spike_gyro_count` and `s_spike_accel_count` accurate without a full window scan.

- Gyro spike: `|ω|` > `omega_stable_dps` (20 dps)
- Accel spike: `| |a| − 1g |` > `spike_accel_g` (0.1 g)

**Step 3 — Instant hold:**  
If `|ω|` > `omega_instant_dps` (45 dps), a hold timer is set for `instant_hold_ms` (250 ms). `evaluate_stability()` returns false until the timer expires. Protects against transient spikes that clear the spike window before evaluation.

**Step 4 — Tilt computation:**  
Preferred: quaternion from GRV report → ZYX Euler (roll, pitch, yaw). Fallback (GRV stale): atan2-based accel decomposition for roll/pitch; yaw held at last known value.

Yaw is **excluded** from all stability distance calculations — gyro-integrated yaw accumulates drift and conflates platform rotation with instability. Only roll and pitch enter `tilt_distance_deg()`.

**Step 5 — Centroid spread:**  
`compute_window_tilt_spread()` computes the mean roll/pitch over the window, then returns the maximum Euclidean distance from any sample to that centroid. This is order-independent (not biased by the oldest sample). Limit: `spread_max_deg` = 7.5° (centroid radius ≈ 15° diameter equivalent).

**Step 6 — Diversity check:**  
`diverse_stable()` computes the mean accelerometer magnitude over the window and checks it is within `diverse_mean_tol_g` (0.1 g) of 1g. Detects free-fall, sustained vibration, or a non-gravity-aligned sensor. Sets `FAULT_DIVERSE`.

**Step 7 — `evaluate_stability()`:**  
```
evaluate_stability(spread):
  if !fault_shadow_ok()                 → FAULT_INTEGRITY, return false
  if !safety_crc_ok()                   → FAULT_INTEGRITY, return false
  if s_fault_mask != 0                  → return false
  if instant_hold active                → return false
  if buf_count < window_samples         → return false
  if gyro_spikes > motion_samples_max   → return false
  if accel_spikes > shock_samples_max   → return false
  if spread > spread_max_deg            → return false
  return true
```

**Step 8 — Anchor:**  
On first TRUE result, the current (roll, pitch) is stored as `s_stable_anchor`. On every subsequent TRUE evaluation, the current tilt is checked against the anchor. If drift exceeds `anchor_max_deg` (8°), the platform is declared unstable and the anchor is invalidated. This catches slow creep that doesn't exceed the window spread threshold.

### 3.4 EEPROM Parameters

All stability thresholds are stored in a `Params` struct (EEPROM address 0) with magic `0xBE4F0001` and version `2`. On load, `params_valid()` checks:

- Magic + version match
- `isfinite()` on all float fields (rejects NaN/Inf before range comparisons — NaN passes all `<`/`>` checks silently)
- Range bounds for all fields

If validation fails, defaults (`PARAMS_DEFAULT`) are used. Version was bumped to `2` when `spread_max_deg` semantics changed from diameter to centroid radius; this invalidates any v1 EEPROM stored by a previous firmware.

Parameters are updated via HTTP `POST /config` with auth token. The update path: parse → `params_valid()` → apply to `g_params` → `params_save()` → timer stop → state reset → timer restart.

### 3.5 HTTP API

Hosted on port 80 at `192.168.168.71`. Single-client (one `EthernetClient` slot). Idle client timeout: 500 ms.

| Endpoint | Method | Purpose |
|---|---|---|
| `/` | GET | HTML dashboard (inline JS + CSS) |
| `/api/status` | GET | JSON: stable, fault, why, fill, spikes, angles, spread |
| `/api/params` | GET | JSON: current `g_params` |
| `/api/js` | GET | Polling JS for dashboard (embeds auth token) |
| `/config` | POST | Update + save parameters (auth required) |
| `/test/toggle-output` | POST | Toggle `g_output_inhibit` for bench testing (auth required) |

WDT feeds are placed: entry, after header read, after body read, and around each `flush()`. Auth is a shared secret (`CONFIG_AUTH_TOKEN`) delivered in the POST body URL-encoded. **Default token `"psm-change-me-v1"` must be changed before deployment.**

`g_output_inhibit` suppresses the physical output for bench testing. It is `volatile` and **not** EEPROM-backed — it clears on every power cycle and WDT reset. Stability evaluation continues normally when inhibit is active.

---

## 4. Safety Mechanisms

### 4.1 Watchdog Timer (RTWDOG3)

**What:** iMXRT1062 RTWDOG3 (NXP RM RTWDOG peripheral, Teensyduino name `WDOG3`). Clocked from the 1 kHz LPO (Low-Power Oscillator). TOVAL register value = timeout in milliseconds.

**Configuration:**  
```
Timeout:    1500 ms
Clock:      LPO 1 kHz (CLK=01)
CMD32EN:    1  (32-bit refresh word required)
UPDATE:     1  (allows reconfiguration)
Unlock:     two consecutive 32-bit writes (0xD928C520, 0xB480A602)
Refresh:    single 32-bit write (0xB480A602, CMD32EN=1 mode)
```

**Placement:**  
- Armed **after** all init (Ethernet MAC can take >1500 ms).
- `wdt_feed()` called: start of every `loop()` iteration, at `bno_poll()` entry, in `imu_recover()` around each blocking delay, and around each `flush()` in `http_handle()`.

**Interaction with ISR frozen-watchdog:**  
The ISR fires unsafe output at ~260 ms of loop freeze. WDT resets the MCU at 1500 ms. The ISR path is faster and avoids a full system reset for transient freezes.

**Reset-cause:** WDOG3 resets set bit 5 of `SRC_SRSR`. Logged at next startup.

**Limitations:**  
- Single internal watchdog. For SIL 2, an independent external watchdog IC (e.g., MAX706) windowed against the internal WDT provides fail-safe coverage for internal WDT faults.
- WDT is not set to window mode — a runaway loop that feeds the WDT at high frequency would not be caught.

### 4.2 Complementary Shadow Register (Fault Mask)

**What:** `s_fault_mask` (uint8_t) stores active fault bits. `s_fault_mask_inv` stores its bitwise complement. Every `fault_set()` and `fault_clr()` updates both.

**Check:** `fault_shadow_ok()` returns `(s_fault_mask ^ s_fault_mask_inv) == 0xFF`. Any single-bit flip in either variable (RAM fault, stack corruption, compiler scheduling issue) causes a mismatch, which triggers `FAULT_INTEGRITY` and blocks stable output.

**Coverage:** Detects stuck-at faults in the fault register itself. Does not cover the tilt buffer or spike count arrays.

**ISR note:** `fault_set/clr` are called only from `loop()`. If ever called from ISR, the call site must be wrapped in `noInterrupts()/interrupts()` to prevent interleaved shadow corruption.

### 4.3 CRC-16 (CCITT-FALSE)

**What:** CRC-16 with polynomial `0x1021`, initialisation `0xFFFF`. Covers 16 bytes:

```
s_spike_gyro_count   (4 bytes, little-endian)
s_spike_accel_count  (4 bytes, little-endian)
s_buf_count          (4 bytes, little-endian)
s_buf_head           (4 bytes, little-endian)
```

**Why CRC-16:** Hamming Distance 4 — detects all 1-, 2-, and 3-bit errors in covered data. Replaced a narrower CRC-8 during remediation.

**Refresh:** `refresh_safety_crc()` is called every time any of the four covered variables changes (`push_sample()`, `reset_stability_state()`). `evaluate_stability()` calls `safety_crc_ok()` (recomputes and compares) before any stability decision.

**Coverage gaps:**  
- `s_tilt_buf` and `s_accel_mag_buf` are not CRC-covered.
- `g_params` is validated once on EEPROM load; in-RAM corruption is not detected between evaluations.
- `g_platform_stable` is re-derived each eval cycle from covered state, so its stored value is not directly CRC-checked (acceptable: it has a very short lifetime between evaluations).

### 4.4 RAM March Test

**What:** March C- algorithm over a dedicated 128-byte buffer (`march_buf`), run once at startup before any other logic.

**Algorithm:**
```
Phase 0 (↑): write 0x00 to all cells
Phase 1 (↑): read 0x00, write 0xFF
Phase 2 (↑): read 0xFF, write 0x00
Phase 3 (↓): read 0x00, write 0xFF
Phase 4 (↓): read 0xFF, write 0x00
Phase 5 (↑): read 0x00
```

Detects stuck-at-0, stuck-at-1, transition faults, and address decoder coupling faults.

**On failure:** LED blinks rapidly; system halts. Does not attempt recovery.

**Coverage gap (MI-2):** Only covers the 128-byte `march_buf`. The safety-critical DTCM globals (`s_fault_mask`, `s_tilt_buf`, `g_params`, etc.) are not march-tested. Full SIL 2 RAM coverage requires a linker-placed march over the entire DTCM data region at startup.

### 4.5 ISR Frozen-Loop Watchdog

**What:** At each 100 Hz ISR tick, `s_loop_alive_token` (written by `loop()` each iteration) is compared against `s_isr_token_last`. If unchanged for `FROZEN_TICKS_MAX` = 26 consecutive ticks (~260 ms):

- `digitalWriteFast(SAFE_OUT_PIN, LOW)` — drives output unsafe directly from ISR
- `s_forced_unsafe_isr = true` — sticky flag, checked at top of every `loop()` iteration

**Recovery:** Only `imu_recover()` clears `s_forced_unsafe_isr`, and only after a successful IMU re-init. This prevents a post-freeze resume from automatically re-asserting stable output.

**Why 260 ms threshold:** Faster than the 1500 ms WDT. Absorbs short I2C delays (typical `bno_poll()` < 5 ms) while catching genuine hangs quickly.

### 4.6 ISR Timing Monitor

**What:** DWT `ARM_DWT_CYCCNT` is sampled at each ISR entry. The expected period is `F_CPU / ODR_HZ` = 6,000,000 cycles. Tolerance: ±5% = ±300,000 cycles.

**On violation:** `s_timing_fault_isr = true`. `loop()` reads it atomically (`noInterrupts()`) and sets `FAULT_TIMING`. After `clean_streak_needed` (5) consecutive clean ticks, `FAULT_TIMING` is cleared.

**Purpose:** Detects interrupt priority inversion, OS preemption (none here), or timer mis-configuration that could cause samples to be timestamped incorrectly.

### 4.7 Output Readback (SAFE_MON_PIN)

Described in §2.3. Provides hardware-level output verification that is independent of the software flag. Two distinct fault bits (`FAULT_OUTPUT`, `FAULT_INTEGRITY`) distinguish "output cannot assert HIGH" from the critical "output cannot deassert LOW".

### 4.8 Sensor Range Faults

| Fault | Condition | Cleared after |
|---|---|---|
| `FAULT_GYRO_RANGE` | `isnan(ω)` or `ω > 248 dps` (99% of 250 dps FS) | `clean_streak_needed` = 5 clean samples |
| `FAULT_ACCEL_RANGE` | `isnan(|a|)` or `|a| < 0.05g` or `|a| > 4.4g` | `clean_streak_needed` = 5 clean samples |
| `FAULT_IMU_COMM` | ≥ `stale_fault_ticks` (60) consecutive stale gyro+accel reports | Cleared by `imu_recover()` only |
| `FAULT_DIVERSE` | Window mean accel mag not within 0.1g of 1g | `clean_streak_needed` = 5 clean evals |

**COMM fault latency note (MA-6):** Stale data drives the output FALSE on the **first** stale tick. The 60-tick (~600 ms) window only delays activation of `imu_recover()`, which performs a hardware reset and re-init (380 ms blocking). During those 600 ms the system is already in the safe state.

### 4.9 IMU Recovery

`imu_recover()` is called from the 200 ms evaluation loop when `FAULT_IMU_COMM` or `FAULT_FROZEN` is active. Exponential back-off: `min(3000 × 2^n, 30000)` ms.

```
imu_recover():
  stop IntervalTimer
  set_safe_output(false)
  RST pin LOW 10 ms → HIGH
  delay 300 ms (BNO085 boot)
  wire_reset()
  delay 20 ms
  IMU.begin() + bno_enable_reports()
  wait for first gyro+accel reports (3 s timeout)
  fault_reset_all()
  reset_stability_state()
  s_forced_unsafe_isr = false   ← ONLY place this is cleared
  restart IntervalTimer
  return true/false
```

### 4.10 Reset-Cause Logging

`SRC_SRSR` bits decoded at startup:

| Bit | Cause |
|---|---|
| 7 | WDOG2 (ENET MAC internal reset) |
| 5 | WDOG3 (PSM watchdog) |
| 4 | WDOG1/2 |
| 3 | IPP user reset |
| 0 | Power-on reset (POR) |

Logged to Serial only. Not stored in EEPROM. Monitoring of WDT resets in production requires a connected Serial terminal or an EEPROM log extension.

### 4.11 Stack Watermark

SP is captured in `setup()` as `s_stack_base`. Each 200 ms eval period, `stack_watermark_update()` captures current SP and updates `s_stack_min_sp`. If `s_stack_base - s_stack_min_sp > STACK_WARN_BYTES` (6144), `FAULT_INTEGRITY` is set. Teensy 4.1 default stack is 8 KB.

### 4.12 Hang Checkpoint

`s_hang_cp` is set before each potentially blocking call and cleared after:

| Value | Location |
|---|---|
| 0 | OK / idle |
| 1 | Inside `bno_poll()` |
| 2 | (reserved) |
| 3 | Inside status-line `Serial.print` |

Visible in the heartbeat log and the Serial status line as `cp=N`. If the WDT fires, the last `cp` value identifies the hang site.

### 4.13 EEPROM Config Authentication

`POST /config` and `POST /test/toggle-output` require `auth=<CONFIG_AUTH_TOKEN>` in the URL-encoded body. Requests with missing or wrong tokens receive HTTP 403 and leave `g_params` unchanged. Invalid config values are rejected by `params_valid()` before they reach runtime.

---

## 5. Data Flow

### 5.1 100 Hz Sample Path

```
IntervalTimer ISR (10 ms)
│
├─ increment s_sample_due
├─ DWT timing check → s_timing_fault_isr
└─ frozen-loop check → s_forced_unsafe_isr

loop() detects s_sample_due > 0:
│
├─ bno_poll()
│   └─ I2C read SH2 events
│       ├─ GYRO report → s_bno_g{x,y,z}_dps, s_bno_gyro_ms
│       ├─ ACCEL report → s_bno_a{x,y,z}_g, s_bno_accel_ms
│       └─ GRV report → s_bno_q{w,i,j,k}, s_bno_grv_ms
│
├─ freshness check (DATA_FRESH_MS = 150 ms)
│   ├─ stale → increment s_comm_stale_ticks → FAULT_IMU_COMM after 60 ticks
│   └─ fresh → clear s_comm_stale_ticks
│
├─ sensor range checks → FAULT_GYRO_RANGE / FAULT_ACCEL_RANGE
│
├─ tilt from GRV quaternion (or accel fallback)
│
├─ push_sample(tilt, omega, accel_mag)
│   ├─ write circular buffer slot
│   ├─ update s_spike_gyro_count, s_spike_accel_count
│   └─ refresh_safety_crc()
│
└─ update instant-hold timer
```

### 5.2 200 ms Evaluation Path

```
loop() @ eval_period_ms (200 ms):
│
├─ imu_recover() if COMM/FROZEN fault [exponential back-off]
├─ stack_watermark_update()
│
├─ compute_window_tilt_spread()
│   └─ centroid of roll/pitch → max radius
│
├─ diverse_stable() → FAULT_DIVERSE
│
├─ evaluate_stability(spread) [all-pass gate]:
│   ├─ fault_shadow_ok() → FAULT_INTEGRITY
│   ├─ safety_crc_ok()   → FAULT_INTEGRITY
│   ├─ s_fault_mask == 0
│   ├─ instant-hold timer
│   ├─ buffer fill
│   ├─ gyro spike count
│   ├─ accel spike count
│   └─ spread limit
│
├─ anchor check → invalidate if drift > anchor_max_deg
│
└─ set_safe_output(g_platform_stable)
    ├─ drive SAFE_OUT_PIN
    └─ readback SAFE_MON_PIN → FAULT_OUTPUT / FAULT_INTEGRITY
```

### 5.3 Fault Handling Paths

```
Fault detected
      │
      ▼
fault_set(FAULT_xxx) ──────────────────────────────────────────┐
      │                                                         │
      ▼                                                         │
evaluate_stability() returns false ◄───────────────────────────┘
      │
      ▼
g_platform_stable = false
      │
      ▼
set_safe_output(false) → SAFE_OUT_PIN = LOW

If FAULT_IMU_COMM or FAULT_FROZEN:
      │
      ▼
imu_recover() [back-off: 3s, 6s, 12s, 24s, 30s max]
      │
      ├── success: fault_reset_all() + reset_stability_state()
      │            s_forced_unsafe_isr = false
      │            → resume sampling
      │
      └── failure: retry at next back-off interval
                   SAFE_OUT_PIN stays LOW

If FAULT_INTEGRITY:
      │
      └── No auto-recovery. Requires power cycle or WDT reset.
          (WDT fires at 1500 ms if loop continues without fix)
```

---

## 6. Failure Modes & Risks

### 6.1 Fault Code Reference

| Bit | Name | Trigger | Recovery |
|---|---|---|---|
| `0x01` | `FAULT_IMU_COMM` | 60 consecutive stale gyro+accel ticks (~600 ms) | `imu_recover()` |
| `0x02` | `FAULT_GYRO_RANGE` | `|ω| > 248 dps` or NaN | 5 clean samples |
| `0x04` | `FAULT_ACCEL_RANGE` | `|a| < 0.05g` or `> 4.4g` or NaN | 5 clean samples |
| `0x08` | `FAULT_OUTPUT` | SAFE_MON reads LOW when HIGH commanded | TRUE readback pass |
| `0x10` | `FAULT_FROZEN` | Loop frozen > 260 ms (ISR watchdog) | `imu_recover()` |
| `0x20` | `FAULT_TIMING` | ISR period jitter > ±5% | 5 clean ISR ticks |
| `0x40` | `FAULT_DIVERSE` | Window mean accel ≠ 1g ± 0.1g | 5 clean eval periods |
| `0x80` | `FAULT_INTEGRITY` | Shadow mismatch, CRC fail, or stack depth > 6 KB | Power cycle / WDT |

### 6.2 "Why Not Stable" Codes

Printed in the Serial status line and returned in `/api/status`:

| Code | Meaning |
|---|---|
| `OK` | Stable |
| `DRIFT` | Anchor exceeded `anchor_max_deg` |
| `SHADOW` | Fault shadow mismatch (`FAULT_INTEGRITY`) |
| `CRC` | CRC-16 mismatch (`FAULT_INTEGRITY`) |
| `FAULT` | Any `s_fault_mask` bit set |
| `HOLD` | Instant-hold timer active |
| `FILL` | Buffer not yet full |
| `GYRO_SPK` | Gyro spike count over limit |
| `ACCEL_SPK` | Accel spike count over limit |
| `SPREAD` | Tilt spread over limit |
| `UNKNOWN` | Stable=FALSE but no specific cause found |

### 6.3 Critical Failure Modes

**BNO085 internal reset during operation:**  
`bno_poll()` detects `IMU.wasReset()` and re-enables reports. If re-enable fails, `FAULT_IMU_COMM` is set immediately. Sensor hub resets with successful re-enable do not trigger a COMM fault; report timestamps are preserved (intentional — clearing them would cause false COMM fault before new events arrive).

**I2C bus lockup:**  
`wire_reset()` bit-bangs 9 SCL pulses. Works for most single-device clock-stretch hangs. Does not guarantee recovery for multi-master arbitration failure (not applicable here) or for hardware-level shorts.

**WDT false trigger during EEPROM write:**  
`params_save()` calls `wdt_feed()` before and after `EEPROM.put()`. Flash write latency is several ms; the WDT margin is 1500 ms, so a single feed is sufficient, but the belt-and-suspenders double-feed eliminates margin concerns.

**Floating SAFE_MON_PIN (wiring omitted):**  
`INPUT_PULLDOWN` ensures an unwired pin reads LOW. A FALSE readback (output=LOW, mon=LOW) passes without fault. A TRUE readback (output=HIGH, mon=LOW) sets `FAULT_OUTPUT` — the system correctly refuses to assert stable and `FAULT_OUTPUT` remains set until the wire is installed and a TRUE readback passes.

**Stack overflow:**  
Detected at 6 KB (watermark). 8 KB Teensy 4.1 default stack. Stack overflow past 8 KB corrupts globals silently — the watermark catches deep usage before the guard zone, but does not prevent overflow if a single call frame exceeds the remaining margin.

---

## 7. Limitations / Known Issues

The following issues prevent the system from meeting IEC 61508 SIL 2 and are explicitly documented from the formal audit (11 findings, remediation steps applied).

### 7.1 RAM March Test Coverage (MI-2) — Critical

**Gap:** `ram_march_test()` covers only the 128-byte `march_buf`. Safety-critical globals in DTCM (`s_fault_mask`, `s_fault_mask_inv`, `s_tilt_buf`, `g_params`, etc.) are not marched.

**Required for SIL 2:** A linker-placed march test covering the entire DTCM data region at startup, before any variable initialisation that could mask faults.

### 7.2 No Brownout Detection / BOD (MI-8) — Major

**Gap:** The iMXRT1062 DCDC controller has a configurable brownout threshold but is not configured. The factory default trips at ~2.7 V for the 3.3 V rail. Under a slow power droop, the CPU may execute erratically before the BOD fires.

**Required for SIL 2:** Configure `DCDC_REG3[TRG]` and enable the BOD interrupt in NVIC for a controlled reset before CPU misbehaviour under supply droop.

### 7.3 Default Config Auth Token (Security / Safety)

**Gap:** `CONFIG_AUTH_TOKEN` is `"psm-change-me-v1"` and is embedded in the JavaScript served at `/api/js`. This means the token is visible in the browser and in HTTP traffic.

**Required before deployment:** Change `CONFIG_AUTH_TOKEN` to a strong random secret. Consider moving the token out of the served JS (e.g., a separate admin interface on a different port or with HTTP Digest Auth).

### 7.4 Single Watchdog, No Window Mode

**Gap:** One internal WDT (WDOG3), not windowed. A tight busy-loop that feeds the WDT at high frequency will not be caught.

**Required for SIL 2:** External watchdog IC (windowed, independent clock) or enable WDOG3 window mode so a feed that arrives too early is also flagged.

### 7.5 No Independent Diverse Channel

**Gap:** Single-channel architecture — one MCU, one IMU, one WDT. IEC 61508 SIL 2 typically requires either redundant hardware channels or a rigorous formal proof of systematic capability.

**Risk:** A common-cause failure (firmware bug, MCU hang, IMU miscommunication) can assert a false stable output without any independent check.

### 7.6 CRC Does Not Cover Tilt Buffer

**Gap:** `s_tilt_buf` and `s_accel_mag_buf` (up to 500 × 3 floats = 6 KB) are not CRC-covered. A RAM corruption in the tilt buffer could produce a false stable tilt spread result without being detected.

**Mitigation in place:** CRC covers the window head/count and spike counts. A corrupted tilt buffer that happens to produce a false spread result without changing head/count is possible.

### 7.7 No Certified Toolchain or MISRA Compliance

**Gap:** Built with GCC (Teensyduino), which is not a TÜV-certified safety compiler. Code uses C++ features (lambdas in `http_handle`, `constexpr`, templates) that are excluded by MISRA C:2012. No static analysis tool results are recorded.

**Required for SIL 2:** Either a certified compiler (e.g., TASKING, IAR with SC option) or a documented compiler qualification process. MISRA C compliance or documented deviations.

### 7.8 Reset-Cause Not Persisted to EEPROM

**Gap:** WDT resets are logged to Serial only. If no terminal is connected, WDT reset events are silent. Post-mortem diagnosis in the field is not possible.

**Recommendation:** Write `SRC_SRSR` and a timestamp (or loop count) to EEPROM on startup for offline diagnostics.

### 7.9 Auth Token in JavaScript (Security Regression)

**Gap:** `CONFIG_AUTH_TOKEN` is embedded in the JavaScript response from `/api/js` as `var PSM_AUTH='...'`. This exposes the token to any browser or network observer that loads the dashboard.

**Mitigation:** This is acceptable for a single-network embedded deployment where the dashboard is only accessible on a trusted LAN. It is not acceptable for any internet-facing deployment.

### 7.10 Yaw Excluded from Stability (Design Note, Not a Defect)

Yaw is excluded from `tilt_distance_deg()` and all spread/anchor calculations. This is intentional and correct — gyro-integrated yaw accumulates ~0.1 dps drift over the 1–5 s window, which would produce false instability signals for a platform that is physically level but slowly rotating. Yaw is retained in the `Tilt` struct and logged for diagnostics only.

---

*End of document.*
