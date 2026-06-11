# platform_stability_monitor.md

Platform Stability Monitor v2 — Teensy 4.1 + ASM330LHB × 2 (SPI)  
Technical Reference — Rev 2.0 (ASIL-D / SIL-3 refactor, dual-channel ASM330LHB)  
Date: 2026-06-03

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

The Platform Stability Monitor (PSM) determines whether a physical platform is stationary and level. It samples two independent ASM330LHB IMUs at 833 Hz over SPI, runs a Mahony complementary filter on each channel, applies a multi-criterion stability algorithm over a rolling window, and asserts a single digital output (`SAFE_OUT_PIN`) HIGH when the platform is confirmed stable.

The system targets **IEC 61508 SIL 3 / ISO 26262 ASIL D / MIL-STD-882E Category I**. It does not hold a formal certification — specific gaps are documented in §7. All six critical findings (CR-1 through CR-6) and key major/minor findings from the prior SIL-2 audit have been remediated in this version.

**Primary decision boundary:** `g_platform_stable` is set TRUE only when all of the following hold simultaneously:

- No active fault flags (`effective_faults() == 0`)
- Fault shadow register consistent (`fault_shadow_ok()`)
- CRC-16 over 36 bytes of critical state passes (`safety_crc_ok()`)
- Window buffer fully populated
- Gyro spike count ≤ `motion_samples_max`
- Accel spike count ≤ `shock_samples_max`
- Tilt spread (centroid radius) ≤ `spread_max_deg`
- Instant-hold timer expired
- ISR frozen-watchdog flag clear
- Hardware FSM gate passes (both channels)
- Absolute tilt anchor within `anchor_max_deg`

**Fail-safe direction:** any fault, communication failure, hardware error, or software integrity violation drives the output LOW. The system is fail-safe by default.

```
┌──────────────────────────────────────────────────────────────────────┐
│                    Platform Stability Monitor v2                      │
│                                                                       │
│  ┌────────────────┐  SPI1 4 MHz  ┌──────────────────────────────────┐│
│  │  ASM330LHB-A   │◄────────────►│          Teensy 4.1               ││
│  │  CS=10  INT=8  │  CS_A=10     │     (iMXRT1062 Cortex-M7)         ││
│  └────────────────┘  MISO=1      │                                   ││
│                       MOSI=26    │  ┌──────────┐ ┌─────────────────┐││
│  ┌────────────────┐  SCK=27      │  │timerISR  │ │  RTWDOG3 1.5s   │││
│  │  ASM330LHB-B   │◄────────────►│  │ 833 Hz   │ │  LPO 1 kHz clk  │││
│  │  CS=9   INT=7  │  CS_B=9      │  └──────────┘ └─────────────────┘││
│  └────────────────┘              │                                   ││
│                                  │  ┌───────────────────────────────┤│
│  ┌────────────────┐              │  │ Mahony A + Mahony B (833 Hz)  ││
│  │  SAFE_OUT      │◄─────────────│  │ Stability FSM + Fault Monitor ││
│  │  PIN 2         │              │  └───────────────────────────────┤│
│  └──────┬─────────┘              │                                   ││
│         │ 100 Ω                  │  ┌───────────────────────────────┤│
│  ┌──────▼─────────┐              │  │ HTTP/1.1 dashboard+API        ││
│  │  SAFE_MON      │─────────────►│  │ QNEthernet 192.168.168.71     ││
│  │  PIN 3 (pulldown)│  readback  │  └───────────────────────────────┘│
│  └────────────────┘              └──────────────────────────────────-─┘
│                                                                       │
└──────────────────────────────────────────────────────────────────────┘
```

---

## 2. Hardware Components

### 2.1 Teensy 4.1 — MCU (iMXRT1062)

**What it is:** NXP iMXRT1062 Cortex-M7 at 600 MHz. 1 MB SRAM (DTCM + OCRAM), 8 MB flash (QSPI), hardware FPU (double-precision), DWT cycle counter.

**Why used:** The 600 MHz clock provides ample headroom for two Mahony filter instances, floating-point stability math, and SPI at 833 Hz without timing margin risk. Teensyduino provides a mature HAL. Built-in ENET MAC eliminates an external Ethernet chip.

**Internal features used:**

| Feature | Usage |
|---|---|
| `IntervalTimer` | 833 Hz ISR via FlexTimer/PIT peripheral |
| `RTWDOG3` (WDOG3) | Watchdog — 1500 ms timeout, LPO 1 kHz clock |
| `SRC_SRSR` | System Reset Controller — reset-cause register, W1C |
| `ARM_DWT_CYCCNT` | DWT cycle counter — ISR period measurement |
| `EEPROM` (emulated) | Persistent parameter storage (FlexNVM) |
| `SPI1` | SCK=27, MOSI=26, MISO=1, 4 MHz, SPI_MODE3 |
| ENET MAC | Built-in Ethernet controller (QNEthernet driver) |
| GPIO | `SAFE_OUT_PIN`=2, `SAFE_MON_PIN_HW`=3 (INPUT_PULLDOWN), LED=13, LIGHT=4 |
| GPIO | `IMU_A_CS`=10, `IMU_B_CS`=9, `IMU_A_INT`=8, `IMU_B_INT`=7 |

**Stack:** Default Teensyduino stack is 8 KB. PSM monitors usage via a watermark; sets `FAULT_INTEGRITY` if > 6 KB consumed.

**Reset-cause logging:** `SRC_SRSR` is read and cleared (W1C) at startup. The decoded cause (POR, WDOG3, WDOG1, user reset) is printed to Serial.

---

### 2.2 ASM330LHB IMU (× 2)

**What it is:** STMicroelectronics ASM330LHB — automotive-grade 6-axis IMU (3-axis accel, 3-axis gyro) with embedded finite-state-machine processor. AEC-Q100 Grade 0.

**Why used:** Automotive-qualified MEMS with an on-chip embedded FSM provides a hardware-level motion detection gate independent of the host MCU software. Dual independent channels allow cross-validation and divergence detection.

**Configuration (both channels):**

| Register | Value | Meaning |
|---|---|---|
| `CTRL1_XL` | `0x70` | Accel: 833 Hz ODR, ±2 g FS |
| `CTRL2_G` | `0x74` | Gyro: 833 Hz ODR, ±500 dps FS |
| `CTRL3_C` | `0x44` | BDU=1 (block data update), IF_INC=1 |
| `CTRL4_C` | `0x08` | DRDY_MASK=1 |
| `INT1_CTRL` | `0x03` | XLDA + GDA interrupt on INT1 |

**Sensitivities:**  
- Accel: 61 µg/LSB at ±2 g → `ASM_ACCEL_SENS_G = 0.000061`  
- Gyro: 17.5 mdps/LSB at ±500 dps → `ASM_GYRO_SENS_DPS = 0.01750`

**IMU-B axis remap:** IMU-B is mounted on the reverse side with 90° rotation relative to IMU-A. Before all comparison and fusion logic, IMU-B raw vectors are remapped: `B.x = B.y, B.y = B.x, B.z = −B.z` (derived from live gravity vector alignment).

**Embedded FSM:** Each sensor's on-chip FSM (FSM1) is configured as a motion-detection gate. `asm_fsm_read_stable()` polls `FSM_STATUS_A` bit 0. Only when both channels report FSM-stable for `FSM_STABLE_NEED` = 3 consecutive evaluations does the `hw_gate` pass. This provides an independent hardware check of the software stability decision.

---

### 2.3 SPI1 Bus

Both IMUs share the same SPI1 bus (MISO=1, MOSI=26, SCK=27) with separate chip-select lines (CS_A=10, CS_B=9). This means the physical MISO signal is common to both channels — a stuck-MISO fault could potentially affect both simultaneously.

**Mitigations for shared-bus risk:**
1. Separate CS lines: only one sensor is active per transaction.
2. Config CRC-8 shadow: per-channel ODR/range register integrity checked every 200 ms (§4.4).
3. CRC-8 burst frame freeze: per-channel 12-byte burst CRC checked every sample (§4.5).
4. Common-mode lock detection: pre-remap channel identity check catches stuck-MISO (§4.11).

---

### 2.4 SAFE_OUT / SAFE_MON Circuit

**What it is:** `SAFE_OUT_PIN` (pin 2, OUTPUT) asserts HIGH when `g_platform_stable == true`. `SAFE_MON_PIN_HW` (pin 3, `INPUT_PULLDOWN`) is wired to the same net via ≤ 100 Ω and independently reads back the actual net voltage.

**Why used:** Detects output driver faults (stuck-at-HIGH, stuck-at-LOW) that pure software cannot detect. Required for SIL 3 output integrity.

**Startup dual-state check (CR-4, added Rev 2.0):**  
During `safety_init()`, before any other logic, the system drives SAFE_OUT_PIN LOW → reads SAFE_MON_PIN_HW → drives HIGH → reads → drives LOW again. Both states must agree. Failure sets `FAULT_OUTPUT` before Serial is even opened. This catches shared-net faults where both driver and readback are stuck at the same voltage.

**Runtime readback policy:**

```
set_safe_output(stable):
  if g_output_inhibit: drive LOW, return

  drive SAFE_OUT_PIN = stable
  delayMicroseconds(10)
  read SAFE_MON_PIN_HW → mon_high

  if (stable == true) && (mon_high == false):
      fault_set(FAULT_OUTPUT)      ← net LOW when HIGH commanded
  if (stable == false) && (mon_high == true):
      fault_set(FAULT_INTEGRITY)   ← net HIGH when LOW commanded — CRITICAL
  if (stable == true) && (mon_high == true):
      fault_clr(FAULT_OUTPUT)      ← confirm path functional

  NOTE: FAULT_OUTPUT only clears via a successful TRUE readback pass.
        FALSE readback does not clear it — prevents oscillation masking.
```

**Wiring requirement:** ≤ 100 Ω series only (ESD). Do not use 10 kΩ — the internal PULLDOWN (~100 kΩ) forms a voltage divider that may not cross the logic-HIGH threshold.

---

### 2.5 Ethernet Interface (QNEthernet)

Teensy 4.1's built-in ENET MAC + external PHY, driven by QNEthernet (lwIP-based). Static IP: `192.168.168.71/24`, GW `192.168.168.1`.

**Safety note:** Ethernet PHY link-up can take several seconds. The WDT is temporarily widened to 8000 ms during `app_init()` and restored to 1500 ms afterward. The ISR is not started until after Ethernet init completes, preventing false `FAULT_FROZEN` during link-up.

---

## 3. Software Architecture

### 3.1 Module Layout

21-file modular architecture split into two strict layers. The independence rule is enforced: `safety/` headers must never `#include` any `app/` header.

```
safety/
  safety_defs.h       — all #defines, fault codes, SafetyThresholds, SafetyStatus structs
  safety_types.h      — Tilt, Mahony, StabFSM
  safety_monitor.h/cpp — WDT (RTWDOG3), fault shadow pair, CRC-16 CCITT, RAM march,
                         stack, set_safe_output, fatal
  safety_imu.h/cpp    — SPI driver, self-test, config CRC-8 shadow, burst CRC-8 freeze,
                         hardware FSM, sensor reads
  safety_ahrs.h/cpp   — Mahony complementary filter (833 Hz, yaw excluded)
  safety_stability.h/cpp — window buffer, spread, FSM, evaluate_stability, g_safety_status
  safety_isr.h/cpp    — 833 Hz timerISR, imu_recover, safety_tick, safety_init

app/
  app_params.h/cpp    — Params struct, EEPROM load/save, params_valid, app_publish_thresholds
  app_http.h/cpp      — Ethernet init, web server, /api/status|params|js, /config,
                        HTML dashboard
  app_light.h/cpp     — LED output toggle (pin 4), /light/toggle endpoint
```

**One-way data flow:**
- `SafetyThresholds g_safety_thresholds` — app writes after `params_valid()`, safety reads
- `SafetyStatus g_safety_status` — safety writes each eval cycle, app reads (const volatile)

### 3.2 Setup and Loop Execution Order

```
setup():
  1. params_load() + app_publish_thresholds()   ← thresholds valid before tick 1
  2. safety_init()                               ← GPIO, startup readback check, SPI, Serial,
                                                    RAM march, IMU init, self-test, FSM,
                                                    Mahony, WDT(1500ms)
  3. Serial: print params outcome
  4. WDT widened to 8000 ms
  5. app_init() + light_init()                  ← Ethernet, web server
  6. WDT restored to 1500 ms
  7. safety_start_isr()                         ← start 833 Hz timer (deferred to prevent
                                                    false FAULT_FROZEN during Ethernet init)

loop() [~833 Hz or higher]:
  safety_tick()    ← WDT feed, AHRS, stability eval, output control, status print
  app_tick()       ← web server accept/handle, Ethernet.loop()
```

### 3.3 Timer ISR (`timerISR`, 833 Hz)

Fires at exactly 833 Hz via `IntervalTimer`. Four responsibilities:

**1. ISR-period timing check:**  
DWT `ARM_DWT_CYCCNT` measures the interval between consecutive ISR firings. Expected: `F_CPU / 833` = 720,288 cycles. Tolerance: ±5% = ±36,014 cycles. Jitter beyond this sets `s_timing_fault_isr` (read-cleared atomically by `loop()` with `noInterrupts()`).

**2. Sample tick:**  
Increments `s_sample_due` (capped at 255). `loop()` atomically reads and clears it. `s_sample_due > 1` means missed samples; `s_missed_samples` is incremented accordingly.

**3. Stability decimation tick:**  
`s_stability_div_cnt` counts to `STABILITY_DIV` = 8 before incrementing `s_stability_due`, decimating the 833 Hz sample stream to ~104 Hz for buffer push and spread computation.

**4. Frozen-loop watchdog (CR-5):**  
`s_loop_alive_token` is written by `loop()` each iteration. If unchanged for `FROZEN_TICKS_MAX` = 26 consecutive ISR ticks (~31 ms at 833 Hz), the ISR directly drives both `SAFE_OUT_PIN` and LED LOW via `digitalWriteFast()`, and sets `s_forced_unsafe_isr = true` (sticky — cleared only by `imu_recover()`). This pre-empts the 1500 ms WDT for transient freezes.

### 3.4 Stability Algorithm

**833 Hz sample block (`s_sample_due > 0`):**
1. Read sensors A and B via `asm_read_sensors()` (SPI burst, CRC-8 freeze check, range plausibility).
2. Common-mode check: compare pre-remap float vectors from A and B. If identical for ≥ 3 consecutive reads → `FAULT_INTEGRITY`.
3. Axis remap IMU-B.
4. Update stale counters for A and B; set `FAULT_IMU_A/B_COMM` after `stale_fault_ticks` consecutive failures.
5. `mahony_update()` on both instances at 833 Hz.
6. Extract roll/pitch from each Mahony quaternion.
7. Compute ω = `|gyro|` and accel magnitude; apply `isnan` + range checks.
8. Dual-diverge check: if `|roll_A − roll_B| > dual_diverge_deg` OR `|pitch_A − pitch_B| > dual_diverge_deg` → `FAULT_DUAL_DIVERGE`.
9. Update `s_instant_hold_until` if `ω > omega_instant_dps`.

**~104 Hz stability tick (`s_stability_due > 0`):**
- If no channel faults: fuse A+B tilt (average unless B degraded) → `push_sample()` → writes buffer slot, updates spike counts, refreshes CRC-16.
- If channel fault active: force output false, skip push.

**200 ms evaluation tick (every `eval_period_ms`):**
1. WHO_AM_I health check on both channels (3-failure debounce).
2. Config CRC-8 shadow check on both channels (3-failure debounce → `FAULT_IMU_A/B_COMM`).
3. `imu_recover()` if any recover-triggering fault active (exponential back-off).
4. Stack watermark update.
5. `compute_window_tilt_spread()` — centroid mean of window, max radius from centroid.
6. `diverse_stable()` — window mean accel mag vs 1 g ± `diverse_mean_tol_g` → `FAULT_DIVERSE`.
7. Dual-diverge clean streak → `fault_clr(FAULT_DUAL_DIVERGE)` after `clean_streak_needed` passes.
8. Hardware FSM gate: read `FSM_STATUS_A` from each channel. `hw_gate = (s_fsm_stable_cnt >= FSM_STABLE_NEED)` = 3 consecutive both-channel FSM-stable readings.
9. `evaluate_stability(spread)` — all-pass gate (shadow, CRC, faults, hold, fill, spikes, spread).
10. Stability FSM (`SFST_INIT → SFST_FILLING → SFST_STABLE / SFST_UNSTABLE / SFST_FAULT`).
11. Anchor check: on first stable reading, set `s_stable_anchor`. On subsequent stable readings, if `tilt_distance_deg(current, anchor) > anchor_max_deg` → force unstable, invalidate anchor.
12. `set_safe_output(g_platform_stable)` — drive + readback.
13. Status line print (5 s cadence), update `g_safety_status`.

### 3.5 IMU Recovery

`imu_recover(b_only)` is called from the 200 ms eval loop when `FAULT_IMU_A/B_COMM`, `FAULT_FROZEN`, or `FAULT_SELFTEST` is active. Exponential back-off: `min(3000 × 2^n, 30000)` ms.

```
imu_recover(b_only):
  if peak_omega still high: decay × 0.9 and abort (avoid recovery during motion)
  stop IntervalTimer
  reset sample/stability counters (noInterrupts)
  set_safe_output(false)
  asm_frozen_burst_reset for target channel(s)
  asm_init() for target channel(s)   ← includes SW reset + WHO_AM_I check
  asm_self_test() for target channel(s)
  wait for fresh sensor data (3 s timeout)
  if b_only: seed Mahony-B from Mahony-A quaternion (prevents FAULT_DUAL_DIVERGE transient)
  else:       mahony_init(A + B)
  asm_fsm_configure() for target channel(s)
  fault_reset_all()
  reset_stability_state()
  s_forced_unsafe_isr = false   ← ONLY place this is cleared
  restart IntervalTimer
```

**IMU-B degradation:** After `IMU_B_DEGRADE_AFTER` = 4 consecutive B recovery failures, `s_imu_b_degraded` is set. The system falls back to IMU-A single-channel mode. `FAULT_IMU_B_COMM` and `FAULT_DUAL_DIVERGE` are suppressed in `effective_faults()` while degraded. Degradation is logged to Serial and reflected in `g_safety_status.imu_b_degraded`.

### 3.6 EEPROM Parameters

`Params` struct stored at EEPROM address 0. Magic `0xBE4F0002`, version `3` (invalidates all BNO085-era and prior ASM330LHB v2 EEPROM images).

On load, `params_valid()` checks: magic + version match, `isfinite()` on all float fields (rejects NaN/Inf before range comparisons — NaN silently passes `<`/`>` checks), and range bounds for every field.

If validation fails, compile-time defaults are used. `app_publish_thresholds()` copies validated `g_params` into `g_safety_thresholds` (atomic, under `noInterrupts()`) and calls `refresh_safety_crc()` to update the CRC-16 with the new threshold values.

Parameters are updated via `POST /config` with auth token. Update path: parse body → `params_valid()` → apply to `g_params` → `params_save()` (EEPROM write under `noInterrupts()`, WDT fed around write) → `app_publish_thresholds()`.

### 3.7 HTTP API

Hosted on port 80. Single-client slot, 500 ms idle timeout.

| Endpoint | Method | Purpose |
|---|---|---|
| `/` | GET | HTML dashboard (inline CSS + JS polling) |
| `/api/status` | GET | JSON: stable, fault_mask, why, fsm, fill%, angles A+B, spread, spikes, imu_b_degraded |
| `/api/params` | GET | JSON: current `g_params` |
| `/api/js` | GET | Dashboard polling JS (auth token injected via `data-auth` HTML attribute, not in JS) |
| `/config` | POST | Update + save parameters (auth required, `params_valid()` gate) |
| `/test/toggle-output` | POST | Toggle `g_output_inhibit` for bench testing (auth required) |
| `/light/toggle` | POST | Toggle app_light LED on pin 4 (auth required) |

Auth: shared `CONFIG_AUTH_TOKEN` in POST body (URL-encoded). Compared via `ct_token_eq()` (constant-time, prevents timing side-channel). **Default token `"psm-change-me-v1"` must be changed before deployment.** Token is injected into the HTML `data-auth` attribute server-side and never placed in JavaScript source or API responses.

`g_output_inhibit` suppresses physical output for bench testing. It is `volatile`, not EEPROM-backed, and clears on power cycle or WDT reset.

---

## 4. Safety Mechanisms

### 4.1 Watchdog Timer (RTWDOG3)

**Configuration:**
```
Timeout:  1500 ms (8000 ms temporarily during Ethernet init)
Clock:    LPO 1 kHz (CLK=01)
CMD32EN:  1  (32-bit refresh word required)
UPDATE:   1  (reconfiguration allowed)
Unlock:   two consecutive 32-bit writes (0xD928C520, 0xB480A602)
Refresh:  single 32-bit write (0xB480A602)
```

`wdt_feed()` is called every `timerISR()` tick (833 Hz) -- independent of `loop()`. It is also called in `safety_init()`/`imu_recover()` around each blocking delay and in `params_save()` around the EEPROM write, for the windows where the ISR is stopped.

**Decoupling app_tick() from the WDT:** Feeding from the ISR (not from `safety_tick()`/`loop()`) means a hang anywhere in `app_tick()` (e.g. a stuck Ethernet/HTTP poll) cannot starve RTWDOG3 and force a full MCU reset. Only a true MCU lockup -- where the 833 Hz ISR itself stops firing (hard fault, IRQs masked) -- still hits the 1500 ms WDT reset.

**Interaction with ISR frozen-watchdog:** The ISR fires unsafe output at ~31 ms of loop freeze regardless of WDT state. For a `loop()`/`app_tick()` hang that leaves interrupts enabled, this is now the *only* fail-safe path -- the WDT keeps being fed by the ISR and will not reset the MCU, so `SAFE_OUT_PIN` stays LOW (and `FAULT_FROZEN` stays latched) until `app_tick()` returns and `safety_tick()`'s fast-recovery path clears it.

**Reset-cause:** WDOG3 resets set bit 7 of `SRC_SRSR`. Logged to Serial at next startup.

**Limitations:** Single internal watchdog, no window mode. A tight busy-loop in the ISR itself (interrupts disabled) would not be caught by the WDT and would also defeat CR-5.

---

### 4.2 Fault Shadow Register

**What:** `s_fault_mask` (uint16_t) stores active fault bits. `s_fault_mask_inv` stores its bitwise complement. Every `fault_set()` / `fault_clr()` updates both atomically (under `noInterrupts()`).

**Check:** `fault_shadow_ok()` returns `(s_fault_mask ^ s_fault_mask_inv) == 0xFFFF`. Any single-bit flip in either word causes a mismatch → `FAULT_INTEGRITY` → blocks stable output.

**ISR-safe variants:** `fault_set_from_isr()` / `fault_clr_from_isr()` — no `noInterrupts()` guard (ISR is already non-preemptible). Used only in the timer ISR's frozen-watchdog path.

---

### 4.3 CRC-16 CCITT-FALSE (36 bytes)

**Polynomial:** `0x1021`, init `0xFFFF`. Hamming Distance 4 — detects all 1, 2, and 3-bit errors in covered data.

**Covered data (36 bytes total):**

```
s_spike_gyro_count    4 B  (little-endian)
s_spike_accel_count   4 B
s_buf_count           4 B
s_buf_head            4 B
s_fault_mask          2 B
s_fault_mask_inv      2 B
g_safety_thresholds.omega_stable_dps   4 B
g_safety_thresholds.spike_accel_g      4 B
g_safety_thresholds.spread_max_deg     4 B   ← added Rev 2.0
g_safety_thresholds.anchor_max_deg     4 B   ← added Rev 2.0
```

`spread_max_deg` and `anchor_max_deg` were previously uncovered; a RAM fault corrupting either threshold could produce a false-stable decision without triggering a CRC mismatch. Both are now integrity-protected.

**Refresh:** `refresh_safety_crc()` is called after every `fault_set/clr/reset`, every `push_sample()`, every `app_publish_thresholds()`, and at the start of every `evaluate_stability()` call (which re-runs `safety_crc_ok()` independently).

**Coverage gaps:** `s_tilt_buf` and `s_accel_mag_buf` (up to 520 × 3 floats ≈ 6 KB) are not CRC-covered. Corruption of these buffers could produce a false spread result without triggering a mismatch.

---

### 4.4 CRC-8 Config Register Shadow

**What:** After every successful `asm_init()`, `cfg_snapshot_crc()` reads `{CTRL1_XL, CTRL2_G, CTRL3_C, CTRL4_C}` from the sensor and stores a CRC-8 (poly `0x07`, init `0xFF`) per channel in `s_cfg_crc[2]`.

**Check:** Every 200 ms eval cycle, `asm_verify_config_crc()` re-reads the four registers and compares. Three consecutive failures set `FAULT_IMU_A/B_COMM`, triggering `imu_recover()`. This detects SPI-induced corruption of ODR or range settings that would silently change sensor output scale without triggering a WHO_AM_I failure.

---

### 4.5 CRC-8 Burst Frame Freeze Detection

**What:** `asm_read_sensors()` computes CRC-8 (poly `0x07`, init `0xFF`) over all 12 raw burst bytes `buf[0..11]` on every read. The per-channel CRC is stored in `s_frame_crc[2]`.

**Logic:** If the current CRC equals the stored CRC from the previous read, `s_frame_crc_frozen_cnt[idx]` is incremented. If the frozen counter reaches `FROZEN_BURST_TICKS` = 5 consecutive identical-CRC reads, `asm_read_sensors()` returns `false` (treated as stale). The counter resets on any CRC change.

**Why this replaces the prior 3-axis gyro comparison:** The old check only compared `(gx_r, gy_r, gz_r)` — 6 of 12 bytes. The CRC-8 covers all 12 bytes (accel + gyro). Initialised to `0xAA` (not `0x00` or `0xFF`) so the first real read always differs from the init value, preventing a false frozen-flag at startup.

`asm_frozen_burst_reset()` resets both `s_frame_crc[idx]` and `s_frame_crc_frozen_cnt[idx]`; called from `imu_recover()` before re-init.

---

### 4.6 RAM March Test

**What:** March C− algorithm over a dedicated 128-byte `march_buf` (static volatile), run once at startup before any other logic.

**Sequence:**
```
Phase 0 (↑): write 0x00 to all cells
Phase 1 (↑): verify 0x00, write 0xFF
Phase 2 (↑): verify 0xFF, write 0x00
Phase 3 (↓): verify 0x00, write 0xFF
Phase 4 (↓): verify 0xFF, write 0x00
Phase 5 (↑): verify 0x00
```

Detects: stuck-at-0, stuck-at-1, transition faults, address-decoder coupling faults.

**On failure:** LED blinks at ~5 Hz; system halts permanently.

**Coverage gap:** Only 128 bytes tested. Safety-critical DTCM globals (`s_fault_mask`, `s_tilt_buf`, `g_params`, etc.) are not march-tested.

---

### 4.7 ISR Frozen-Loop Watchdog (CR-5)

`s_loop_alive_token` (uint8_t) is incremented by `loop()` each iteration. At every 833 Hz ISR tick, if the token is unchanged from the previous tick, `s_isr_frozen_cnt` increments. At `FROZEN_TICKS_MAX` = 26 consecutive frozen ticks (~31 ms at 833 Hz):

- `digitalWriteFast(SAFE_OUT_PIN, LOW)` — drives output unsafe directly from ISR
- `s_forced_unsafe_isr = true` — sticky; checked at top of every `loop()` iteration

**Recovery:** Only `imu_recover()` clears `s_forced_unsafe_isr`, and only after a complete successful re-init sequence. Prevents a post-freeze resume from automatically re-asserting stable output.

**Fast-path for FAULT_FROZEN-only:** If `s_forced_unsafe_isr` is set but no IMU hardware fault exists, execution in `loop()` proves the loop is alive — the fast path clears `s_forced_unsafe_isr` and `FAULT_FROZEN` directly without `imu_recover()` overhead.

**Known app-layer trigger (fixed):** `app_tick()` previously called `EthernetClient::stop()` after every HTTP request (every response sends `Connection: close`). QNEthernet's `stop()` busy-waits up to `connTimeout_` (default 1000 ms) inside `yield()`/`Ethernet.loop()` for the peer's FIN-ACK before returning. Whenever a client (browser GUI poll or curl) was slow to ACK the FIN, `loop()` blocked long enough to trip CR-5 — observed even with only the dashboard's idle 300 ms `/api/status` poll running. Fixed by using the non-blocking `EthernetClient::close()` instead, which sends the FIN and returns immediately; lwIP completes the close handshake in the background on subsequent `Ethernet.loop()` calls.

---

### 4.8 ISR Timing Monitor

**What:** DWT `ARM_DWT_CYCCNT` is sampled at each ISR entry. Expected period: `F_CPU / AHRS_HZ` = `600,000,000 / 833` = 720,288 cycles. Tolerance: ±5%.

**On violation:** `s_timing_fault_isr = true`. `loop()` reads it atomically and sets `FAULT_TIMING`. Clears after `clean_streak_needed` = 5 consecutive clean ISR ticks.

**Purpose:** Detects interrupt priority inversion, timer mis-configuration, or CPU frequency changes that would cause the 833 Hz sample stream to be incorrectly timed, skewing the Mahony filter.

---

### 4.9 Output Readback + Startup Dual-State Check (CR-4)

**Startup dual-state check:** During `safety_init()`, before `ram_march_test()`, the system explicitly exercises both output states (LOW → HIGH → LOW) and verifies `SAFE_MON_PIN_HW` follows each transition (50 µs settle each). Failure sets `FAULT_OUTPUT`. This catches shared-net faults — cases where the driver net and the monitoring pin are both shorted to the same supply rail, rendering the runtime readback unable to distinguish intentional state from a fault.

**Runtime readback:** Described in §2.4. Two distinct fault bits: `FAULT_OUTPUT` ("cannot assert stable") vs `FAULT_INTEGRITY` ("cannot deassert unsafe" — critical).

**Residual:** The monitoring path cannot detect a stuck-at-stable net fault in the one scenario where both driver and readback share the same stuck-at-HIGH voltage AND the system is actively trying to assert stable. In that scenario, however, the output is correctly HIGH (safe), so no dangerous failure results.

---

### 4.10 Dual-Channel AHRS + Divergence Detection

Two independent Mahony complementary filter instances (`s_imu_a`, `s_imu_b`) run at 833 Hz. Each produces its own roll/pitch estimate. Yaw is excluded from all stability calculations (gyro-integrated yaw drifts ~0.1 dps over a 1–5 s window and would produce false instability signals on a level but slowly rotating platform).

**Mahony parameters:** Kp = 2.0, Ki = 0.005. Ki is normalised per-sample as `Ki / AHRS_HZ` to avoid integrator accumulation varying with sample rate.

**Divergence check:** Each 833 Hz sample, if both channels are fresh:
```
if |roll_A − roll_B| > dual_diverge_deg OR |pitch_A − pitch_B| > dual_diverge_deg:
    fault_set(FAULT_DUAL_DIVERGE)
```
Default `dual_diverge_deg` = 5.0°. Clears after `clean_streak_needed` consecutive evaluations within threshold. Suppressed in `effective_faults()` when IMU-B is degraded.

**Common-mode limitation:** Both IMUs share the SPI1 MISO line. A common-mode MISO fault can corrupt both channels simultaneously in a way that preserves apparent agreement. The common-mode lock check (§4.11) partially mitigates this.

---

### 4.11 Common-Mode Channel Lock Detection

**What:** After reading both sensors in the 833 Hz sample block, but before axis remapping IMU-B, the six pre-remap float values from A and B are compared exactly:

```
if ax_a == ax_b && ay_a == ay_b && az_a == az_b &&
   gx_a == gx_b && gy_a == gy_b && gz_a == gz_b:
     s_cm_cnt++
     if s_cm_cnt >= 3: fault_set(FAULT_INTEGRITY)
else:
     s_cm_cnt = 0
```

**Why pre-remap floats:** Float equality ↔ raw int16 equality ↔ identical 12-byte SPI frames. Checking before remap means we compare the raw SPI data directly.

**Threat addressed:** Stuck-MISO (both CS transactions receive the same signal) or accidental hardware wiring of both CS pins to the same physical device. Two truly independent 16-bit 6-axis sensors under thermal noise will essentially never produce bit-for-bit identical readings in three consecutive samples.

**Limitation:** A common-mode fault that consistently produces distinct-but-wrong values from both channels (e.g., a bus corruption that maps consistently to two different wrong values) would not be caught by this check but would likely be caught by the divergence check or range faults.

---

### 4.12 Hardware FSM Gate

Each ASM330LHB runs an on-chip FSM (FSM1) configured as a motion-stability classifier. The FSM output is polled every 200 ms eval cycle via `asm_fsm_read_stable()` (reads `FSM_STATUS_A` bit 0). Both channels must report stable for `FSM_STABLE_NEED` = 3 consecutive cycles (hw_gate). The FSM gate is a hardware-independent confirmation that the Mahony software stability decision is plausible.

**Limitation:** Both FSM instances are configured identically from the same SPI transactions. A common-mode SPI configuration fault could corrupt both FSMs symmetrically. The config CRC-8 shadow (§4.4) detects configuration corruption. In degraded mode (IMU-B absent), `hw_b` is forced true so the gate does not deadlock.

---

### 4.13 Sensor Range Faults

| Fault | Condition | Cleared after |
|---|---|---|
| `FAULT_IMU_A_COMM` | ≥ `stale_fault_ticks` consecutive failed reads OR 3× WHO_AM_I fail OR 3× config CRC fail | `imu_recover()` only |
| `FAULT_IMU_B_COMM` | Same as above for IMU-B | `imu_recover()` only |
| `FAULT_GYRO_RANGE` | `isnan(ω)` or `ω > gyro_range_fault_dps` (248 dps) | `clean_streak_needed` = 5 clean samples |
| `FAULT_ACCEL_RANGE` | `isnan(|a|)` or `|a| < 0.05g` or `|a| > 4.4g` | `clean_streak_needed` = 5 clean samples |
| `FAULT_DIVERSE` | Window mean accel mag not within `diverse_mean_tol_g` of 1 g | `clean_streak_needed` = 5 clean evals |
| `FAULT_DUAL_DIVERGE` | `|roll_A − roll_B|` or `|pitch_A − pitch_B|` > `dual_diverge_deg` | `clean_streak_needed` = 5 clean evals |

`stale_fault_ticks` default = 15 at 833 Hz = ~18 ms of consecutive failed reads. The output is already forced FALSE on the first stale read; the stale counter only gates `imu_recover()` activation.

---

### 4.14 Self-Test

`asm_self_test()` is run on both channels during `safety_init()` and after every `asm_init()` in `imu_recover()`. The procedure follows the ASM330LHB application note:

1. Enable gyro self-test mode (`CTRL5_C[4:3] = 01`) → collect 5 samples → compare against baseline
2. Enable positive accel self-test (`CTRL5_C[1:0] = 01`) → 5 samples
3. Enable negative accel self-test (`CTRL5_C[1:0] = 10`) → 5 samples → compute symmetric average

Gyro acceptance band: 2143–10,000 LSB/5 per axis. Accel acceptance band: 328–13,934 LSB/5 per axis. Failure sets `FAULT_SELFTEST`.

---

### 4.15 Stack Watermark

SP is captured in `setup()` as `s_stack_base`. Each 200 ms eval cycle, `stack_watermark_update()` captures current SP and updates `s_stack_min_sp`. If `s_stack_base − s_stack_min_sp > STACK_WARN_BYTES` (6144), `FAULT_INTEGRITY` is set. Teensy 4.1 default stack is 8 KB.

---

### 4.16 Hang Checkpoint

`s_hang_cp` is set before each potentially blocking call and cleared after:

| Value | Location |
|---|---|
| 0 | OK / idle |
| 1 | Inside `asm_read_sensors()` (SPI transactions) |
| 3 | Inside status-line `Serial.println()` |

Captured as `hang_cp_snap` before the status print (MI-7: prevents the print from overwriting the checkpoint with its own marker before it is logged). Visible in the Serial status line as `cp=N`.

---

### 4.17 EEPROM Config Authentication

`POST /config` and `POST /test/toggle-output` require `auth=<token>` in the URL-encoded body. Auth uses `ct_token_eq()` — a constant-time comparison that processes every byte of both strings regardless of match outcome, preventing timing side-channel attacks. Tokens are injected into the HTML `data-auth` attribute server-side; they never appear in `/api/js` response body. Invalid parameters are rejected by `params_valid()` before any runtime state is touched.

---

## 5. Data Flow

### 5.1 833 Hz Sample Path

```
timerISR (833 Hz)
  ├─ DWT delta → s_timing_fault_isr
  ├─ s_sample_due++
  ├─ s_stability_div_cnt → s_stability_due every 8 ticks
  └─ frozen-loop check → s_forced_unsafe_isr

loop() detects s_sample_due > 0:
  ├─ asm_read_sensors(A) + asm_read_sensors(B)
  │     ├─ STATUS_REG check (0xFF → false)
  │     ├─ SPI burst 12 bytes
  │     └─ CRC-8 frame freeze check (5 consecutive identical CRCs → false)
  │
  ├─ common-mode lock check (pre-remap)
  │     └─ 3× identical → FAULT_INTEGRITY
  │
  ├─ axis remap IMU-B
  │
  ├─ stale counters → FAULT_IMU_A/B_COMM after stale_fault_ticks failures
  │
  ├─ mahony_update(A) + mahony_update(B)
  ├─ mahony_to_tilt(A) + mahony_to_tilt(B)
  │
  ├─ range checks → FAULT_GYRO_RANGE / FAULT_ACCEL_RANGE
  ├─ dual-diverge check → FAULT_DUAL_DIVERGE
  └─ instant-hold arm if ω > omega_instant_dps
```

### 5.2 ~104 Hz Buffer Push Path

```
loop() detects s_stability_due > 0:
  if effective_faults() == 0:
    ├─ fuse tilt: average(A, B) [or A-only if IMU-B degraded]
    ├─ push_sample(tilt, ω, |a|)
    │     ├─ write circular buffer slot
    │     ├─ update s_spike_gyro_count + s_spike_accel_count
    │     └─ refresh_safety_crc()
  else:
    ├─ refresh_safety_crc()
    └─ g_platform_stable = false, set_safe_output(false)
```

### 5.3 200 ms Evaluation Path

```
loop() @ eval_period_ms:
  ├─ WHO_AM_I check A + B (3-fail debounce)
  ├─ config CRC-8 check A + B (3-fail debounce → FAULT_IMU_A/B_COMM)
  ├─ imu_recover() if recov_mask active [exponential back-off]
  ├─ stack_watermark_update()
  ├─ compute_window_tilt_spread() → centroid max-radius
  ├─ diverse_stable() → FAULT_DIVERSE
  ├─ dual_diverge_clean streak → fault_clr(FAULT_DUAL_DIVERGE)
  ├─ hw_gate: FSM A + FSM B stable ≥ FSM_STABLE_NEED
  ├─ evaluate_stability(spread):
  │     ├─ fault_shadow_ok() → FAULT_INTEGRITY
  │     ├─ safety_crc_ok()   → FAULT_INTEGRITY
  │     ├─ effective_faults() == 0
  │     ├─ instant-hold expired
  │     ├─ buf_count ≥ window_samples
  │     ├─ spike_gyro_count ≤ motion_samples_max
  │     ├─ spike_accel_count ≤ shock_samples_max
  │     └─ spread ≤ spread_max_deg
  ├─ stability FSM (INIT→FILLING→STABLE/UNSTABLE/FAULT)
  ├─ anchor check (tilt_distance_deg > anchor_max_deg → unstable + invalidate)
  └─ set_safe_output(g_platform_stable)
        ├─ drive SAFE_OUT_PIN
        └─ readback SAFE_MON_PIN_HW → FAULT_OUTPUT / FAULT_INTEGRITY
```

### 5.4 Fault Handling Path

```
Fault detected
  │
  ▼
fault_set(FAULT_xxx) ──────────────────────────────────────────────┐
  │                                                                 │
  ▼                                                                 │
effective_faults() != 0 → evaluate_stability() returns false ◄────┘
  │
  ▼
g_platform_stable = false → set_safe_output(false) → SAFE_OUT_PIN = LOW

If FAULT_IMU_A/B_COMM, FAULT_FROZEN, or FAULT_SELFTEST:
  └─ imu_recover() [back-off: 3s, 6s, 12s, 24s, 30s max]
       ├── success: fault_reset_all() + reset_stability_state()
       │            s_forced_unsafe_isr = false, recover_attempts = 0
       └── failure: retry at next back-off, IMU-B → degrade after 4 B-only failures

If FAULT_INTEGRITY:
  └─ No auto-recovery. Power cycle or WDT reset required.
```

---

## 6. Failure Modes & Risks

### 6.1 Fault Code Reference

| Bits | Name | Trigger | Recovery |
|---|---|---|---|
| `0x0001` | `FAULT_IMU_A_COMM` | ≥ `stale_fault_ticks` stale reads, or 3× WHO_AM_I fail, or 3× config CRC fail | `imu_recover()` |
| `0x0002` | `FAULT_IMU_B_COMM` | Same for IMU-B | `imu_recover()` |
| `0x0004` | `FAULT_GYRO_RANGE` | `|ω| > 248 dps` or NaN | 5 clean samples |
| `0x0008` | `FAULT_ACCEL_RANGE` | `|a| < 0.05g` or `> 4.4g` or NaN | 5 clean samples |
| `0x0010` | `FAULT_OUTPUT` | SAFE_MON reads LOW when HIGH commanded | TRUE readback pass |
| `0x0020` | `FAULT_FROZEN` | Loop frozen > 26 ISR ticks (~31 ms) | `imu_recover()` |
| `0x0040` | `FAULT_TIMING` | ISR period jitter > ±5% | 5 clean ISR ticks |
| `0x0080` | `FAULT_DIVERSE` | Window mean accel mag ≠ 1 g ± 0.1 g | 5 clean eval periods |
| `0x0100` | `FAULT_DUAL_DIVERGE` | `|roll_A − roll_B|` or `|pitch_A − pitch_B|` > 5° | 5 clean eval periods |
| `0x0200` | `FAULT_SELFTEST` | Accel or gyro self-test out of acceptance band | `imu_recover()` |
| `0x0400` | `FAULT_INTEGRITY` | Shadow mismatch, CRC fail, stack > 6 KB, common-mode lock, SAFE_MON stuck HIGH | Power cycle / WDT |

### 6.2 "Why Not Stable" Codes

Printed in the Serial status line (`why=...`) and returned in `/api/status`:

| Code | Meaning |
|---|---|
| `OK` | Platform stable |
| `DRIFT` | Anchor exceeded `anchor_max_deg` |
| `SHADOW` | Fault shadow mismatch |
| `CRC` | CRC-16 mismatch |
| `FAULT` | Any `fault_mask` bit set (FSM in SFST_FAULT or fault bits active) |
| `HOLD` | Instant-hold timer active post-motion |
| `FILL` | Window buffer not yet full |
| `GYRO_SPK` | Gyro spike count over `motion_samples_max` |
| `ACCEL_SPK` | Accel spike count over `shock_samples_max` |
| `SPREAD` | Tilt spread over `spread_max_deg` |
| `HW_FSM` | Hardware FSM gate not passing (sensor FSM not stable) |
| `UNKNOWN` | Stable=FALSE but no specific cause found |

### 6.3 Critical Failure Modes

**IMU SPI communication failure:**  
`asm_read_sensors()` returns false when STATUS_REG = 0xFF (SPI dead), data-not-ready persists after 50 µs retry, or CRC-8 burst freeze triggers after 5 identical frames. The stale counter increments; `FAULT_IMU_A/B_COMM` sets after `stale_fault_ticks` (default 15 = ~18 ms at 833 Hz). Output is already forced false on the first stale read. `imu_recover()` is called with exponential back-off.

**Common-mode SPI fault:**  
MISO stuck at a constant voltage returns all-same bytes, caught by CRC-8 burst freeze. A fault that returns the same valid-looking data for both CS transactions is caught by the common-mode lock detection (`FAULT_INTEGRITY` after 3 consecutive identical pre-remap vectors).

**Config register corruption:**  
ODR or range setting changed by SPI glitch → CRC-8 shadow mismatch → `FAULT_IMU_A/B_COMM` → `imu_recover()` re-initialises and resets the shadow.

**Stack overflow:**  
Watermark catches > 6 KB usage and sets `FAULT_INTEGRITY`. Does not prevent overflow if a single call frame exceeds the remaining 2 KB margin; `FAULT_INTEGRITY` is non-recoverable (power cycle required).

**Floating SAFE_MON_PIN (wiring omitted):**  
`INPUT_PULLDOWN` holds unwired pin LOW. A FALSE readback (output=LOW, mon=LOW) passes without fault. A TRUE readback (output=HIGH, mon=LOW) sets `FAULT_OUTPUT` and blocks stable assertion. The startup dual-state check also fires `FAULT_OUTPUT` if the monitoring wire is absent and SAFE_OUT_PIN was HIGH during the HIGH phase of the check.

**IMU-B permanent failure:**  
After 4 consecutive B-only recovery failures, `s_imu_b_degraded` is set. Single-channel (A-only) operation continues. `FAULT_IMU_B_COMM` and `FAULT_DUAL_DIVERGE` are suppressed in `effective_faults()`. All independent-channel safety properties are degraded to single-sensor level while in this mode.

---

## 7. Limitations / Known Issues

### 7.1 RAM March Test Coverage — Critical

**Gap:** `ram_march_test()` covers only the 128-byte `march_buf`. DTCM globals (`s_fault_mask`, `s_fault_mask_inv`, `s_tilt_buf`, `g_params`, `s_imu_a`, `s_imu_b`, etc.) are not march-tested.

**Required for SIL 3:** A linker-placed march test covering the entire DTCM data region at startup, executed before any variable initialisation that could mask a latent fault.

### 7.2 No Brownout Detection Configured

**Gap:** iMXRT1062 DCDC has a configurable brownout threshold, not currently set. Factory default trips at ~2.7 V. Under a slow power droop the CPU may execute erratically before the BOD fires.

**Required for SIL 3:** Configure `DCDC_REG3[TRG]` and enable the BOD interrupt for a controlled reset before CPU misbehaviour under supply droop.

### 7.3 Default Config Auth Token (Security / Safety)

**Gap:** `CONFIG_AUTH_TOKEN = "psm-change-me-v1"` is a public default. Any actor on the LAN can modify stability thresholds.

**Required before deployment:** Change `CONFIG_AUTH_TOKEN` to a strong, randomly generated secret before building the firmware for deployment.

### 7.4 Single Watchdog, No Window Mode

**Gap:** One internal WDT (WDOG3), not windowed. A tight busy-loop that feeds the WDT at high frequency would not be caught.

**For SIL 3:** External windowed watchdog IC (independent clock) or enable WDOG3 window mode so a feed that arrives too early also triggers reset.

### 7.5 Shared SPI1 Bus — Residual Common-Mode Exposure

**Gap:** Both IMUs share the same physical MISO/MOSI/SCK conductors. A common-mode fault that produces two distinct but consistently wrong datasets (not identical — thus not caught by the lock check) could fool both channels simultaneously without triggering divergence.

**Mitigations in place:** Config CRC-8 shadow, burst CRC-8 freeze, common-mode lock detection, and dual-diverge check together cover the dominant failure scenarios. The residual is a correlated fault producing non-identical but consistently wrong output — this requires a hardware fault tolerant architecture (separate buses) to fully eliminate.

### 7.6 CRC Does Not Cover Tilt Buffer

**Gap:** `s_tilt_buf` and `s_accel_mag_buf` (up to 520 × 3 floats ≈ 6 KB) are not CRC-covered. A RAM corruption affecting the tilt buffer could produce a false spread result without triggering a CRC mismatch. The CRC-16 covers `s_buf_count` and `s_buf_head` (buffer metadata), so a corruption that alters sample data without changing the metadata would be undetected.

### 7.7 No Certified Toolchain or MISRA Compliance

**Gap:** Built with GCC (Teensyduino) — not a TÜV-certified safety compiler. Code uses C++ features excluded by MISRA C:2012. No static analysis results are recorded.

**Required for SIL 3:** Certified compiler (e.g., TASKING, IAR with SC option) or documented qualification process. MISRA C compliance or documented deviations with rationale.

### 7.8 Reset-Cause Not Persisted to EEPROM

**Gap:** WDT resets are logged to Serial only. Post-mortem diagnosis in the field (no Serial terminal) is not possible.

**Recommendation:** Write `SRC_SRSR` value, firmware version, and a fault mask snapshot to EEPROM at startup for offline diagnostics.

### 7.9 IMU-B Degrade Mode Reduces Fault Coverage

**Gap:** When `s_imu_b_degraded` is true (IMU-B permanently failed), the system reverts to single-channel operation. All independent-channel safety properties — dual-diverge, common-mode lock detection, diversity check — are unavailable. The system ASIL/SIL level in this state is equivalent to a single-sensor system.

**Mitigation:** Degraded mode is logged to Serial, reflected in `g_safety_status.imu_b_degraded`, and reported in the web dashboard. An operator alarm on the `imu_b_degraded` field in `/api/status` is strongly recommended.

### 7.10 Yaw Excluded (Design Note)

Yaw is excluded from all stability distance calculations by design. Gyro-integrated yaw accumulates ~0.1 dps of drift over a 1–5 s window and conflates platform rotation with instability. Yaw is retained in the `Tilt` struct for diagnostic use only.

---

*End of document.*
