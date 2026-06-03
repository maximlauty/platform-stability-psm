// Platform Stability Monitor v2 -- Teensy 4.1 + ASM330LHB x2 (SPI)
// ASIL D (ISO 26262) / SIL 3 (IEC 61508) / MIL-STD-882E
// Entry point only. All safety logic in safety/, application logic in app/.

#ifndef __DMB
#define __DMB() asm volatile("dmb 0xF" ::: "memory")
#endif
#ifndef __DSB
#define __DSB() asm volatile("dsb 0xF" ::: "memory")
#endif

#include "src/safety/safety_defs.h"
#include "src/safety/safety_types.h"
#include "src/safety/safety_monitor.h"
#include "src/safety/safety_imu.h"
#include "src/safety/safety_ahrs.h"
#include "src/safety/safety_stability.h"
#include "src/safety/safety_isr.h"
#include "src/app/app_params.h"
#include "src/app/app_http.h"
#include "src/app/app_light.h"

void setup()
{
    // 1. Load EEPROM params and publish thresholds to safety layer.
    //    Must happen before safety_init() so g_safety_thresholds is valid for tick 1.
    bool params_ok = params_load();
    app_publish_thresholds();

    // 2. Safety layer init: GPIO, SPI, Serial, IMU, self-test, FSM, WDT, timer.
    safety_init();

    // Print params outcome after Serial.begin() (called inside safety_init).
    if (params_ok) Serial.println("[PARAMS] Loaded from EEPROM");
    else           Serial.println("[PARAMS] Defaults (EEPROM invalid or version mismatch)");

    // 3. App layer init: Ethernet, web server.
    // Ethernet PHY link-up can take several seconds; widen WDT window then tighten.
    wdt_begin(8000);
    app_init();
    light_init();
    wdt_begin(1500);
    wdt_feed();

    // 4. Start the 833 Hz ISR now that loop() is about to run.
    //    Deferring prevents false FAULT_FROZEN during the long Ethernet init above.
    safety_start_isr();
}

void loop()
{
    // Safety tick: WDT feed, AHRS integration, stability eval, output control.
    safety_tick();

    // App tick: web server accept/handle, Ethernet.loop().
    app_tick();
}
