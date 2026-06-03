// app_light.cpp -- Olight Odin GL open-drain PWM controller (QM layer).

#include "app_light.h"
#include <Arduino.h>

// Open-drain PWM: Olight Odin GL remote port.
// IntervalTimer at 1500 Hz, 16-tick phase counter → 93.75 Hz output.
// Ticks  0-12 → INPUT (high-Z, 81.25%)   remote pull-up → light ON.
// Ticks 13-15 → OUTPUT+LOW (GND, 18.75%) sinks remote port → light dimmed/controlled.
// LIGHT_PIN data latch is never driven HIGH; pin either floats or sinks. No 3.3 V into remote port.

static IntervalTimer    s_light_timer;
static volatile bool    s_active = false;
static volatile uint8_t s_phase  = 0U;

static void lightISR()
{
    if (s_phase == 0U) {
        if (s_active) pinMode(LIGHT_PIN, INPUT);
    } else if (s_phase == 13U) {
        if (s_active) { pinMode(LIGHT_PIN, OUTPUT); digitalWrite(LIGHT_PIN, LOW); }
    }
    s_phase = (s_phase + 1U < 16U) ? (s_phase + 1U) : 0U;
}

void light_init()
{
    digitalWrite(LIGHT_PIN, LOW);                        // data latch permanently 0
    pinMode(LIGHT_PIN, INPUT);                           // safe high-Z at startup
    s_light_timer.begin(lightISR, 1000000U / 1500U);    // 666.67 µs tick
    s_light_timer.priority(192);                         // lower than safety ISR default (128)
}

void light_set(bool on)
{
    if (!on) {
        noInterrupts(); s_active = false; s_phase = 0U; interrupts();
        pinMode(LIGHT_PIN, INPUT);   // s_active=false guards ISR from touching pin
    } else {
        noInterrupts(); s_phase = 0U; s_active = true; interrupts();
    }
}

bool light_get() { return s_active; }
