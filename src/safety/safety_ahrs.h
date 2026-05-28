#pragma once
// safety_ahrs.h -- Mahony complementary filter (ASIL B / SIL 2).
// No app/ includes.
#include "safety_defs.h"
#include "safety_types.h"

// Per-IMU AHRS instances (defined in safety_ahrs.cpp)
extern Mahony s_imu_a;
extern Mahony s_imu_b;

void mahony_init  (Mahony &m);
void mahony_update(Mahony &m,
                   float gx, float gy, float gz,
                   float ax, float ay, float az);
void mahony_to_tilt(const Mahony &m, float &roll_deg, float &pitch_deg);
