#pragma once
// safety_types.h -- core data types for the safety layer.
// No app/ includes permitted.
#include "safety_defs.h"

struct Tilt { float roll_deg; float pitch_deg; float yaw_deg; };

struct Mahony {
    float qw, qx, qy, qz;
    float exi, eyi, ezi;
};

enum StabFSM : uint8_t {
    SFST_INIT = 0,
    SFST_FILLING,
    SFST_STABLE,
    SFST_UNSTABLE,
    SFST_FAULT,
};
