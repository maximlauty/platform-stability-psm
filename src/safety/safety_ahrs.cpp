// safety_ahrs.cpp -- Mahony complementary filter (ASIL B / SIL 2).
// Independence rule: no app/ includes.

#include "safety_ahrs.h"
#include <math.h>

Mahony s_imu_a = {1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
Mahony s_imu_b = {1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};

void mahony_init(Mahony &m)
{
    m.qw = 1.0f; m.qx = m.qy = m.qz = 0.0f;
    m.exi = m.eyi = m.ezi = 0.0f;
}

void mahony_update(Mahony &m,
                   float gx, float gy, float gz,
                   float ax, float ay, float az)
{
    float norm = sqrtf(ax*ax + ay*ay + az*az);
    if (norm > 1e-6f) {
        norm = 1.0f / norm;
        ax *= norm; ay *= norm; az *= norm;

        float vx = 2.0f*(m.qx*m.qz - m.qw*m.qy);
        float vy = 2.0f*(m.qw*m.qx + m.qy*m.qz);
        float vz = m.qw*m.qw - m.qx*m.qx - m.qy*m.qy + m.qz*m.qz;

        float ex = (ay*vz - az*vy);
        float ey = (az*vx - ax*vz);
        float ez = (ax*vy - ay*vx);

        // MA-10 style: MAHONY_KI/AHRS_HZ avoids accumulating across variable dt
        m.exi += ex * (MAHONY_KI / (float)AHRS_HZ);
        m.eyi += ey * (MAHONY_KI / (float)AHRS_HZ);
        m.ezi += ez * (MAHONY_KI / (float)AHRS_HZ);

        gx += MAHONY_KP * ex + m.exi;
        gy += MAHONY_KP * ey + m.eyi;
        gz += MAHONY_KP * ez + m.ezi;
    }

    float dt = 1.0f / (float)AHRS_HZ;
    float qw = m.qw + (-m.qx*gx - m.qy*gy - m.qz*gz) * (0.5f*dt);
    float qx = m.qx + ( m.qw*gx + m.qy*gz - m.qz*gy) * (0.5f*dt);
    float qy = m.qy + ( m.qw*gy - m.qx*gz + m.qz*gx) * (0.5f*dt);
    float qz = m.qz + ( m.qw*gz + m.qx*gy - m.qy*gx) * (0.5f*dt);

    float qnorm = sqrtf(qw*qw + qx*qx + qy*qy + qz*qz);
    if (qnorm > 1e-6f) {
        qnorm = 1.0f / qnorm;
        m.qw = qw*qnorm; m.qx = qx*qnorm;
        m.qy = qy*qnorm; m.qz = qz*qnorm;
    } else {
        mahony_init(m);
    }
}

void mahony_to_tilt(const Mahony &m, float &roll_deg, float &pitch_deg)
{
    // MA-3: yaw excluded from all safety calculations
    roll_deg  = atan2f(2.0f*(m.qw*m.qx + m.qy*m.qz),
                       1.0f - 2.0f*(m.qx*m.qx + m.qy*m.qy)) * RAD_TO_DEG;
    pitch_deg = asinf(2.0f*(m.qw*m.qy - m.qz*m.qx)) * RAD_TO_DEG;
}
