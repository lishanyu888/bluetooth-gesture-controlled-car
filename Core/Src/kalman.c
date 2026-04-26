#include "kalman.h"

void Kalman_Init(Kalman_t *kalman)
{
    if (kalman == 0) {
        return;
    }

    kalman->q_angle = 0.001f;
    kalman->q_bias = 0.003f;
    kalman->r_measure = 0.03f;

    kalman->angle = 0.0f;
    kalman->bias = 0.0f;
    kalman->rate = 0.0f;

    kalman->p00 = 0.0f;
    kalman->p01 = 0.0f;
    kalman->p10 = 0.0f;
    kalman->p11 = 0.0f;
}

void Kalman_SetAngle(Kalman_t *kalman, float angle)
{
    if (kalman == 0) {
        return;
    }
    kalman->angle = angle;
}

float Kalman_GetAngle(Kalman_t *kalman, float new_angle, float new_rate, float dt)
{
    float s;
    float k0;
    float k1;
    float y;
    float p00_temp;
    float p01_temp;

    if (kalman == 0) {
        return 0.0f;
    }

    kalman->rate = new_rate - kalman->bias;
    kalman->angle += dt * kalman->rate;

    kalman->p00 += dt * (dt * kalman->p11 - kalman->p01 - kalman->p10 + kalman->q_angle);
    kalman->p01 -= dt * kalman->p11;
    kalman->p10 -= dt * kalman->p11;
    kalman->p11 += kalman->q_bias * dt;

    s = kalman->p00 + kalman->r_measure;
    k0 = kalman->p00 / s;
    k1 = kalman->p10 / s;

    y = new_angle - kalman->angle;
    kalman->angle += k0 * y;
    kalman->bias += k1 * y;

    p00_temp = kalman->p00;
    p01_temp = kalman->p01;

    kalman->p00 -= k0 * p00_temp;
    kalman->p01 -= k0 * p01_temp;
    kalman->p10 -= k1 * p00_temp;
    kalman->p11 -= k1 * p01_temp;

    return kalman->angle;
}
