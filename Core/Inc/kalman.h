#ifndef CAR_KALMAN_H
#define CAR_KALMAN_H

typedef struct {
    float q_angle;
    float q_bias;
    float r_measure;

    float angle;
    float bias;
    float rate;

    float p00;
    float p01;
    float p10;
    float p11;
} Kalman_t;

void Kalman_Init(Kalman_t *kalman);
void Kalman_SetAngle(Kalman_t *kalman, float angle);
float Kalman_GetAngle(Kalman_t *kalman, float new_angle, float new_rate, float dt);

#endif /* CAR_KALMAN_H */
