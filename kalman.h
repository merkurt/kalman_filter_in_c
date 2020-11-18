#ifndef __KALMAN_
#define __KALMAN_
#include <stdint.h>

struct kalman{
    float angle;
    float bias;
    float rate;

    float Y;
    float S;
    float K[2];
    float P_temp[2];

    float qAngle;
    float qGyroBias;
    float rMeasure;
    float P[4];
};

uint8_t kalmanInit(struct kalman* p, float qAngle, float qGyroBias, float rMeasure);
float kalmanGetAngle(struct kalman* p, float newAngle, float newRate, float dt);
void kalmanSetAngle(struct kalman* p, float angle);
#endif
