#include "kalman.h"

uint8_t kalmanInit(struct kalman* p, float qAngle, float qGyroBias, float rMeasure){
    p->qAngle = qAngle;
    p->qGyroBias = qGyroBias;
    p->rMeasure = rMeasure;

    p->angle = 0;
    p->bias = 0;
    p->rate = 0;

    p->Y = 0;
    p->S = 0;
    p->K[0] = 0;
    p->K[1] = 0;
    p->P_temp[0] = 0;
    p->P_temp[1] = 0;
    p->P[0] = 0;
    p->P[1] = 0;
    p->P[2] = 0;
    p->P[3] = 0;
    return 0;
}

float kalmanGetAngle(struct kalman* p, float newAngle, float newRate, float dt){
    // step 1
    p->rate = newRate - p->bias;
    p->angle += dt * p->rate;

    // step 2
    p->P[0] += dt * (dt*p->P[3] - p->P[1] - p->P[2] + p->qAngle);
	p->P[1] -= dt * p->P[3];
	p->P[2] -= dt * p->P[3];
	p->P[3] += p->qGyroBias * dt;

    // step 4
    p->S = p->P[0] + p->rMeasure;

    // step 5
    p->K[0] = p->P[0] / p->S;
    p->K[1] = p->P[2] / p->S;

    // step 3
	p->Y = newAngle - p->angle;


    // step 6
    p->angle += p->K[0] * p->Y;
    p->bias += p->K[1] * p->Y;

    // step 7
    p->P_temp[0] = p->P[0];
    p->P_temp[1] = p->P[1];
    p->P[0] -= p->K[0] * p->P_temp[0];
    p->P[1] -= p->K[0] * p->P_temp[1];
    p->P[2] -= p->K[1] * p->P_temp[0];
    p->P[3] -= p->K[1] * p->P_temp[1];

    return p->angle;
}

void kalmanSetAngle(struct kalman* p, float angle){
    p->angle = angle;
}
