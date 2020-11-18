#include <stdio.h>
#include "kalman.h"

struct kalman myKalman;

int main(void){
    kalmanInit(&myKalman, 0.003, 0.002, 0.01);
    

    return 0;
}