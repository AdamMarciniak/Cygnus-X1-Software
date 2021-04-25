#pragma once

#include "BasicLinearAlgebra.h"
#include "Data.h"

extern void updateBaro(float altitude);
extern void updateAccel(float accel);
extern float getKalmanPosition();

extern float getKalmanVelocity();

extern float getKalmanAcceleration();
extern void zeroKalman();
extern void handleChangeBaroNoise();
extern void initKalman();
extern void predict();
void setDataVariables();
