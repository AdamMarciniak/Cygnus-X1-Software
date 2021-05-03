#pragma once

#include "BasicLinearAlgebra.h"
#include "Data.h"

extern void updateBaro(float altitude);

extern float getKalmanPosition();

extern float getKalmanVelocity();

extern float getKalmanAcceleration();
extern void zeroKalman();
extern void handleChangeBaroNoise();
extern void initKalman();
extern void predict(float accel);
void setDataVariables();
