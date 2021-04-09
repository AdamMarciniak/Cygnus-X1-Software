#pragma once

extern void updateBaro(float altitude);
extern void updateAccel(float accel);
extern float getKalmanPosition();

extern float getKalmanVelocity();

extern float getKalmanAcceleration();
extern void zeroKalman();
extern void handleChangeBaroNoise();

void setDataVariables();