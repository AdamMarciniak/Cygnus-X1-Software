#pragma once

extern void updateAccel(float accel);

extern void updateBaro(float altitude);

extern float getKalmanPosition();

extern float getKalmanVelocity();

extern float getKalmanAcceleration();
extern void zeroKalman();