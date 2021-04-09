#pragma once

extern float ypr[3];
extern float accel_raw[3];
extern float vel_local[3];
extern bool initIMU();
extern void getAccel();
extern void getYPR();
extern void getAccel();
extern void zeroGyroscope();
extern void getCurrentYawAndPitchFromAccel();
extern void getInitYawAndPitchBiases();
