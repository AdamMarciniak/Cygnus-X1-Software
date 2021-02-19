#ifndef NAV_H
#define NAV_H

extern float ypr[3];
extern float accel_raw[3];
extern float vel_local[3];
extern bool initIMU();
extern void getAccel();
extern void getYPR();
extern void getAccel();
extern void zeroGyroscope();

#endif
