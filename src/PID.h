#pragma once

#include "Chrono.h"
#include <Arduino.h>
#include "Data.h"
#include "ServoControl.h"

#define Y_KP 0.5
#define Y_KI 0.06
#define Y_KD 0.14

#define Z_KP 0.5
#define Z_KI 0.06
#define Z_KD 0.14

extern void initPIDs();

extern void setZPIDInput(float val);
extern void setYPIDInput(float val);
extern void computeBothPIDs();

class PID
{
public:
  PID();
  void compute();
  void setTunings(float Kp, float Ki, float Kd);
  void setOutputLimits(int Min, int Max);
  void setSetpoint(float setPt);
  void setInput(float input);
  void incrementKP(float num);
  void incrementKI(float num);
  void incrementKD(float num);
  float getPError();
  float getDError();
  float getIError();
  float getOutput();
  float Output;
  float kp, ki, kd;

private:
  unsigned long lastTime;
  float Input, Setpoint;
  float ITerm, lastError, error, dErr;
  float deltaT;
  int outMin, outMax;
  bool firstCompute;
};

