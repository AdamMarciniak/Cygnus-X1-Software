#pragma once

#include "Chrono.h"
#include <Arduino.h>
#include "Data.h"
#include "ServoControl.h"
#include "Config.h"

extern void initPIDs();
extern void setZPIDInput(float val);
extern void setYPIDInput(float val);
extern void computeBothPIDs();
extern void setYPIDSetpoint(int setpoint);
extern void setZPIDSetpoint(int setpoint);

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
  float getDelT();

private:
  unsigned long lastTime;
  float Input, Setpoint;
  float ITerm, lastError, error, dErr;
  float deltaT;
  int outMin, outMax;
  bool firstCompute;
};
