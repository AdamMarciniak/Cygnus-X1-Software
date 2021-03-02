#ifndef PID_H
#define PID_H

class PID
{
public:
  PID();
  void compute();
  void setTunings(float Kp, float Ki, float Kd);
  void setOutputLimits(float Min, float Max);
  void setSetpoint(float setPt);
  void setInput(float input);
  void incrementKP(float num);
  void incrementKI(float num);
  void incrementKD(float num);
  float getPError();
  float getDError();
  float getIError();
  float Output;
  float kp, ki, kd;

private:
  unsigned long lastTime;
  float Input, Setpoint;
  float ITerm, lastError, error, dErr;
  float deltaT;
  float outMin, outMax;
  bool firstCompute;
};

#endif