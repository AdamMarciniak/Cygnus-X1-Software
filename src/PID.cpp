#include "PID.h"
#include "Chrono.h"
#include <Arduino.h>
#include "Data.h"
Chrono PIDTimer;

PID::PID()
{
  firstCompute = true;
  setSetpoint(0.0f);
}

void PID::setSetpoint(float setPt)
{
  Setpoint = setPt;
}

void PID::setInput(float inpt)
{
  Input = inpt;
}



void PID::compute()
{

  if (firstCompute)
  {
    firstCompute = false;
    lastTime = micros();
    lastError = 0.0;
  }
  else
  {
    unsigned long now = micros();
    float timeChange = float((now - lastTime)) / 1000000.0f;
    data.pid_delta = timeChange;
    /*Compute all the working error variables*/
    float error = Setpoint - Input;
    ITerm += (ki * error) * timeChange;
    if (ITerm > outMax)
      ITerm = outMax;
    else if (ITerm < outMin)
      ITerm = outMin;
    float dErr = (error - lastError) / timeChange;

    /*Compute PID Output*/
    Output = kp * error + ITerm + kd * dErr;
    if (Output > outMax)
      Output = outMax;
    else if (Output < outMin)
      Output = outMin;
    /*Remember some variables for next time*/
    lastError = error;
    lastTime = now;
    // Serial.print(timeChange, 6);
    // Serial.print(" ");
    // Serial.print(Input);
    // Serial.print(" ");
    // Serial.print(error);
    // Serial.print(" ");
    // Serial.print(dErr);
    // Serial.print(" ");
    // Serial.println(ITerm);
  }
}

void PID::setTunings(float Kp, float Ki, float Kd)
{
  kp = Kp;
  ki = Ki;
  kd = Kd;
}

void PID::setOutputLimits(float Min, float Max)
{
  if (Min > Max)
    return;
  outMin = Min;
  outMax = Max;
}

void PID::incrementKP(float num)
{
  float val = kp + num;
  if (val >= 0)
  {
    kp = val;
  }
  else
  {
    kp = 0.0f;
  }
}

void PID::incrementKI(float num)
{
  float val = ki + num;
  if (val >= 0)
  {
    ki = val;
  }
  else
  {
    ki = 0.0f;
  }
}
void PID::incrementKD(float num)
{
  float val = kd + num;
  if (val >= 0)
  {
    kd = val;
  }
  else
  {
    kd = 0.0f;
  }
}
