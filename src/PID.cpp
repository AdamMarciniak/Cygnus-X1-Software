#include "PID.h"



Chrono PIDTimer;

PID z_PID;
PID y_PID;


void initPIDs()
{
  z_PID.setTunings(Z_KP, Z_KI, Z_KD);
  z_PID.setOutputLimits(-SERVO_RANGE, SERVO_RANGE);
  z_PID.setSetpoint(0);

  y_PID.setTunings(Y_KP, Y_KI, Y_KD);
  y_PID.setOutputLimits(-SERVO_RANGE, SERVO_RANGE);
  y_PID.setSetpoint(0);
}

void setZPIDInput(float val) {
  z_PID.setInput(val);
}

void setYPIDInput(float val) {
  y_PID.setInput(val);
}

void computeBothPIDs(){
  z_PID.compute();
  y_PID.compute();

  data.p_err_y = y_PID.getPError();
  data.p_err_z = z_PID.getPError();

  data.i_err_y = y_PID.getIError();
  data.i_err_z = z_PID.getIError();

  data.d_err_y = y_PID.getDError();
  data.d_err_z = z_PID.getDError();

  data.servo_z = z_PID.getOutput();
  data.servo_y = y_PID.getOutput();
}

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

float PID::getPError()
{
  return error;
}

float PID::getDError()
{
  return dErr;
}

float PID::getIError()
{
  return ITerm;
}

float PID::getOutput()
{
  return Output;
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
    deltaT = float((now - lastTime)) / 1000000.0f;
    error = Setpoint - Input;

    ITerm += (ki * error) * deltaT;

    if (lastError < 0.0 && error > 0.0)
    {
      ITerm = 0.0;
    }

    if (lastError > 0.0 && error < 0.0)
    {
      ITerm = 0.0;
    }

    if (error == 0.0)
    {
      ITerm = 0.0;
    }

    if (ITerm > outMax)
      ITerm = outMax;
    else if (ITerm < outMin)
      ITerm = outMin;
    dErr = (error - lastError) / deltaT;

    Output = kp * error + ITerm + kd * dErr;
    if (Output > outMax)
    {
      Output = outMax;
    }
    if (Output < outMin)
    {
      Output = outMin;
    }

    lastError = error;
    lastTime = now;
  }
  return;
}

void PID::setTunings(float Kp, float Ki, float Kd)
{
  kp = Kp;
  ki = Ki;
  kd = Kd;
}

void PID::setOutputLimits(int Min, int Max)
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
