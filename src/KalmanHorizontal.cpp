#include "KalmanHorizontal.h"

using namespace BLA;

KalmanH::KalmanH()
{
  Q = {0.01, 0, 0, 0,
       0, 1, 0, 0,
       0, 0, 1, 0,
       0, 0, 0, 0.1};

  R = {1};

  I = {1, 0, 0, 0,
       0, 1, 0, 0,
       0, 0, 1, 0,
       0, 0, 0, 1};

  H = {0, 0, 1, 0};

  zeroKalman();
}

void KalmanH::zeroKalman()
{
  X = {
      0,
      0,
      0,
      0,
  };

  P = {10, 0, 0, 0,
       0, 10, 0, 0,
       0, 0, 10, 0,
       0, 0, 0, 100};
}

void KalmanH::predict()
{

  currentTime = micros();
  delT = (currentTime - prevTime) / 1000000.0f;
  prevTime = currentTime;

  if (!isFirstStep)
  {
    F = {
        1, delT, 0.5 * delT * delT, 0,
        0, 1, delT, 0,
        0, 0, 1, -1,
        0, 0, 0, 1};

    X = F * X;
    P = F * P * ~F + Q;
  }
  isFirstStep = false;
}

void KalmanH::update(float val)
{
  predict();
  Z = {val};
  K = P * ~H * (H * P * ~H + R).Inverse();
  X = X + K * (Z - H * X);
  //P = P - K_Accel * H_Accel * P;
  P = (I - K * H) * P * (~(I - K * H)) + K * R * ~K;
}

float KalmanH::getPosition()
{
  return X(0, 0);
}

float KalmanH::getVelocity()
{
  return X(1, 0);
}

float KalmanH::getAcceleration()
{
  return X(2, 0);
}

float KalmanH::getBias()
{
  return X(3, 0);
}
