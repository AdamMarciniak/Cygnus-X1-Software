
#include "KalmanNew.h"

using namespace BLA;

KalmanNew::KalmanNew()
{
  Q = {10, 0, 0, 0,
       0, 10, 0, 0,
       0, 0, 0.1, 0,
       0, 0, 0, 10};

  R_Baro = {1};
  R_GPS = {10};

  I = {1, 0, 0, 0,
       0, 1, 0, 0,
       0, 0, 1, 0,
       0, 0, 0, 1};

  H_Baro = {1, 0, 0, -1};
  H_GPS = {1, 0, 0, 0};

  zeroKalman();
}

void KalmanNew::zeroKalman()
{
  X = {
      0,
      0,
      9.81,
      0,
  };

  P = {100, 0, 0, 0,
       0, 100, 0, 0,
       0, 0, 0.01, 0,
       0, 0, 0, 100};
}

void KalmanNew::predict(float accel)
{

  currentTime = micros();
  delT = (currentTime - prevTime) / 1000000.0f;
  prevTime = currentTime;

  A = {
      accel * delT * delT * 0.5,
      accel * delT,
      0,
      0};

  if (!isFirstStep)
  {
    F = {
        1, delT, -0.5 * delT * delT, 0,
        0, 1, -delT, 0,
        0, 0, 1, 0,
        0, 0, 0, 1};

    X = F * X + A;
    P = F * P * ~F + Q;
  }
  isFirstStep = false;
}

void KalmanNew::updateBaro(float val)
{
  Z = {val};
  K_Baro = P * ~H_Baro * (H_Baro * P * ~H_Baro + R_Baro).Inverse();
  X = X + K_Baro * (Z - H_Baro * X);
  P = (I - K_Baro * H_Baro) * P * (~(I - K_Baro * H_Baro)) + K_Baro * R_Baro * ~K_Baro;
}

void KalmanNew::updateGPS(float val)
{
  Z = {val};
  K_GPS = P * ~H_GPS * (H_GPS * P * ~H_GPS + R_GPS).Inverse();
  X = X + K_GPS * (Z - H_GPS * X);
  P = (I - K_GPS * H_GPS) * P * (~(I - K_GPS * H_GPS)) + K_GPS * R_GPS * ~K_GPS;
}

float KalmanNew::getPosition()
{
  return X(0, 0);
}

float KalmanNew::getVelocity()
{
  return X(1, 0);
}

float KalmanNew::getGravity()
{
  return X(2, 0);
}

float KalmanNew::getBias()
{
  return X(3, 0);
}