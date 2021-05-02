
#include "KalmanNew.h"
#include "Data.h"

using namespace BLA;

KalmanNew::KalmanNew()
{
  Q = {.00015, 0, 0, 0,
       0, 0.00015, 0, 0,
       0, 0, 0.001, 0,
       0, 0, 0, 0.00083};

  R_Baro = {1};
  R_GPS = {100};

  I = {1, 0, 0, 0,
       0, 1, 0, 0,
       0, 0, 1, 0,
       0, 0, 0, 1};

  H_Baro = {1, 0, 0, 1};
  H_GPS = {1, 0, 0, 0};

  zeroKalman();
}

void KalmanNew::setBias(float bias)
{
  X = {
      0,
      0,
      9.81,
      bias,
  };

  P = {100, 0, 0, 0,
       0, 0.0001, 0, 0,
       0, 0, 0.0001, 0,
       0, 0, 0, 0.0001};
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
  updateVariables();
}

void KalmanNew::updateBaro(float val)
{
  Z = {val};
  K_Baro = P * ~H_Baro * (H_Baro * P * ~H_Baro + R_Baro).Inverse();
  X = X + K_Baro * (Z - H_Baro * X);
  P = (I - K_Baro * H_Baro) * P * (~(I - K_Baro * H_Baro)) + K_Baro * R_Baro * ~K_Baro;
  updateVariables();
}

void KalmanNew::updateGPS(float val)
{
  Z = {val};
  K_GPS = P * ~H_GPS * (H_GPS * P * ~H_GPS + R_GPS).Inverse();
  X = X + K_GPS * (Z - H_GPS * X);
  P = (I - K_GPS * H_GPS) * P * (~(I - K_GPS * H_GPS)) + K_GPS * R_GPS * ~K_GPS;
  updateVariables();
}

void KalmanNew::updateVariables()
{
  data.kal_X_pos = X(0, 0);
  data.kal_X_vel = X(1, 0);
  data.kal_X_accel = X(2, 0);
  data.kal_Z_bias = X(3, 0);

  data.kal_X_p = P(0, 0);
  data.kal_V_p = P(1, 1);
  data.kal_G_p = P(2, 2);
  data.kal_B_p = P(3, 3);
};
