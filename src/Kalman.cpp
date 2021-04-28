#include "Kalman.h"

using namespace BLA;

BLA::Matrix<3, 3> Q = {0.01, 0, 0,
                       0, 0.01, 0,
                       0, 0, 10};

BLA::Matrix<1, 1> R_Accel = {0.1};

// Measured baro variance was 0.03251531
BLA::Matrix<1, 1> R_Baro = {10};

BLA::Matrix<1, 1> R_GPS = {100};

BLA::Matrix<3, 1> X = {
    0,
    0,
    0,
};

BLA::Matrix<3, 3> P = {1, 0, 0,
                       0, 1, 0,
                       0, 0, 1};

BLA::Matrix<1, 1> PBaro = {0};
BLA::Matrix<3, 3> I = {1, 0, 0,
                       0, 1, 0,
                       0, 0, 1};

BLA::Matrix<1, 3> H_Baro = {1, 0, 0};

BLA::Matrix<1, 3> H_GPS = {1, 0, 0};

BLA::Matrix<1, 3> H_Accel = {0, 0, 1};

BLA::Matrix<3, 3> F;
BLA::Matrix<1, 1> Z_Accel;
BLA::Matrix<1, 1> Z_Baro;
BLA::Matrix<1, 1> Z_GPS;
BLA::Matrix<3, 1> K_Accel;
BLA::Matrix<3, 1> K_Baro;
BLA::Matrix<3, 1> K_GPS;

void initKalman()
{
  zeroKalman();
}

void zeroKalman()
{
  X = {
      0,
      0,
      0,
  };

  P = {0.001, 0, 0,
       0, 0.001, 0,
       0, 0, 0.001};
}

unsigned long currentTime = 0;
unsigned long prevTime = 0;
float delT = 0.0f;

bool isFirstStep = true;

void predict()
{
  currentTime = micros();
  delT = (currentTime - prevTime) / 1000000.0f;
  data.loopTime = delT;
  prevTime = currentTime;
  F = {
      1, delT, 0.5 * delT * delT,
      0, 1, delT,
      0, 0, 1};

  X = F * X;
  P = F * P * ~F + Q;
}

void updateAccel(float accel)
{
  predict();
  Z_Accel = {accel};
  K_Accel = P * ~H_Accel * (H_Accel * P * ~H_Accel + R_Accel).Inverse();
  X = X + K_Accel * (Z_Accel - H_Accel * X);
  P = (I - K_Accel * H_Accel) * P * (~(I - K_Accel * H_Accel)) + K_Accel * R_Accel * ~K_Accel;
  setDataVariables();
}

void updateBaro(float altitude)
{
  predict();
  Z_Baro = {altitude};
  PBaro = H_Baro * P * ~H_Baro;
  K_Baro = P * ~H_Baro * (H_Baro * P * ~H_Baro + R_Baro).Inverse();

  X = X + K_Baro * (Z_Baro - H_Baro * X);
  P = (I - K_Baro * H_Baro) * P * (~(I - K_Baro * H_Baro)) + K_Baro * R_Baro * ~K_Baro;
  setDataVariables();
}

void updateGPS(float altitude)
{
  predict();
  Z_GPS = {altitude};
  K_GPS = P * ~H_GPS * (H_GPS * P * ~H_GPS + R_GPS).Inverse();
  X = X + K_GPS * (Z_GPS - H_GPS * X);
  P = (I - K_GPS * H_GPS) * P * (~(I - K_GPS * H_GPS)) + K_GPS * R_GPS * ~K_GPS;
  setDataVariables();
}

void setDataVariables()
{
  data.kal_X_pos = X(0, 0);
  data.kal_X_vel = X(1, 0);
  data.kal_X_accel = X(2, 0);

  data.kal_X_posP = P(0, 0);
  data.kal_X_velP = P(1, 1);
  data.kal_X_accelP = P(2, 2);
  data.p_baro = PBaro(0, 0);
}

float getKalmanPosition()
{
  return X(0, 0);
}

float getKalmanVelocity()
{
  return X(1, 0);
}

float getKalmanAcceleration()
{
  return X(2, 0);
}
