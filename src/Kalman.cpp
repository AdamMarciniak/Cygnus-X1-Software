#include "Kalman.h"
#include "BasicLinearAlgebra.h"
#include "Data.h"
using namespace BLA;

BLA::Matrix<3, 3> Q = {0.001, 0, 0,
                       0, 0.001, 0,
                       0, 0, 0.001};

BLA::Matrix<1, 1> R_Accel = {0.105434};

// Measured baro variance was 0.03251531
BLA::Matrix<1, 1> R_Baro = {0.7};

BLA::Matrix<3, 1> X = {
    0,
    0,
    0,
};

BLA::Matrix<3, 1> X_Prev = {
    0,
    0,
    0,
};

BLA::Matrix<3, 3> P = {1, 0, 0,
                       0, 1, 0,
                       0, 0, 1};

BLA::Matrix<3, 3> P_Prev = {1, 0, 0,
                            0, 1, 0,
                            0, 0, 1};

BLA::Matrix<1, 3> H_Baro = {1, 0, 0};

BLA::Matrix<1, 3> H_Accel = {0, 0, 1};

BLA::Matrix<3, 3> F;

BLA::Matrix<1, 1> Z_Accel;
BLA::Matrix<1, 1> Z_Baro;

BLA::Matrix<3, 1> K_Accel;
BLA::Matrix<3, 1> K_Baro;

void zeroKalman()
{
  X = {
      0,
      0,
      0,
  };

  P = {0.1, 0, 0,
       0, 0.1, 0,
       0, 0, 0.1};

  P_Prev = {0.1, 0, 0,
            0, 0.1, 0,
            0, 0, 0.1};

  X_Prev = {
      0,
      0,
      0,
  };
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
  P = P - K_Accel * H_Accel * P;
}

void updateBaro(float altitude)
{
  predict();
  Z_Baro = {altitude};
  K_Baro = P * ~H_Baro * (H_Baro * P * ~H_Baro + R_Baro).Inverse();
  X = X + K_Baro * (Z_Baro - H_Baro * X);
  P = P - K_Baro * H_Baro * P;
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