#pragma once
#include "BasicLinearAlgebra.h"

class KalmanNew
{
public:
  KalmanNew();
  float getPosition();
  float getVelocity();
  float getGravity();
  float getBias();
  void updateBaro(float val);
  void updateGPS(float val);

  void zeroKalman();
  void predict(float accel);

private:
  BLA::Matrix<4, 4> Q;
  BLA::Matrix<1, 1> R_Baro;
  BLA::Matrix<1, 1> R_GPS;
  BLA::Matrix<4, 1> K_Baro;
  BLA::Matrix<4, 1> K_GPS;
  BLA::Matrix<1, 4> H_Baro;
  BLA::Matrix<1, 4> H_GPS;
  BLA::Matrix<1, 1> Z;
  BLA::Matrix<4, 1> X;
  BLA::Matrix<4, 4> P;
  BLA::Matrix<4, 4> I;
  BLA::Matrix<4, 4> F;
  BLA::Matrix<4, 1> A;

  unsigned long currentTime = 0;
  unsigned long prevTime = 0;
  float delT = 0.0f;
  bool isFirstStep = true;
};
