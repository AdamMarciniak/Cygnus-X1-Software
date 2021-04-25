#pragma once
#include "BasicLinearAlgebra.h"

class KalmanH
{
public:
  KalmanH();
  float getPosition();
  float getVelocity();
  float getAcceleration();
  float getBias();
  void update(float val);
  void zeroKalman();

private:
  BLA::Matrix<4, 4> Q;
  BLA::Matrix<1, 1> R;
  BLA::Matrix<4, 1> X;
  BLA::Matrix<4, 4> P;
  BLA::Matrix<4, 4> I;
  BLA::Matrix<1, 4> H;
  BLA::Matrix<4, 4> F;
  BLA::Matrix<1, 1> Z;
  BLA::Matrix<4, 1> K;
  void predict();
  unsigned long currentTime = 0;
  unsigned long prevTime = 0;
  float delT = 0.0f;
  bool isFirstStep = true;
};
