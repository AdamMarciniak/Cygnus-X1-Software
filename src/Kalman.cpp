#include "Kalman.h"

using namespace BLA;

BLA::Matrix<2, 2> Q = {0.00000003125, 0,
                       0, 0.0000125};

// Measured baro variance was 0.03251531
BLA::Matrix<1, 1> R_Baro = {0.09};

BLA::Matrix<2, 1> X = {
    0,
    0,

};

BLA::Matrix<2, 2> P = {1, 0,
                       0, 1};

BLA::Matrix<2, 2> I = {1, 0,
                       0, 1};

BLA::Matrix<1, 2> H_Baro = {1, 0};

BLA::Matrix<2, 2> F;
BLA::Matrix<2, 1> B;
BLA::Matrix<1, 1> U;
BLA::Matrix<1, 1> Z_Baro;
BLA::Matrix<2, 1> K_Baro;

void initKalman()
{
  zeroKalman();
}

void zeroKalman()
{
  X = {
      0,
      0,
  };

  P = {0.01, 0,
       0, 0.000000001};
}

unsigned long currentTime = 0;
unsigned long prevTime = 0;
float delT = 0.0f;

bool isFirstStep = true;

void predict(float accel)
{
  currentTime = micros();
  delT = (currentTime - prevTime) / 1000000.0f;
  data.loopTime = delT;
  prevTime = currentTime;

  if (!isFirstStep)
  {
    F = {
        1, delT,
        0, 1};

    B = {0.5 * delT * delT,
         delT};

    U = {accel};

    X = F * X + B * U;
    P = F * P * ~F + Q;
  }
  isFirstStep = false;
}

BLA::Matrix<1, 1> Innovation;

void updateBaro(float altitude)
{
  Z_Baro = {altitude};
  K_Baro = P * ~H_Baro * (H_Baro * P * ~H_Baro + R_Baro).Inverse();

  Innovation = (Z_Baro - H_Baro * X);
  X = X + K_Baro * Innovation;
  P = (I - K_Baro * H_Baro) * P * (~(I - K_Baro * H_Baro)) + K_Baro * R_Baro * ~K_Baro;
  setDataVariables();
}

void setDataVariables()
{
  data.kal_X_pos = X(0, 0);
  data.kal_X_vel = X(1, 0);
  data.kal_X_posP = P(0, 0);
  data.kal_X_velP = P(1, 1);
  data.kal_inno = Innovation(0, 0);
  data.kal_K_1 = K_Baro(0, 0);
  data.kal_K_2 = K_Baro(1, 0);
}
