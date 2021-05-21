#pragma once

#include "Quaternion.h"
#include "Wire.h"
#include "Data.h"
#include "./libraries/Adafruit_Sensor.h"
#include "./libraries/Adafruit_BNO055.h"
#include "./libraries/utility/imumaths.h"

class BNOIMU
{

public:
  BNOIMU();
  void getGyroBiases();
  void zeroGyroscope();
  void initBNO();
  void initNav();
  void getYPR();
  void getCurrentYawAndPitchFromAccel();
  void getInitYawAndPitchBiases();
  void getWorldABiases();
  void getBNOData();

private:
  float ypr[3] = {0, 0, 0};
  float q_body_mag = 0;
  float q_gyro[4] = {0, 0, 0, 0};
  float q[4] = {1, 0, 0, 0};
  float q_body[4] = {1, 0, 0, 0};
  float q_grad[4] = {0, 0, 0, 0};
  float omega[3] = {0, 0, 0};
  float theta = 0;
  int i = 0;

  bool first_gyro_reading = true;
  unsigned long gyro_current_time = 0;
  unsigned long gyro_past_time = 0;
  float gyro_dt = 0;

  Quaternion localAccelQuat;
  Quaternion worldAccelQuat;
  Quaternion orientation;

  float oriBiases[4] = {0, 0, 0, 0};

  Quaternion yawBiasQuaternion;
  Quaternion pitchBiasQuaternion;

  Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);

  sensors_event_t orientationData, angVelocityData, linearAccelData, magnetometerData, accelerometerData, gravityData;

  void quatToEuler();

  float worldAxBias = 0.0f;
  float worldAyBias = 0.0f;
  float worldAzBias = 0.0f;

  float worldAxBiasTemp = 0.0f;
  float worldAyBiasTemp = 0.0f;
  float worldAzBiasTemp = 0.0f;

  float axAve = 0;
  float ayAve = 0;
  float azAve = 0;

  float g_bias[3] = {0, 0, 0};
  float pitchBias = 0;
  float yawBias = 0;

  unsigned long ori_bias_current_time = 0;
  bool ori_bias_first_gyro_reading = true;
  unsigned long ori_bias_gyro_past_time = 0;
  float ori_bias_gyro_dt = 0.0;
  float magRoll = 0.0f;
};
