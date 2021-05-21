#include "BNOIMU.h"

BNOIMU::BNOIMU()
{
  Quaternion orientation(1, 0, 0, 0);
  zeroGyroscope();
}

// Quaternion Stuff

void BNOIMU::zeroGyroscope()
{
  q_gyro[0] = 0;
  q_gyro[1] = 0;
  q_gyro[2] = 0;
  q_gyro[3] = 0;
  q[0] = 1;
  q[1] = 0;
  q[2] = 0;
  q[3] = 0;
  q_body[0] = 1;
  q_body[1] = 0;
  q_body[2] = 0;
  q_body[3] = 0;
  q_grad[0] = 0;
  q_grad[1] = 0;
  q_grad[2] = 0;
  q_grad[3] = 0;
  omega[0] = 0;
  omega[1] = 0;
  omega[2] = 0;
  ypr[0] = 0;
  ypr[1] = 0;
  ypr[2] = 0;
}

void BNOIMU::initBNO()
{
  if (!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while (1)
      ;
  }
  zeroGyroscope();
  getGyroBiases();
  zeroGyroscope();
  getInitYawAndPitchBiases();
  getWorldABiases();
  zeroGyroscope();
}

void BNOIMU::getBNOData()
{

  bno.getEvent(&angVelocityData, Adafruit_BNO055::VECTOR_GYROSCOPE);
  bno.getEvent(&accelerometerData, Adafruit_BNO055::VECTOR_ACCELEROMETER);
  bno.getEvent(&magnetometerData, Adafruit_BNO055::VECTOR_MAGNETOMETER);

  data.bno_ax = accelerometerData.acceleration.x;
  data.bno_ay = accelerometerData.acceleration.y;
  data.bno_az = accelerometerData.acceleration.z;

  data.bno_gx = angVelocityData.gyro.x;
  data.bno_gy = angVelocityData.gyro.y;
  data.bno_gz = angVelocityData.gyro.z;

  data.bno_magx = magnetometerData.magnetic.x;
  data.bno_magy = magnetometerData.magnetic.y;
  data.bno_magz = magnetometerData.magnetic.z;

  getYPR();
}

void BNOIMU::getGyroBiases()
{
  int count = 0;
  const int averageAmount = GYRO_BIAS_COUNT;
  while (count < averageAmount)
  {
    bno.getEvent(&angVelocityData, Adafruit_BNO055::VECTOR_GYROSCOPE);

    g_bias[0] += accelerometerData.acceleration.x;
    g_bias[1] += accelerometerData.acceleration.y;
    g_bias[2] += accelerometerData.acceleration.z;
    count += 1;
    Serial.print(".");
    delay(10);
  }
  Serial.println();
  g_bias[0] /= (float)averageAmount;
  g_bias[1] /= (float)averageAmount;
  g_bias[2] /= (float)averageAmount;

  Serial.print(g_bias[0], 8);
  Serial.print(" ");
  Serial.print(g_bias[1], 8);
  Serial.print(" ");
  Serial.print(g_bias[2], 8);
  Serial.println(" ");
}

void BNOIMU::getWorldABiases()
{
  int i = 0;
  while (i < WORLD_ACCEL_BIAS_COUNT)
  {
    i += 1;
    delay(10);
    getYPR();
    worldAxBiasTemp += data.bno_worldAx;
    worldAyBiasTemp += data.bno_worldAy;
    worldAzBiasTemp += data.bno_worldAz;
  }
  worldAxBias = worldAxBiasTemp / float(WORLD_ACCEL_BIAS_COUNT);
  worldAyBias = worldAyBiasTemp / float(WORLD_ACCEL_BIAS_COUNT);
  worldAzBias = worldAzBiasTemp / float(WORLD_ACCEL_BIAS_COUNT);
}

void BNOIMU::getInitYawAndPitchBiases()
{
  const int accelAveCount = YAW_PITCH_BIAS_COUNT;
  while (i < accelAveCount)
  {

    bno.getEvent(&accelerometerData, Adafruit_BNO055::VECTOR_ACCELEROMETER);

    axAve += accelerometerData.acceleration.x;
    ayAve += accelerometerData.acceleration.y;
    azAve += accelerometerData.acceleration.z;
    delay(10);
    i += 1;
  }
  axAve /= float(accelAveCount);
  ayAve /= float(accelAveCount);
  azAve /= float(accelAveCount);

  pitchBias = atan2(-azAve, (sqrt(sq(axAve) + sq(ayAve)))) * RAD_TO_DEG;
  yawBias = atan2(ayAve, (sqrt(sq(axAve) + sq(azAve)))) * RAD_TO_DEG;
}

void BNOIMU::getYPR()
{
  gyro_current_time = micros();

  bno.getEvent(&angVelocityData, Adafruit_BNO055::VECTOR_GYROSCOPE);

  if (!first_gyro_reading)
  {

    omega[2] = (angVelocityData.gyro.x - g_bias[0]);
    omega[1] = -(angVelocityData.gyro.y - g_bias[1]);
    omega[0] = (angVelocityData.gyro.z - g_bias[2]);

    data.bno_gx = omega[2];
    data.bno_gy = omega[1];
    data.bno_gz = omega[0];

    q_body_mag = sqrt(sq(omega[0]) + sq(omega[1]) + sq(omega[2]));

    gyro_dt = ((gyro_current_time - gyro_past_time) / 1000000.0);

    theta = q_body_mag * gyro_dt;
    float mag = q_body_mag * sin(theta / 2.0);
    q_gyro[0] = cos(theta / 2);
    q_gyro[1] = -(omega[0] / mag);
    q_gyro[2] = -(omega[1] / mag);
    q_gyro[3] = -(omega[2] / mag);

    

    q[0] = q_body[0];
    q[1] = q_body[1];
    q[2] = q_body[2];
    q[3] = q_body[3];

    q_body[0] = q_gyro[0] * q[0] - q_gyro[1] * q[1] - q_gyro[2] * q[2] - q_gyro[3] * q[3];
    q_body[1] = q_gyro[0] * q[1] + q_gyro[1] * q[0] + q_gyro[2] * q[3] - q_gyro[3] * q[2];
    q_body[2] = q_gyro[0] * q[2] - q_gyro[1] * q[3] + q_gyro[2] * q[0] + q_gyro[3] * q[1];
    q_body[3] = q_gyro[0] * q[3] + q_gyro[1] * q[2] - q_gyro[2] * q[1] + q_gyro[3] * q[0];

    // // For getting world frame acceleration
    // float norm = sqrtf(sq(omega[0]) + sq(omega[1]) + sq(omega[2]));
    // norm = copysignf(max(abs(norm), 1e-9), norm); // NO DIVIDE BY 0
    // orientation *= from_axis_angle(gyro_dt * norm, omega[0] / norm, omega[1] / norm, omega[2] / norm);
    // orientation.rotate(Quaternion(0.0, 0.0, yawBias, pitchBias));
    // // Leave these out still figuring out world accel based on biases
    // // orientation = pitchBiasQuaternion.rotate(orientation);
    // // orientation = yawBiasQuaternion.rotate(orientation);
    // localAccelQuat = Quaternion(0.0, data.bno_ax, data.bno_ay, data.bno_az);
    // worldAccelQuat = orientation.rotate(localAccelQuat);
    // data.bno_worldAx = worldAccelQuat.b - worldAxBias;
    // data.bno_worldAy = worldAccelQuat.c - worldAyBias;
    // data.bno_worldAz = worldAccelQuat.d - worldAzBias;

    // quatToEuler();

    // data.bno_yaw = ypr[0] + yawBias;
    // data.bno_pitch = ypr[1] + pitchBias;
    // data.bno_roll = ypr[2];
  }
  first_gyro_reading = false;
  gyro_past_time = gyro_current_time;
}

void BNOIMU::quatToEuler()
{
  double sinr_cosp = 2.0 * (q_body[0] * q_body[1] + q_body[2] * q_body[3]);
  double cosr_cosp = 1.0 - 2.0 * (q_body[1] * q_body[1] + q_body[2] * q_body[2]);
  ypr[2] = atan2(sinr_cosp, cosr_cosp) * RAD_TO_DEG;
  double sinp = 2.0 * (q_body[0] * q_body[2] - q_body[1] * q_body[3]);
  if (sinp >= 1)
    ypr[1] = 90.0;
  else if (sinp <= -1.0)
    ypr[1] = -90.0;
  else
    ypr[1] = asin(sinp) * RAD_TO_DEG;

  double siny_cosp = 2.0 * (q_body[0] * q_body[3] + q_body[1] * q_body[2]);
  double cosy_cosp = 1.0 - 2.0 * (q_body[2] * q_body[2] + q_body[3] * q_body[3]);
  ypr[0] = atan2(siny_cosp, cosy_cosp) * RAD_TO_DEG;
}