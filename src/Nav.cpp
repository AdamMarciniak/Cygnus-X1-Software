#include "Nav.h"

Bmi088Accel accel(Wire, 0x18);
Bmi088Gyro gyro(Wire, 0x68);

float accel_raw[3] = {0, 0, 0};
float accel_raw_prev[3] = {0, 0, 0};
float gyro_raw[3] = {0, 0, 0}; // rad/sec
float g_bias[3] = {0, 0, 0};

// Quaternion Stuff
float q_body_mag = 0;
float q_gyro[4] = {0, 0, 0, 0};
float q[4] = {1, 0, 0, 0};
float q_body[4] = {1, 0, 0, 0};
float q_grad[4] = {0, 0, 0, 0};
float omega[3] = {0, 0, 0};
float theta;

bool first_gyro_reading = true;
unsigned long gyro_current_time = 0;
unsigned long gyro_past_time = 0;
float gyro_dt = 0;

// Yaw pitch roll of rocket
float ypr[3] = {0, 0, 0};

float vel_local[3] = {0, 0, 0};
bool firstAccelReading = true;
float accel_dt = 0;
unsigned long accel_current_time = 0;
unsigned long accel_past_time = 0;

Quaternion localAccelQuat;
Quaternion worldAccelQuat;
Quaternion orientation(1, 0, 0, 0);

Quaternion yawBiasQuaternion;
Quaternion pitchBiasQuaternion;
float worldAccelArray[4] = {0, 0, 0, 0};
float worldAccelAngles[3] = {0, 0, 0};

void zeroGyroscope()
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

void quatToEuler(float *qBody, float *ypr);

void getGyroBiases()
{
    int count = 0;
    const int averageAmount = 500;
    while (count < averageAmount)
    {
        gyro.readSensor();
        g_bias[0] += gyro.getGyroX_rads();
        g_bias[1] += gyro.getGyroY_rads();
        g_bias[2] += gyro.getGyroZ_rads();
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

bool initNav()
{

    initAltimeter();
    if (accel.begin() < 0)
    {
        return 0;
    }
    if (gyro.begin() < 0)
    {
        return 0;
    }
    Serial.println("Getting Gyro Biases...");
    zeroGyroscope();
    //getGyroBiases();
    g_bias[0] = 0.004;
    g_bias[1] = -0.0009;
    g_bias[2] = 0.00001;
    zeroGyroscope();
    getInitYawAndPitchBiases();
    getWorldABiases();
    return 1;
}

void getAccel()
{
    accel.readSensor();
    accel_raw[0] = accel.getAccelX_mss();
    accel_raw[1] = accel.getAccelY_mss();
    accel_raw[2] = accel.getAccelZ_mss();

    data.ax = accel_raw[0];
    data.ay = accel_raw[1];
    data.az = accel_raw[2];
}

void measureNav()
{
    //getAccel();
    handleAltimeter();

    if (isNewAltimeterData())
    {
        getAltitude();
    }
    // updateAccel(data.worldAx);
}

float worldAxBias = 0.0f;
float worldAyBias = 0.0f;
float worldAzBias = 0.0f;

float worldAxBiasTemp = 0.0f;
float worldAyBiasTemp = 0.0f;
float worldAzBiasTemp = 0.0f;

void getWorldABiases()
{
    int i = 0;
    const int count = 100;
    while (i < count)
    {
        i += 1;
        delay(10);
        getYPR();
        worldAxBiasTemp += data.worldAx;
        worldAyBiasTemp += data.worldAy;
        worldAzBiasTemp += data.worldAz;
    }
    worldAxBias = worldAxBiasTemp / 100.0;
    worldAyBias = worldAyBiasTemp / 100.0;
    worldAzBias = worldAzBiasTemp / 100.0;
}

float axAve = 0;
float ayAve = 0;
float azAve = 0;
const int accelAveCount = 50;
int i = 0;

void getInitYawAndPitchBiases()
{

    while (i < accelAveCount)
    {
        getAccel();
        axAve += data.ax;
        ayAve += data.ay;
        azAve += data.az;
        delay(10);
        i += 1;
    }
    axAve /= float(accelAveCount);
    ayAve /= float(accelAveCount);
    azAve /= float(accelAveCount);

    data.pitchBias = atan2(-azAve, (sqrt(sq(axAve) + sq(ayAve)))) * RAD_TO_DEG;
    data.yawBias = atan2(ayAve, (sqrt(sq(axAve) + sq(azAve)))) * RAD_TO_DEG;

    // Still figuring out how to get world accel rotated properly based on biases
    // yawBiasQuaternion = from_axis_angle(data.yawBias * DEG_TO_RAD, 0, 0, 1);
    // pitchBiasQuaternion = from_axis_angle(data.pitchBias * DEG_TO_RAD, 0, 1, 0);
}

void getCurrentYawAndPitchFromAccel()
{
    data.pitchBias = atan2(-data.az, (sqrt(sq(data.ax) + sq(data.ay)))) * RAD_TO_DEG;
    data.yawBias = atan2(data.ay, (sqrt(sq(data.ax) + sq(data.az)))) * RAD_TO_DEG;
}

void getYPR()
{
    gyro_current_time = micros();
    gyro.readSensor();

    if (!first_gyro_reading)
    {

        omega[0] = gyro.getGyroX_rads() - g_bias[0];
        omega[1] = gyro.getGyroY_rads() - g_bias[1];
        omega[2] = gyro.getGyroZ_rads() - g_bias[2];

        data.gx = omega[0];
        data.gy = omega[1];
        data.gz = omega[2];

        q_body_mag = sqrt(sq(omega[0]) + sq(omega[1]) + sq(omega[2]));
        gyro_dt = ((gyro_current_time - gyro_past_time) / 1000000.0);

        theta = q_body_mag * gyro_dt;
        q_gyro[0] = cos(theta / 2);
        q_gyro[1] = -(omega[0] / q_body_mag * sin(theta / 2));
        q_gyro[2] = -(omega[1] / q_body_mag * sin(theta / 2));
        q_gyro[3] = -(omega[2] / q_body_mag * sin(theta / 2));

        q[0] = q_body[0];
        q[1] = q_body[1];
        q[2] = q_body[2];
        q[3] = q_body[3];

        q_body[0] = q_gyro[0] * q[0] - q_gyro[1] * q[1] - q_gyro[2] * q[2] - q_gyro[3] * q[3];
        q_body[1] = q_gyro[0] * q[1] + q_gyro[1] * q[0] + q_gyro[2] * q[3] - q_gyro[3] * q[2];
        q_body[2] = q_gyro[0] * q[2] - q_gyro[1] * q[3] + q_gyro[2] * q[0] + q_gyro[3] * q[1];
        q_body[3] = q_gyro[0] * q[3] + q_gyro[1] * q[2] - q_gyro[2] * q[1] + q_gyro[3] * q[0];

        // For getting world frame acceleration
        float norm = sqrtf(sq(omega[0]) + sq(omega[1]) + sq(omega[2]));
        norm = copysignf(max(abs(norm), 1e-9), norm); // NO DIVIDE BY 0
        orientation *= from_axis_angle(gyro_dt * norm, omega[0] / norm, omega[1] / norm, omega[2] / norm);
        // Leave these out still figuring out world accel based on biases
        // orientation = pitchBiasQuaternion.rotate(orientation);
        // orientation = yawBiasQuaternion.rotate(orientation);
        localAccelQuat = Quaternion(0, data.ax, data.ay, data.az);
        worldAccelQuat = orientation.rotate(localAccelQuat);
        data.worldAx = worldAccelQuat.b - worldAxBias;
        data.worldAy = worldAccelQuat.c - worldAyBias;
        data.worldAz = worldAccelQuat.d - worldAzBias;

        quatToEuler(q_body, ypr);
        data.yaw = ypr[0] + data.yawBias;
        data.pitch = ypr[1] + data.pitchBias;
        data.roll = ypr[2];
    }
    first_gyro_reading = false;
    gyro_past_time = gyro_current_time;
}

void quatToEuler(float *qBody, float *ypr)
{
    double sinr_cosp = 2 * (q_body[0] * q_body[1] + q_body[2] * q_body[3]);
    double cosr_cosp = 1 - 2 * (q_body[1] * q_body[1] + q_body[2] * q_body[2]);
    ypr[2] = atan2(sinr_cosp, cosr_cosp) * RAD_TO_DEG;
    double sinp = 2 * (q_body[0] * q_body[2] - q_body[1] * q_body[3]);
    if (sinp >= 1)
        ypr[1] = 90;
    else if (sinp <= -1)
        ypr[1] = -90;
    else
        ypr[1] = asin(sinp) * RAD_TO_DEG;

    double siny_cosp = 2 * (q_body[0] * q_body[3] + q_body[1] * q_body[2]);
    double cosy_cosp = 1 - 2 * (q_body[2] * q_body[2] + q_body[3] * q_body[3]);
    ypr[0] = atan2(siny_cosp, cosy_cosp) * RAD_TO_DEG;
}