#include "Nav.h"

Bmi088Accel accel(Wire, 0x18);
Bmi088Gyro gyro(Wire, 0x68);

LSM9DS1 TVC_IMU;

float accel_raw[3] = {0, 0, 0};
float g_bias[3] = {0, 0, 0};

// Quaternion Stuff
float q_body_mag = 0;
float q_gyro[4] = {0, 0, 0, 0};
float q[4] = {1, 0, 0, 0};
float q_body[4] = {1, 0, 0, 0};
float q_grad[4] = {0, 0, 0, 0};
float omega[3] = {0, 0, 0};
float theta;

float ypr[3] = {0, 0, 0};

bool first_gyro_reading = true;
unsigned long gyro_current_time = 0;
unsigned long gyro_past_time = 0;
float gyro_dt = 0;

Quaternion localAccelQuat;
Quaternion worldAccelQuat;
Quaternion orientation(1, 0, 0, 0);

float oriBiases[4] = {0, 0, 0, 0};

Quaternion yawBiasQuaternion;
Quaternion pitchBiasQuaternion;

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

void getGyroBiases()
{
    int count = 0;
    const int averageAmount = GYRO_BIAS_COUNT;
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

void initTVCIMU()
{
    if (TVC_IMU.begin() == false) // with no arguments, this uses default addresses (AG:0x6B, M:0x1E) and i2c port (Wire).
    {
        while (1)
        {
            Serial.println("Failed to communicate with LSM9DS1.");
            Serial.println("Double-check wiring.");
            Serial.println("Default settings in this sketch will "
                           "work for an out of the box LSM9DS1 "
                           "Breakout, but may need to be modified "
                           "if the board jumpers are.");
        };
    }
}

void getTVCIMUAccel()
{

    if (TVC_IMU.accelAvailable())
    {

        TVC_IMU.readAccel();
        data.tvc_ax = TVC_IMU.calcAccel(TVC_IMU.ax) * 9.81;
        data.tvc_ay = TVC_IMU.calcAccel(TVC_IMU.ay) * 9.81;
        data.tvc_az = TVC_IMU.calcAccel(TVC_IMU.az) * 9.81;
    }
}

void getTVCAttitude()
{
    data.tvc_yaw = atan2(TVC_IMU.ay, TVC_IMU.az) * RAD_TO_DEG + 90.0;
    data.tvc_pitch = atan2(-TVC_IMU.ax, sqrt(TVC_IMU.ay * TVC_IMU.ay + TVC_IMU.az * TVC_IMU.az)) * RAD_TO_DEG;
}

bool initNav()
{
    initAltimeter();

    if (ENABLE_TVC_IMU)
    {
        initTVCIMU();
    };

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
    getGyroBiases();
    // g_bias[0] = 0.004;
    // g_bias[1] = -0.0009;
    // g_bias[2] = 0.00001;
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

float worldAxBias = 0.0f;
float worldAyBias = 0.0f;
float worldAzBias = 0.0f;

float worldAxBiasTemp = 0.0f;
float worldAyBiasTemp = 0.0f;
float worldAzBiasTemp = 0.0f;

void getWorldABiases()
{
    int i = 0;
    while (i < WORLD_ACCEL_BIAS_COUNT)
    {
        i += 1;
        delay(10);
        getYPR();
        worldAxBiasTemp += data.worldAx;
        worldAyBiasTemp += data.worldAy;
        worldAzBiasTemp += data.worldAz;
    }
    worldAxBias = worldAxBiasTemp / float(WORLD_ACCEL_BIAS_COUNT);
    worldAyBias = worldAyBiasTemp / float(WORLD_ACCEL_BIAS_COUNT);
    worldAzBias = worldAzBiasTemp / float(WORLD_ACCEL_BIAS_COUNT);
    data.worldAxBias = worldAxBias;
    data.worldAyBias = worldAyBias;
    data.worldAzBias = worldAzBias;
}

float axAve = 0;
float ayAve = 0;
float azAve = 0;

int i = 0;

void getInitYawAndPitchBiases()
{
    const int accelAveCount = YAW_PITCH_BIAS_COUNT;
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

    Serial.print(" NAV BIAS ");
    Serial.print(" PITCH: ");
    Serial.print(data.pitchBias);
    Serial.print(" YAW: ");
    Serial.println(data.yawBias);

    // Still figuring out how to get world accel rotated properly based on biases
    // yawBiasQuaternion = from_axis_angle(data.yawBias * DEG_TO_RAD, 0, 0, 1);
    // pitchBiasQuaternion = from_axis_angle(data.pitchBias * DEG_TO_RAD, 0, 1, 0);
}

unsigned long ori_bias_current_time = 0;
bool ori_bias_first_gyro_reading = true;
unsigned long ori_bias_gyro_past_time = 0;
float ori_bias_gyro_dt = 0.0;

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
        float mag = sin(theta / 2.0);
        q_gyro[0] = cos(theta / 2);
        q_gyro[1] = -(omega[0] / q_body_mag * mag);
        q_gyro[2] = -(omega[1] / q_body_mag * mag);
        q_gyro[3] = -(omega[2] / q_body_mag * mag);

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
        orientation.rotate(Quaternion(0.0, 0.0, data.yawBias, data.pitchBias));
        // Leave these out still figuring out world accel based on biases
        // orientation = pitchBiasQuaternion.rotate(orientation);
        // orientation = yawBiasQuaternion.rotate(orientation);
        localAccelQuat = Quaternion(0.0, data.ax, data.ay, data.az);
        worldAccelQuat = orientation.rotate(localAccelQuat);
        data.worldAx = worldAccelQuat.b - data.worldAxBias;
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

float getMovingAverageWorldXAccel(float worldXAccel)
{
    const static int WINDOW_SIZE = 20;
    static int INDEX = 0;
    static float SUM = 0;
    static float READINGS[WINDOW_SIZE];
    float AVERAGED = 0;
    SUM -= READINGS[INDEX];
    READINGS[INDEX] = worldXAccel;
    SUM += worldXAccel;
    INDEX = (INDEX + 1) % WINDOW_SIZE;
    if (INDEX < 19)
    {
        return 0.0;
    }

    AVERAGED = SUM / WINDOW_SIZE;

    return AVERAGED;
}