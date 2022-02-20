#pragma once

// Options for flight
// test mode false
// enable parachute true
// Enable angle check true
// Engage pyro true
// Do EUI false
// SELF_FIRE true if firing with BTLE
// DO_GPS true (can be always true)

#define IS_TEST_MODE false
#define IS_DUMP_MODE false

#define IS_DUAL_STAGE false
// Altitude where below this, the angle limit abort gets fired.
#define ANGLE_ABORT_MAX_ALT 40.0

#define ENABLE_PARACHUTE true
#define ENABLE_ANGLE_CHECK true
#define ABORT_ANGLE_THRESHOLD 45.0f // degrees

#define SELF_FIRE true

#define ENGAGE_PYRO true
#define DO_EUI false
#define DO_GPS false

#define PARACHUTE_ALTITUDE_THRESHOLD 60.0f // meters

#define ENABLE_TVC_IMU false
#define INIT_BNO false

//PITCH OVR INSTRUCTIONS
// POINT BOARD FRONT IN DIRECTION OF PITCH OVER (AWAY FROM PEOPLE)
// USE Z AXIS TRUE
// USE NEGATIVE VALUE FOR PITCH ANGLE

#define ENABLE_PITCH_OVER_Y false
#define ENABLE_PITCH_OVER_Z false

#define PITCH_OVER_ANGLE -15    //deg
#define PITCH_OVER_START 1000   //ms
#define PITCH_OVER_DURATION 500 //ms

#define DATA_SAMPLE_RATE 200      // hz
#define DATA_SAMPLE_TOTAL_TIME 45 // seconds

#define LANDING_DETECT_DELAY 15000  //ms
#define ABORT_TO_LANDED_DELAY 25000 //ms

#define MOTOR_FAIL_DELAY 10000 // ms

#define Y_SETPOINT 0.0f //deg
#define Z_SETPOINT 0.0f //deg

#define KPF15 0.87f
#define KIF15 0.22f
#define KDF15 0.28f

#define KPE12 0.6f
#define KIE12 0.02f
#define KDE12 0.2f

#define Y_KP KPF15
#define Y_KI KIF15
#define Y_KD KDF15

#define Z_KP Y_KP
#define Z_KI Y_KI
#define Z_KD Y_KD

#define YAW_PITCH_BIAS_COUNT 50
#define WORLD_ACCEL_BIAS_COUNT 500
#define GYRO_BIAS_COUNT 100

#define BATTERY_VOLTAGE_MIN 11.2f //volts
#define NAV_RATE 5                //ms

#define PARACHUTE_SERVO_DEPLOY 50
#define PARACHUTE_SERVO_INIT 97

#define TELEMETRY_RATE 20 // ms

#define SERVO_RANGE 21
#define Y_CENTER 91
#define Z_CENTER 102

#define PYRO_CONTINUITY_THRESHOLD 200;
#define FIRE_ON_TIME 1000 // ms

#define LAUNCH_ACCEL_THRESHOLD 2.0f // m/s^2

#define ACCEL_UNPOWERED_THRESHOLD 4.0f //m/s^2

#define FIRE_TO_PID_DELAY 350 //ms

// This gets counted when launch commanded.
// Parachute will eject if this takes too long
// Probably useless
#define PARACHUTE_EJECT_SAFETY_TIME 100000 // ms
