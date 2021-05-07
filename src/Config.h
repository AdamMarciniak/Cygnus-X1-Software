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

#define ENABLE_PARACHUTE true
#define ENABLE_ANGLE_CHECK false
#define ENGAGE_PYRO false
#define DO_EUI false
#define DO_GPS true

#define SELF_FIRE false

#define YAW_PITCH_BIAS_COUNT 50
#define WORLD_ACCEL_BIAS_COUNT 500
#define GYRO_BIAS_COUNT 100

#define DATA_SAMPLE_RATE 200      // hz
#define DATA_SAMPLE_TOTAL_TIME 15 // seconds

#define LANDING_DETECT_DELAY 15000  //ms
#define ABORT_TO_LANDED_DELAY 20000 //ms

#define MOTOR_FAIL_DELAY 10000 // ms

#define Y_SETPOINT 0.0f //deg
#define Z_SETPOINT 0.0f //deg

#define KPF15 0.3f
#define KIF15 0.1f
#define KDF15 0.15f

#define KPE12 0.5f
#define KIE12 0.06f
#define KDE12 0.14f

#define Y_KP KPE12
#define Y_KI KIE12
#define Y_KD KDE12

#define Z_KP KPE12
#define Z_KI KIE12
#define Z_KD KDE12

#define BATTERY_VOLTAGE_MIN 11.3f //volts
#define NAV_RATE 5                //ms

#define PARACHUTE_SERVO_DEPLOY 50
#define PARACHUTE_SERVO_INIT 97

#define TELEMETRY_RATE 20 // ms

#define SERVO_RANGE 26
#define Y_CENTER 91
#define Z_CENTER 102

#define PYRO_CONTINUITY_THRESHOLD 200;
#define FIRE_ON_TIME 1000 // ms

#define LAUNCH_ACCEL_THRESHOLD 2.0f // m/s^2

#define PARACHUTE_ALTITUDE_THRESHOLD 40.0f // meters
#define ABORT_ANGLE_THRESHOLD 40.0f        // degrees
#define ACCEL_UNPOWERED_THRESHOLD 1.5f     //m/s^2
#define FIRE_ON_TIME 1000                  // ms

#define FIRE_TO_PID_DELAY 500 //ms

// This gets counted when launch commanded.
// Parachute will eject if this takes too long
// Probably useless
#define PARACHUTE_EJECT_SAFETY_TIME 100000 // ms
