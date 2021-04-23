#pragma once

#define ENABLE_PARACHUTE false
#define ENABLE_ANGLE_CHECK false

#define Y_SETPOINT 0.0f //deg
#define Z_SETPOINT 0.0f //deg

#define KP 0.5
#define KI 0.06
#define KD 0.14

#define Y_KP KP
#define Y_KI KI
#define Y_KD KD

#define Z_KP KP
#define Z_KI KI
#define Z_KD KD

#define ENGAGE_PYRO false

#define DO_EUI false

#define DATA_SAMPLE_RATE 200      // hz
#define DATA_SAMPLE_TOTAL_TIME 50 // seconds

#define BATTERY_VOLTAGE_MIN 11.0 //volts
#define NAV_RATE 10              //ms

#define PARACHUTE_SERVO_DEPLOY 53
#define PARACHUTE_SERVO_INIT 97

#define TELEMETRY_RATE 10 // ms

#define SERVO_RANGE 24
#define TVC_TO_SERVO_SCALE 4
#define Y_CENTER 91
#define Z_CENTER 102

#define PYRO_CONTINUITY_THRESHOLD 200;
#define FIRE_ON_TIME 1000 // ms

#define LAUNCH_ACCEL_THRESHOLD 2.0f // m/s^2

#define PARACHUTE_ALTITUDE_THRESHOLD 40.0f // meters
#define ABORT_ANGLE_THRESHOLD 35.0f        // degrees
#define ACCEL_UNPOWERED_THRESHOLD 2.0f     //m/s^2
#define FIRE_ON_TIME 1000                  // ms

// Unused now
#define FIRE_TO_PID_DELAY 500 //ms

// This gets counted when launch commanded.
// Parachute will eject if this takes too long
// Probably useless
#define PARACHUTE_EJECT_SAFETY_TIME 100000 // ms

#define LANDING_DETECT_DELAY 2000 //ms