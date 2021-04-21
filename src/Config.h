#pragma once

#define Y_SETPOINT 0.0f
#define Z_SETPOINT 0.0f

#define Y_KP 0.5
#define Y_KI 0.06
#define Y_KD 0.14

#define Z_KP 0.5
#define Z_KI 0.06
#define Z_KD 0.14

#define LOW_VOLT_THRESHOLD 10.5f

#define DATA_SAMPLE_RATE 100 // hz
#define DATA_SAMPLE_TOTAL_TIME 20 // seconds

#define BATTERY_VOLTAGE_MIN 11.5 //volts
#define NAV_RATE 10 //ms

#define PARACHUTE_SERVO_DEPLOY 53
#define PARACHUTE_SERVO_INIT 97

#define TELEMETRY_RATE 50     // ms

#define SERVO_RANGE 22
#define TVC_TO_SERVO_SCALE 4
#define Y_CENTER 91
#define Z_CENTER 102


#define PYRO_CONTINUITY_THRESHOLD 200;
#define FIRE_ON_TIME 1000     // ms

#define LAUNCH_ACCEL_THRESHOLD 2.0f // m/s^2

#define PARACHUTE_ALTITUDE_THRESHOLD 40.0f      // meters
#define ABORT_ANGLE_THRESHOLD 30.0f             // degrees
#define ACCEL_UNPOWERED_THRESHOLD 2.0f          //m/s^2
#define FIRE_ON_TIME 1000                       // ms
#define FIRE_TO_PID_DELAY 500                   //ms

#define PARACHUTE_EJECT_SAFETY_TIME 10000 // ms

#define LANDING_DETECT_DELAY 10000 //ms