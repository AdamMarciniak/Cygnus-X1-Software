#ifndef STATES_H
#define STATES_H

enum RocketState
{
  USB_CONTROL,
  CONFIG,
  CALIBRATION,
  IDLE,
  IGNITION,
  POWERED_ASCENT,
  UNPOWERED_ASCENT,
  FREE_DESCENT,
  PARACHUTE_DESCENT,
  LANDED,
  DATA_SAVING,
  COMPLETE,
  ABORT,
  ERROR,
};

extern RocketState currentState;

extern void setState(RocketState newState);

#endif