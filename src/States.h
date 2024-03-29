#pragma once

enum State
{
  INITIALIZING,
  IDLE,
  LAUNCH_COMMANDED,
  POWERED_ASCENT,
  UNPOWERED_ASCENT,
  FREE_DESCENT,
  PARACHUTE_DESCENT,
  LANDED,
  ERROR,
  ABORT,
  TEST,
  GPS_BIAS_GATHER
};
