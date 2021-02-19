#include "Utils.h"

long mapFloats(float x, float in_min, float in_max, float out_min, float out_max)
{
  long result = (long)((x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min);
  if (result > out_max)
  {
    return (long)out_max;
  }
  if (result < out_min)
  {
    return (long)out_min;
  }
  return result;
}