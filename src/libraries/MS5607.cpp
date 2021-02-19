/*File   : MS5607.cpp
  Author : Amit Ate
  Email  : amit@uravulabs.com
  Company: Uravu Labs
*/

#include <math.h>
#include "MS5607.h"
#include <Wire.h>

unsigned long convElapsed = 0;

MS5607::MS5607(float *altitudeVariable)
{
  this->internal_altitude = altitudeVariable;
}

MS5607::MS5607(short address)
{
  this->MS5607_ADDR = address;
}

// Initialise coefficient by reading calibration data
char MS5607::begin()
{
  Wire1.begin();
  return (readCalibration());
}

char MS5607::resetDevice(void)
{
  Wire1.beginTransmission(MS5607_ADDR);
  Wire1.write(RESET);
  char error = Wire1.endTransmission();
  if (error == 0)
  {
    delay(3); // wait for internal register reload
    return (1);
  }
  else
  {
    return (0);
  }
}

// read calibration data from PROM
char MS5607::readCalibration()
{
  if (resetDevice() &&
      readUInt_16(PROM_READ + 2, C1) &&
      readUInt_16(PROM_READ + 4, C2) &&
      readUInt_16(PROM_READ + 6, C3) &&
      readUInt_16(PROM_READ + 8, C4) &&
      readUInt_16(PROM_READ + 10, C5) &&
      readUInt_16(PROM_READ + 12, C6))
  {
    return (1);
  }
  else
  {
    return (0);
  }
}

// convert raw data into unsigned int
char MS5607::readUInt_16(char address, unsigned int &value)
{
  unsigned char data[2]; //4bit
  data[0] = address;
  if (readBytes(data, 2))
  {
    value = (((unsigned int)data[0] * (1 << 8)) | (unsigned int)data[1]);
    return (1);
  }
  value = 0;
  return (0);
}

// read number of bytes over i2c
char MS5607::readBytes(unsigned char *values, char length)
{
  char x;

  Wire1.beginTransmission(MS5607_ADDR);
  Wire1.write(values[0]);

  char error = Wire1.endTransmission();
  if (error == 0)
  {
    Wire1.requestFrom(MS5607_ADDR, length);
    while (!Wire1.available())
      ; // wait until bytes are ready
    for (x = 0; x < length; x++)
    {
      values[x] = Wire1.read();
    }
    return (1);
  }
  return (0);
}

// send command to start measurement
char MS5607::startMeasurment(void)
{
  Wire1.beginTransmission(MS5607_ADDR);
  Wire1.write(R_ADC);
  char error = Wire1.endTransmission();
  if (error == 0)
  {
    convElapsed = millis();
    return (1);
  }
  else
  {
    return (0);
  }
}

// send command to start conversion of temp/pressure
char MS5607::startConversion(char CMD)
{
  Wire1.beginTransmission(MS5607_ADDR);
  Wire1.write(CMD);
  char error = Wire1.endTransmission();
  if (error == 0)
  {
    convElapsed = millis();
    return (1);
  }
  else
  {
    return (0);
  }
}

int MS5607::handleAltimeter(void)

{
  if (state == 0)
  {
    startConversion(CONV_D1);
    state = 1;
  }

  if (state == 1 && (millis() - convElapsed) > 10)
  {
    startMeasurment();
    state = 2;
  }

  if (state == 2 && (millis() - convElapsed) > 3)
  {
    getDigitalValue(DP);
    state = 3;
  }

  if (state == 3)
  {
    startConversion(CONV_D2);
    state = 4;
  }

  if (state == 4 && (millis() - convElapsed) > 10)
  {
    startMeasurment();
    state = 5;
  }

  if (state == 5 && (millis() - convElapsed) > 3)
  {
    getDigitalValue(DT);
    state = 6;
  }

  if (state == 6)
  {
    *internal_altitude = getAltitude();
    state = 0;
    return 1;
  }
  return 0;
}

// read raw digital values of temp & pressure from MS5607
char MS5607::readDigitalValue(void)
{
  if (startConversion(CONV_D1))
  {
    if (startMeasurment())
    {
      if (getDigitalValue(DP))
        ;
    }
  }
  else
  {
    return 0;
  }
  if (startConversion(CONV_D2))
  {
    if (startMeasurment())
    {
      if (getDigitalValue(DT))
        ;
    }
  }
  else
  {
    return 0;
  }
  return 1;
}

char MS5607::getDigitalValue(unsigned long &value)
{
  char x, length = 3;
  unsigned char data[3];
  Wire1.requestFrom(MS5607_ADDR, length);
  while (!Wire1.available())
    ; // wait until bytes are ready
  for (x = 0; x < length; x++)
  {
    data[x] = Wire1.read();
  }
  value = (unsigned long)data[0] * 1 << 16 | (unsigned long)data[1] * 1 << 8 | (unsigned long)data[2];
  return (1);
}

float MS5607::getTemperature(void)
{
  dT = (float)DT - ((float)C5) * ((int)1 << 8);
  TEMP = 2000.0 + dT * ((float)C6) / (float)((long)1 << 23);
  return TEMP / 100;
}

float MS5607::getPressure(void)
{
  dT = (float)DT - ((float)C5) * ((int)1 << 8);
  TEMP = 2000.0 + dT * ((float)C6) / (float)((long)1 << 23);
  OFF = (((int64_t)C2) * ((long)1 << 17)) + dT * ((float)C4) / ((int)1 << 6);
  SENS = ((float)C1) * ((long)1 << 16) + dT * ((float)C3) / ((int)1 << 7);
  float pa = (float)((float)DP / ((long)1 << 15));
  float pb = (float)(SENS / ((float)((long)1 << 21)));
  float pc = pa * pb;
  float pd = (float)(OFF / ((float)((long)1 << 15)));
  P = pc - pd;
  return P / 100;
}

// set OSR and select corresponding values for conversion commands & delay
void MS5607::setOSR(short OSR_U)
{
  this->OSR = OSR_U;
  switch (OSR)
  {
  case 256:
    CONV_D1 = 0x40;
    CONV_D2 = 0x50;
    Conv_Delay = 1;
    break;
  case 512:
    CONV_D1 = 0x42;
    CONV_D2 = 0x52;
    Conv_Delay = 2;
    break;
  case 1024:
    CONV_D1 = 0x44;
    CONV_D2 = 0x54;
    Conv_Delay = 3;
    break;
  case 2048:
    CONV_D1 = 0x46;
    CONV_D2 = 0x56;
    Conv_Delay = 5;
    break;
  case 4096:
    CONV_D1 = 0x48;
    CONV_D2 = 0x58;
    Conv_Delay = 10;
    break;
  default:
    CONV_D1 = 0x40;
    CONV_D2 = 0x50;
    Conv_Delay = 1;
    break;
  }
}

float MS5607::getAltitude(void)
{
  float h, t, p;
  t = getTemperature();
  p = getPressure();
  p = P0 / p;
  h = 153.84615 * (pow(p, 0.19) - 1) * (t + 273.15);
  return h;
}
