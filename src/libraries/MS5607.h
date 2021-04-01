/*File   : MS5607.h
  Author : Amit Ate
  Email  : amit@uravulabs.com
  Company: Uravu Labs
*/

#define R_ADC 0X00     // adc read command
#define PROM_READ 0xA0 // prom read command
#define RESET 0x1E     // soft reset command

#include <Arduino.h>

class MS5607
{
public:
  MS5607();
  MS5607(short address);
  MS5607(float *altitudeVariable);
  char begin();
  void setOSR(short OSR_U);
  float getTemperature(void);
  float getPressure(void);
  char readDigitalValue(void);
  float getAltitude(void);
  int handleAltimeter(void);
  float altitude;
  bool isDataAvailable(void);

private:
  const float P0 = 1013.25;
  short MS5607_ADDR = 0X76; // default device address of MS5607 (CBS == HIGH)
  short OSR = 4096;         // default over sampling ratio
  short CONV_D1 = 0x48;     // corresponding temp conv. command for OSR
  short CONV_D2 = 0x58;     // corresponding pressure conv. command for OSR
  char Conv_Delay = 10;     // corresponding conv. delay for OSR
  float *internal_altitude;

  unsigned int C1, C2, C3, C4, C5, C6;
  unsigned long DP, DT;
  float dT, TEMP, P;
  int64_t OFF, SENS;
  int state = 0;

  char resetDevice(void);
  char readCalibration();
  char readUInt_16(char address, unsigned int &value);
  char readBytes(unsigned char *values, char length);
  char startConversion(char CMD);
  char startMeasurment(void);

  char getDigitalValue(unsigned long &value);
};
