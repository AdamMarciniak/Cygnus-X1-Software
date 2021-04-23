#include "Data.h"


Chrono dataWriteTimer;
SPIFlash flash(SS_FLASH, &SPI2);
Data data;
NonLoggedData nonLoggedData;

uint16_t rateHz = DATA_SAMPLE_RATE;
uint16_t numSeconds = DATA_SAMPLE_TOTAL_TIME;
uint16_t millisPerSample = 1000 / rateHz;
uint16_t totalSamples = rateHz * numSeconds;

// Starting at next 4K block
unsigned long startAddress = 4097;
unsigned int startSector = 1;
int sectorSize = 4096;

unsigned long write_addr = startAddress;
unsigned long read_addr = startAddress;

unsigned long addrStep = sizeof(data);
unsigned long maxAddr = totalSamples * sizeof(data);

float YCENTER;
float ZCENTER;

void goToState(State state) {
  data.state = state;
  data.fState = float(state);
  
}

void writeTVCCenters()
{
  flash.eraseSector(0);
  flash.writeAnything(0, data.Y_Servo_Center);
  flash.writeAnything(sizeof(float), data.Z_Servo_Center);
  Serial.println("Wrote TVC Centers");
}

void readTVCCenters()
{
  flash.readAnything(0, YCENTER);
  flash.readAnything(sizeof(float), ZCENTER);
  Serial.print("Reading TVC Centers: ");
  Serial.print("Y: ");
  Serial.print(YCENTER);
  Serial.print("  Z: ");
  Serial.println(ZCENTER);
  if (YCENTER >= -200)
  {
    data.Y_Servo_Center = YCENTER;
  }

  if (ZCENTER >= -200)
    data.Z_Servo_Center = ZCENTER;
}

void getMaxAddr()
{
  unsigned long capacity = flash.getCapacity();
  if (maxAddr + sizeof(data) > capacity)
  {
    maxAddr = capacity;
  }
}

void eraseFlightData()
{
  Serial.println("Erasing flight data sectors");
  unsigned long capacity = flash.getCapacity();
  for (unsigned long i = startAddress; i < capacity; i += sectorSize)
  {
    // Erases 1 4K sector
    Serial.print(i);
    Serial.print(",");
    flash.eraseSector(i);
  }
}

void initFlash()
{
  flash.begin();

  readTVCCenters();

  if (flash.eraseChip())
  {
    Serial.println("Erased Flash");
  };

  writeTVCCenters();

  //eraseFlightData();

  getMaxAddr();
}

bool handleWriteFlash()
{
  bool writing = true;
  if (dataWriteTimer.hasPassed(millisPerSample))
  {
    dataWriteTimer.restart();
    writing = writeToFlash();
  }

  return writing;
}

bool firstWrite = true;
unsigned long startTime = 0;

bool writeToFlash()
{
  if (write_addr < maxAddr)
  {
    if(firstWrite == true){
      firstWrite = false;
      data.ms = 0.0;
      startTime = millis();
    } else {
      data.ms = float(millis() - startTime);
    }
    flash.writeAnything(write_addr, data);
    write_addr += addrStep;
    return true;
  }
  else
  {
    return false;
  }
}

bool readFromFlash()
{
  if (read_addr < write_addr)
  {
    Serial.print("Reading From Flash Addr: ");
    Serial.print(" ");
    Serial.println(read_addr);
    flash.readAnything(read_addr, data);

    read_addr += addrStep;
    return true;
  }
  else
  {
    return false;
  }
}


