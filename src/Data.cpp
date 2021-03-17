#include <Arduino.h>
#include "Data.h"
#include "./SdCard/SPI.h"
#include "./SdCard/SPIMemory.h"
#include "Chrono.h"

Chrono dataWriteTimer;
SPIFlash flash(SS_FLASH, &SPI2);
Data data;

uint16_t rateHz = 100;
uint16_t numSeconds = 10;
uint16_t millisPerSample = 1000 / rateHz;
uint16_t totalSamples = rateHz * numSeconds;

unsigned long write_addr = 0;
unsigned long read_addr = 0;

unsigned long addrStep = sizeof(data);
unsigned long maxAddr = totalSamples * sizeof(data);

void getMaxAddr()
{
  unsigned long capacity = flash.getCapacity();
  if (maxAddr + sizeof(data) > capacity)
  {
    maxAddr = capacity;
  }
}

void initFlash()
{
  flash.begin();
  delay(50);

  if (flash.eraseChip())
  {
    Serial.println("Erased Flash");
  };

  getMaxAddr();

  Serial.println("Continuing...");
  delay(200);
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

bool writeToFlash()
{
  if (write_addr < maxAddr)
  {
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
  if (read_addr <= write_addr)
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
