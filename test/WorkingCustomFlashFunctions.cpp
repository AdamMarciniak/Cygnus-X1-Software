#include <Arduino.h>
#include "SPI.h"

byte datas[256];

void printJEDEC()
{
  Serial.println("Getting JEDEC..");
  digitalWrite(SS_FLASH, LOW);
  SPI2.transfer(0x9F);
  byte msb = SPI2.transfer(0x9F);

  byte msb1 = SPI2.transfer(0x9F);
  byte msb2 = SPI2.transfer(0x9F);
  digitalWrite(SS_FLASH, HIGH);

  Serial.print(msb, HEX);
  Serial.print(" ");
  Serial.print(msb1, HEX);
  Serial.print(" ");
  Serial.print(msb2, HEX);
  Serial.println();
}

void writeEnable()
{
  digitalWrite(SS_FLASH, LOW);
  SPI2.transfer(0x06);
  digitalWrite(SS_FLASH, HIGH);
}

void sectorErase()
{
  writeEnable();
  digitalWrite(SS_FLASH, LOW);
  SPI2.transfer(0x20);
  SPI2.transfer(0x00);
  SPI2.transfer(0x00);
  SPI2.transfer(0x00);
  digitalWrite(SS_FLASH, HIGH);
}

void unlock()
{
  writeEnable();
  digitalWrite(SS_FLASH, LOW);
  SPI2.transfer(0x98);
  digitalWrite(SS_FLASH, HIGH);
}

void getConfig()
{
  byte newConfig = 0b01010000;
  digitalWrite(SS_FLASH, LOW);
  byte config = SPI2.transfer(0x35);
  SPI2.transfer(newConfig);
  Serial.println(config, BIN);
  digitalWrite(SS_FLASH, HIGH);

  digitalWrite(SS_FLASH, LOW);
  config = SPI2.transfer(0x35);
  Serial.println(config, BIN);
  digitalWrite(SS_FLASH, HIGH);
}

bool isBusy()
{
  byte status;
  digitalWrite(SS_FLASH, LOW);
  SPI2.transfer(0x05);
  status = SPI2.transfer(0x00);
  digitalWrite(SS_FLASH, HIGH);
  Serial.println(status, BIN);
  if (status & 0x01)
  {
    return true;
  }
  else
  {
    return false;
  }
}

void getStatus()
{

  digitalWrite(SS_FLASH, LOW);
  SPI2.transfer(0x05);
  Serial.println(SPI2.transfer(0x00), BIN);
  Serial.println(SPI2.transfer(0x00), BIN);
  Serial.println(SPI2.transfer(0x00), BIN);
  digitalWrite(SS_FLASH, HIGH);
}

void write()
{
  writeEnable();
  digitalWrite(SS_FLASH, LOW);
  SPI2.transfer(0x02);
  SPI2.transfer(0);
  SPI2.transfer(0x02);
  SPI2.transfer(0);

  for (int i = 0; i < 256; i += 1)
  {
    if (i == 10)
    {
      SPI2.transfer(0X12);
    }
    else
    {
      SPI2.transfer(0X15);
    }
  }

  digitalWrite(SS_FLASH, HIGH);
}

void read()
{
  digitalWrite(SS_FLASH, LOW);
  SPI2.transfer(0x03);
  SPI2.transfer(0x00);
  SPI2.transfer(0x02);
  SPI2.transfer(0x00);

  for (int i = 0; i < 256; i += 1)
  {
    Serial.println(SPI2.transfer(0x00), HEX);
  }

  digitalWrite(SS_FLASH, HIGH);
}

void setup()
{
  Serial.begin(115200);
  while (!Serial)
    ;
  pinMode(SS_FLASH, OUTPUT);
  digitalWrite(SS_FLASH, HIGH);

  for (int i = 0; i < 256; i += 1)
  {
    datas[i] = 0xBC;
  }

  Serial.println("Starting..");
  SPI2.begin();
  SPI2.setClockDivider(SPI_CLOCK_DIV32);
  printJEDEC();

  delay(1000);
  //getConfig();
  unlock();

  sectorErase();

  while (isBusy())
  {
    Serial.println("Busy Erasing..");
    delay(20);
  }

  //getConfig();
  Serial.println("Write enabled");
  write();
  delay(1000);
  Serial.println("Wrote all data");
  read();
}

void loop()
{
}