
#include "libraries/Adafruit_NeoPixel.h"
#include "Arduino.h"
#include "LED.h"
#include "Chrono.h"

#define NUMPIXELS 1
Adafruit_NeoPixel pixels(NUMPIXELS, LED_PIN, NEO_GRB + NEO_KHZ800);

Chrono LED_blink_chrono;

bool blinkState = false;

int blinkDuration = 1000;

int blinkCount = 0;

void initLED()
{
    pixels.begin(); // INITIALIZE NeoPixel strip object (REQUIRED)
}

void turnOnLED()
{
    pixels.clear();
    pixels.setPixelColor(0, pixels.Color(0, 0, 255));
    pixels.show();
}

void turnOffLED()
{
    pixels.clear();
    pixels.setPixelColor(0, pixels.Color(0, 0, 0));
    pixels.show();
}

void setLEDRed(int val)
{
    pixels.clear();
    pixels.setPixelColor(0, pixels.Color(val, 0, 0));
    pixels.show();
}

void setLEDColor(int red, int green, int blue)
{
    pixels.clear();
    pixels.setPixelColor(0, pixels.Color(red, green, blue));
    pixels.show();
}

void setBlinkDuration(int duration)
{
    blinkDuration = duration;
}

void handleLEDBlink(int red, int green, int blue)
{
    if (LED_blink_chrono.hasPassed(blinkDuration))
    {
        if (blinkState == false)
        {
            setLEDColor(red, green, blue);
            blinkState = true;
            blinkCount += 1;
        }
        else if (blinkState == true)
        {
            turnOffLED();
            blinkState = false;
        }
        LED_blink_chrono.restart(); // restart the crono so that it triggers again later
    }
}

void handleFailLED(int errorNum)
{
    handleLEDBlink(255, 0, 0);
}
