#include "NTUTimer.h"
#include <Arduino.h>

void NTUTimer::Start(int timeSpan)
{
  this->originalTime = timeSpan;
  this->countDown = timeSpan;

  this->lastCallTime = millis();
}

void NTUTimer::Reset()
{
  this->countDown = this->originalTime;
  this->lastCallTime = millis();
}

bool NTUTimer::Update()
{
  if(this->countDown > 0)
  {
    // overall system time as start of this call.
    unsigned long timeNow = millis();

    // How many milliseconds expired since the last update?
    long exp = timeNow - this->lastCallTime;

    // Keep track of the overall system time for the next call.
    this->lastCallTime = timeNow;
    this->countDown -= exp;

    // Did the timer expire?
    if(this->countDown < 0)
    {
      if(this->autoReset)
      {
        // Reset the timer
        this->countDown += this->originalTime;
      }
      else
      {
        this->countDown = 0;
      }

      // Return a value indicating the timer expired
      return true;
    }
  }
   
  // Return a value indicating that the timer has not yet expired
  return false;
}