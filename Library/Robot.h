#ifndef Robot_h
#define Robot_h

#include "Arduino.h"

class Robot
{
  public:
   Robot(int pin);
    void dot();
    void dash();
  private:
    int _pin;
};

#endif