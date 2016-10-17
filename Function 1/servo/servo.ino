//use of this library disables the use of analogWrite()[PWM]
//functionality on pins 9 and 10
//whether or not those pins are connected to a servo motor
#include <Servo.h> 

// Create servo object to control a servo
Servo myServo;

// Variable to store the current position
int pos;

void setup() {
  // Attaches servo on pin 9 to servo object
  myServo.attach(9);
}

void loop() {
  setServo();
  // left = 180
  // right = 90
  // straight = 0
}

/*
 * Set servo motor at a particular angular position
 *    param:  targetPos - the angle you want to set it to
 */
void setServo(){
  //make the servor motor turn 1 degree every 15 milliseconds
  //until it reaches the target position

int targetPos = 180;
  for(pos = 0; pos < targetPos; pos += 1)    
  {                                  
    myServo.write(pos);                 
    delay(20);                       
  } 
  
  for(pos = targetPos; pos > 0; pos -= 1)    
  {                                  
    myServo.write(pos);                 
    delay(20);                       
  } 
}

