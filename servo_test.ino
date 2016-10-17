/**
 * This makes the car move, can be controlled with the keyboard
 */

#include <Servo.h> //servo library
/////////////////////////
//Initialize pins motor//
/////////////////////////

int S1 = 5; // PWM1 Controls speed
int D1 = 4; // Direction 1  IN Controls Distance
int S2 = 6; // PWM2    EN Controls speed
int D2 = 7; // Direction 2 Controls Distance
int MAX_SPEED = 255; // Max speed 43.3cm/s
int current_speed; // Default speed for debugging purposes

///////////////////////////////
//Initialize pins ultrasonic //
///////////////////////////////
int temp = A3;
int trig = 3;
int echo = 2;
long maxDistance = 1000.0; // Max Distance you want the Sensor to measure
long buzzerDistance = 10.0; // Threshold value for the buzzer
long duration, sound, tempVal;
float distance; //This is in cm.
int max_object_distance = 30; // 30cm
double current_distance;

int pos = 0;
// Variables to store the values of sensor readings
float curr = 0;
float left = 0;
float right = 0;
Servo myServo;


void setup() {
  myServo.attach(9);
  // For motor
  pinMode(D1, OUTPUT);
  pinMode(D2, OUTPUT);
  /*pinMode(S1, OUTPUT);
    pinMode(S2, OUTPUT);*/


  // For distance

  pinMode(temp, INPUT);
  pinMode(trig, OUTPUT);
  pinMode(echo, INPUT);
  Serial.begin(9600);
  current_speed = MAX_SPEED;

}
void loop() {
  scan_left();
  delay(5000);
  Serial.println("done");
}

void straight_wall(){
  current_distance = getDistance();

  Serial.println(current_distance);
  
  while(current_distance <= max_object_distance && current_distance < 1000){
    Serial.println("Warning getting close");
    
    /*if(current_distance <= max_object_distance*2/3){
      Serial.println("Slowing Down Bro");
      go_forward(MAX_SPEED/4);
    }*/

    if(current_distance <= buzzerDistance){
    Serial.println("Stopping");
      stop();
     Serial.println("Stopped");
      turn_to_best();
    }

    current_distance = getDistance();
  }
  
  go_forward(MAX_SPEED);
  Serial.println("going forward");
}

/**
 * Makes the car move foward given the speed in int.
 * @param  {int} int speed         input speed
 */
void go_forward(int speed) {
  motor1(speed, true);
  motor2(speed, true);
}

/**
 * Stops the car from moving
 */
void stop() {
  motor1(0, true);
  motor2(0, true);
}
/**
 * Makes the car move backward in the given speed
 * @param  {int} speed         given speed
 */
void go_backward(int speed) {
  motor1(speed, false);
  motor2(speed, false);
}
/**
 * Turns the card right in the given speed
 * @param  {int} int speed         input speed
 */
void turn_right(int speed) {
  motor1(speed, true);
  motor2(speed, false);
}
/**
 * Turns the car left in the given speed
 * @param  {int} int speed         input speed
 */
void turn_left(int speed) {
  motor1(speed, false);
  motor2(speed, true);
}


/**
 * Moves the left motor from given speed and direction.
 * boolean determines the direction of the rotation
 * @param  {[int]} int     speed           input speed
 * @param  {boolean} boolean reverse       rotation direction
 */
void motor1(int speed, boolean forward) {
  analogWrite(S1, speed);
  if (forward)
    digitalWrite(D1, HIGH);
  else
    digitalWrite(D1, LOW);
}
/**
 * Moves the right motor from given speed and direction
 * @param  {int} int speed                 rotation speed
 * @param  {boolean} boolean reverse       rotation direction, true for reverse
 */
void motor2(int speed, boolean reverse) {
  analogWrite(S2, speed);
  if (reverse)
    digitalWrite(D2, HIGH);
  else
    digitalWrite(D2, LOW);
}

double getDistance() {
  // Calculating Temperature and Sound using given formulas
  tempVal = (5.0 * analogRead(temp) * 100.0) / 1024;
  sound = 331.5 + (0.6 * tempVal);

  // Using property of Ultrasonic sensor, we can find the duration of each pulse
  // Trigger sends pulses, then echo can calculate the duration
  digitalWrite(trig, LOW);
  delayMicroseconds(5);
  digitalWrite(trig, HIGH);
  delayMicroseconds(5);
  digitalWrite(trig, LOW);
  duration = pulseIn(echo, HIGH);
  distance = duration / (2.0 / (sound * 100 * (1.0 / 1000000.0)));
  return distance;
}

void turn_to_best(){

  //scan left
  //scan right
  Serial.println("Scanning left");
  double left_distance = scan_left();
 Serial.println("Scanning right");
  double right_distance = scan_right();
  if(left_distance > right_distance){
  Serial.println("Turning right");
    turn_right(125);
    delay(2000);
    Serial.println("Turning back to original");
    setServoNow(90);
  }
  //turn to greater distance
  if(right_distance > left_distance){
  Serial.println("Turning left");
    turn_left(125);
    delay(2000);
    Serial.println("Turning back to original");
    setServoNow(90);
  }
}

double scan_right(){
  double value = 1001;
  setServoNow(0);
  delay(250);
  while(value > 1000){
    value = getDistance();
  }
  return value;
}

double scan_left(){
  double value = 1001;
  setServoNow(180);
  delay(250);
  while(value > 1000){
    value = getDistance();
  }
  return value;
}

void setServoNow(int targetPos) {
  //make the servor motor turn 1 degree every 15 milliseconds
  //until it reaches the target position
  while (pos != targetPos) {
    if (targetPos > pos)
      pos++;
    else
      pos--;
    myServo.write(pos);  // tell servo to go to position in variable 'pos'
    delay(15);  // waits 15ms for the servo to reach the position
  }
}
