//////////////////////////////////////////////////////////
//==========This file is for Everything ================//
//////////////////////////////////////////////////////////

///////////////////////
//=====Libraries=====//
///////////////////////
#include <Servo.h>


////////////////////////
//=====Constants===== //
////////////////////////
#define MAX_SPEED 225   // Max Speed
#define TURN_SPEED 125    // Turning Speed
#define TURN_DURATION 730 // Turn Duration calculated using average of two RPM
#define TURN_LEFT_DUR 690
#define MIN_SPEED 85    // Minimum speed that allows the 2WD to move at, and for crawling
#define SPEED_ADJUST 3    // Speed difference tweaker
#define STR_DURATION 600 //straight duration for functionality 3
#define BACK_DURATION 300
#define FLP_DURATION 1465
#define STR_SPEEED 160


////////////////////////////////////////////////////////////
//=================Pin Initializations=================== //
////////////////////////////////////////////////////////////

////////////////
//Lever Switch//
////////////////
const byte leverPin = 1;

//////////////
//Car Motor //
//////////////
const byte S1 = 5; // PWM1 Controls speed
const byte D1 = 4; // Direction 1  IN Controls Distance
const byte S2 = 6; // PWM2    EN Controls speed
const byte D2 = 7; // Direction 2 Controls Distance


///////////////
//button pin //
///////////////
const byte buttonPin = 8;


////////////////////////
//servo (pins 9 & 10) //
////////////////////////
Servo myServo; // Create servo object to control a servo


/////////////
//LED pins //
/////////////
const byte ledPin1 =  11; // LED 1 connected to digital pin 11 (Red)
const byte ledPin2 =  12; // LED 2 connected to digital pin 12 (Green)
const byte ledPin3 =  13; // LED 3 connected to digital pin 13 (Blue)


////////////////////////
//optical sensor pins //
////////////////////////
const byte opticalPinR = A3;
const byte opticalPinC = A4;
const byte opticalPinL = A5;

////////////////
//ultra sonic //
////////////////
const byte temp = A3;
const byte trig = 3;
const byte echo = 2;


/////////////////////////
// Hall effect sensors //
/////////////////////////
int left_sensor = 0;
int right_sensor = 1;


//////////////////////////////////////////////////////////
//=================Global Variables===================  //
//////////////////////////////////////////////////////////


//////////////////////////
//robot state variables //
//////////////////////////
byte robotMode = 0; //variable for robot mode
byte moveMode = 0; //varaible for robot move mode (functionality 2)
byte hCnt = 0; //horizontal line counter (functionality 2)


//////////////////////////
//robot speed variables //
//////////////////////////
int robotSpd = 115;
int turnSpd = 100;
int slowSpd = 80;


///////////////////////////
//button state variables //
///////////////////////////
byte newButtonState = 0; //state variable for funcitionality
byte pastButtonState = 0; //state variable for funcitionality


/////////////////////////////
//optical sensor variables //
/////////////////////////////
float sensor_L, sensor_C, sensor_R;


//////////////////////////////
//robot move mode constants //
//////////////////////////////
enum mode {findLine, straight, left, right, horLine};


/////////////
//Distance //
/////////////
long maxMeasuring = 1000.0;   // Max Distance you want the Sensor to measure
int caution_distance = 70;    // Caution distance, starts looking to slow down at this distance
long stopDistance = 15.0;     // Threshold value to stop
long duration, tempVal, sound;
double current_distance;



//////////
//Servo //
//////////
int pos = 0;
float curr = 0;
//float left = 0;
//float right = 0;
//Servo myServo;


////////////////
//Hall Effect //
////////////////
int left_speed;

///////////
//Func 3 //
///////////
byte fScan;
byte turnDir;
double turnDistance[2];

//////////////////////////////////////////////////////
//=================Arduino Setup=================== //
//////////////////////////////////////////////////////
void setup() {
  // initialize the LED pins as outputs
  pinMode(ledPin1, OUTPUT);
  pinMode(ledPin2, OUTPUT);
  pinMode(ledPin3, OUTPUT);

  //initialize buttonPin as input
  pinMode(buttonPin, INPUT);

  //initialize servo
  myServo.attach(9);

  //set LED colour to match the robotMode
  setLED(robotMode);

  // For motor
  pinMode(D1, OUTPUT);
  pinMode(D2, OUTPUT);

  // For distance
  pinMode(temp, INPUT);
  pinMode(trig, OUTPUT);
  pinMode(echo, INPUT);

  // Hall-effect
  left_speed = MAX_SPEED;

  //lever switch
  pinMode(leverPin, INPUT);

  fScan=0;
  turnDistance[0] = 99;
  turnDistance[1] = 99;
}


/////////////////////////////////////////////////////
//=================Arduino Loop=================== //
/////////////////////////////////////////////////////
void loop() {
  execFunction();
}


/////////////////////////////////////////////////////////
//=================Helper Functions=================== //
/////////////////////////////////////////////////////////

/*
 * Makes the robot execute a particular functionality.
 * The function the robot will execute will depend on robotMode.
 * 
 * Function 1:
 *    If there is no object in front of the robot, 
 *    the robot will move straightforward at maximum speed. 
 *    If the robot detects an object, it will slow down gradually and stop. 
 *    Then, it will use the ultrasonic sensor to scan in which direction
 *    has the most free space, and will turn toward that direciton.
 * 
 *  Function 2:
 *    The robot will use reflective sensors to
 *    follow a line autonomously. If it does not detect a line,
 *    it will try to find a line.
 *    
 * Function 3: (Soon to be implemented)
 *    The robot will cover as much as area as possible.   
 */
void execFunction() {
  setRobotMode(); //set robotMode

  switch (robotMode) {
    case 0: //execute function 1
      straight_wall();
      break;
    case 1: //execute function 2
      setMoveMode();
      break;
    case 2: //execute function 3
      coverSpace();
      break;
  }
}

//////////////////////////////////
/*
* Sets the mode of the robot.
* The color of the LED will be set to indicate 
* which function the robot is executing.
* 
* LED Color:
*   Red - function 1 
*   Green - function 2
*   Blue - function 3
*/
void setRobotMode() {
  newButtonState = digitalRead(buttonPin);

  //only executed if the button is pressed
  if (newButtonState != pastButtonState && newButtonState == HIGH) {
    pastButtonState = newButtonState;
    robotMode++;
    if (robotMode == 3) //mode is bounded between 0-2
      robotMode = 0;
    delay(300); //delay added to prevent debouncing
    pastButtonState = LOW;

    //set functionality mode
    setLED(robotMode);
  }
}

/*
 * Sets the color of the LED
 *    param - lightMode : 0 - set LED to red
 *                        1 - set LED to green
 *                        2 - set LED to blue
 */
void setLED(byte lightMode) {
  switch (lightMode) {
    case 0: //Red
      digitalWrite(ledPin1, LOW);
      digitalWrite(ledPin2, HIGH);
      digitalWrite(ledPin3, HIGH);
      break;
  
    case 1: //Green
      digitalWrite(ledPin1, HIGH);
      digitalWrite(ledPin2, LOW);
      digitalWrite(ledPin3, HIGH);
      break;
  
    case 2: //Blue
      digitalWrite(ledPin1, HIGH);
      digitalWrite(ledPin2, HIGH);
      digitalWrite(ledPin3, LOW);
      break;
  }
}

/////////////////////////////////////////////////////////
//=====Functions of Functionality 3 (Cover space)======//
/////////////////////////////////////////////////////////
/*
 * Main function for Functionality 3
 * The robot will cover as much as space as possible.
 * Initially, the robot will move forward until it hits a wall.
 * 
 */
void coverSpace(){  
  custom_forward(STR_SPEEED+20,STR_SPEEED);
  delay(200);
    
  if(readLever()){
    stop();
    delay(50); 
  
    checkDirAndTurn();
  }
}

/*
 * Makes the robot check direction and turn.
 * This function is triggered after the robot hits a wall.
 * After robot hits a wall, it will move forward for a brief moment
 * (to ensure that it is aligned against the wall).
 * 
 * When it hits the wall for the first time
 * It will check how much space there is in the left direction and in the right direction. 
 * Then, it will turn toward the direction that has more space, 
 * move forward briefy, and make a another 90 degree turn in the same direction.
 *        ===============
 *        || --------> ||
 *        ||        |  ||
 *        || <-------- ||
 *        || |         ||     
 *        || --------> ||
 *        ===============
 * It will continue to do this until it reaches the other edge.
 * Then, it will move toward the opposite edge.
 */
void checkDirAndTurn(){
  go_forward(MAX_SPEED);
  delay(STR_DURATION);
  stop();
  delay(50);
  
  if(!fScan){
    turnDir = spaceScan();
    fScan=1;
  }
  else if(!turnDir)
     turnDistance[turnDir]=scan_right();
  else
     turnDistance[turnDir]=scan_left();
  
  setServo(90);
  
  if(turnDistance[0]<20 && turnDistance[1]<20 ){
     incTurnDir(); //when reached the other edge, move toward the opposite direction
  }


  //if too close to the wall, 
  //make a straight 180 degree turn
  if(turnDistance[turnDir]<20){
    turn180();
    stop();
    delay(100);
  }
  else
    turn();

  incTurnDir();
} 

/*
 * increments turnDir
 */
void incTurnDir(){
  turnDir++;
  if(turnDir==2)
    turnDir=0;
}

/*
 * Reads lever state
 * 
 * returns :
 *        0 - if lever is pressed
 *        1 - if lever is not pressed 
 */
byte readLever(){  
  if(digitalRead(leverPin)==HIGH)
    return 1;
  else
    return 0;
}

/*
 * Makes the robot back up a little bit
 * then turn left/right, move forward slightly, then turns left/right. 
 *   The robot will turn right (both times) if turnDir is 0.
 *   The robot will turn left if turnDir is 1.
 */
void turn(){
  custom_backward(robotSpd+25,robotSpd);
  delay(BACK_DURATION);
  stop();
  delay(50);
  
  if(!turnDir){
    turn_right(TURN_SPEED);
    delay(TURN_DURATION);
    
    custom_forward(robotSpd+18,robotSpd);
    delay(STR_DURATION);

    turn_right(TURN_SPEED);
    delay(TURN_DURATION);
 
  }
  else{
    turn_left(TURN_SPEED);
    delay(TURN_LEFT_DUR);

    custom_forward(robotSpd+18,robotSpd);
    delay(TURN_LEFT_DUR);    

    turn_left(TURN_SPEED);
    delay(TURN_LEFT_DUR);
  }
}

/*
 * Turns 180 degrees 
 */
void turn180(){
    turn_right(TURN_SPEED);
    delay(FLP_DURATION);
}

/*
 * Scans right and left and returns
 * the direction that has most space
 * 
 * returns:
 *      0 - right
 *      1 - left
 */
byte spaceScan(){
  turnDistance[0] = scan_right();
  turnDistance[1] = scan_left();
  setServo(90);

  if(turnDistance[0] > turnDistance[1])
    return 0; //right
  else
    return 1; //left
}

/*
 * Go forward with each motor set at different speed
 * (to make it go straight)
 *    param : left_speed - speed of left motor
 *            right_speed - speed of right motor
 */
void custom_forward(int left_speed, int right_speed){
  motor1(left_speed, true);
  motor2(right_speed, true); 
}

/*
 * Go backward with each motor set at different speed
 * (to make it go straight)
 *      param : left_speed - speed of left motor
 *              right_speed - speed of right motor
 */
void custom_backward(int left_speed, int right_speed){
  motor1(left_speed, false);
  motor2(right_speed, false); 
}

////////////////////////////////////////////////////////////////
//======Functions for Functionality 2 (Line Following)========//
////////////////////////////////////////////////////////////////

/*
 * sets the robot's movement mode
 * which is determined by optical sensor readings
 */
void setMoveMode() {
  moveMode = getMoveMode();
  switch (moveMode) {
    case findLine: //try to find a line
      searchLine();
      break;
    case straight: //go straight
      go_forward(robotSpd);
      break;
    case left: //turn left
      turn_left(turnSpd);
      break;
    case right: //turn right
      turn_right(turnSpd);
      break;
    case horLine: //behvavior for when encountering a horizontal line
      hline();
      break;
  }
}

/*
 * Behavior for when robot encounters a horizontal line
 * The robot will turn right or left slightly.
 * The direction it will turn will alternate each time the function runs.
 */
void hline() {
  if (hCnt)
    turn_right(turnSpd);
  else
    turn_left(turnSpd);
  delay(100);
  
  go_forward(slowSpd);
  delay(30);
  
  if (hCnt == 2)
    hCnt = 0;
}

/*
 * Behavior for when robot does not detect a line.
 * Initially, it will go around in a square.
 * If it still does not find a line, it will go around a larger square.
 */
void searchLine() {
  byte dirCnt = 0;
  int whiteCnt = 0;
  int whiteLimit = 200;
  /*
   *   Robot will go straight for (whiteLimit) cycles while it does not detect a line.
   *   Then, it will turn 90 degrees to the right and will go straight for another (whiteLimit) cycles.
   *   It will repeat this 3 more times. Then, (whiteLimit) will increase by two folds.
   *   The robot will keep doing this until it finds a line.
   */
  while (getMoveMode() == findLine) {
    if (whiteCnt == whiteLimit) {
      turn90();
      dirCnt++;
      if (dirCnt == 4) {
        whiteLimit = whiteLimit << 1;
        dirCnt = 0;
      }
      whiteCnt = 0;
    }
    go_forward(slowSpd);
    delay(40);
    whiteCnt++;
  }
}

/*
 * Determines movement mode of the robot by comparing optical sensor values.
 *
 * returns : findLine - if sensor_C<300 && sensor_R<300 && sensor_L<300
 *           stp - if sensor_C>700 && sensor_R>700 && sensor_L>700
 *           straight - if sensor_C>sensor_R && sensor_C>sensor_L
 *           right - if sensor_R-sensor_L>20
 *           left - if sensor_L-sensor_R>20
 */
byte getMoveMode() {
  readOptical();
  if (sensor_C < 300 && sensor_R < 300 && sensor_L < 300)
    return findLine;
  else if (sensor_C > 700 && sensor_R > 700 && sensor_L > 700)
    return horLine;
  else if (sensor_C > sensor_R && sensor_C > sensor_L)
    return straight;
  else if (sensor_R - sensor_L > 20)
    return right;
  else if (sensor_L - sensor_R > 20)
    return left;
  else
    return straight;
}

/*
 * Reads optical sensor values and saves those values
 * into sensor_L, sensor_C, sensor_R
 */
void readOptical() {
  sensor_L = analogRead(opticalPinL);
  sensor_C = analogRead(opticalPinC);
  sensor_R = analogRead(opticalPinR);
}

/*
 * turns 90 degrees to the right
 */
void turn90() {
  turn_right(150);
  delay(565);
  stop();
  delay(30);
}



////////////////////////////////////////////////////////
//=========Functions for Functionality 1============= //
////////////////////////////////////////////////////////

/*
 * Set servo motor at a particular angular position
 *    param:  targetPos - the angle you want to set it to
 */
void setServo(int targetPos) {
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

/**
 * B-Line to a surface, slows down, stops and turns
 */
void straight_wall() {
  int slow_speed;         // slowed down speed

  current_distance = getDistance(); // Get current distance

  Serial.println(current_distance);

  // Enters the loop when the distance reaches a valid caution_distance
  while (current_distance <= caution_distance && current_distance < maxMeasuring) {

    // Priority1: Stopping at a stopDistance
    if (current_distance <= stopDistance) {
      Serial.println("Stopping");
      stop();
      Serial.println("Stopped");
      turn_to_best();           // Turn 90 degrees to the "best direction"
    }
    // Priority2: slow down when there's space and crawl
    else if (current_distance <= stopDistance * 3) {
      go_forward(MIN_SPEED);
      Serial.println("crawling");
    }

    // Priority 3: slow down relative to the left over distance
    else if (current_distance <= stopDistance * 5) {
      Serial.println("Slowing Down Bro");
      slow_speed = scale_speed(current_distance);
      if (slow_speed < MIN_SPEED) {
        slow_speed = MIN_SPEED;
      }
      go_forward(slow_speed);
      Serial.print("Your speed is: ");
      Serial.println(slow_speed);

      Serial.print("current distance is: ");
      Serial.println(current_distance);
    }

    current_distance = getDistance(); // Update the distance
  }
  go_forward(MAX_SPEED);
  //go_straight();
  Serial.println("going forward");
}

/**
 * Calculates left and right distance and turn towards the bigger distance
 */
void turn_to_best() {

  // Scan Left and retrieve the distance
  Serial.println("Scanning left");
  double left_distance = scan_left();       
  Serial.print("Left distance: ");
  Serial.println(left_distance);

  // Scan Right and retrieve the distance
  Serial.println("Scanning right");
  double right_distance = scan_right();
  Serial.print("Right distance: ");
  Serial.println(right_distance);


  // Return Servo to middle position
  setServo(90);

  //turn to greater distance
  if (right_distance > left_distance) {
    Serial.println("Turning right");
    turn_right(TURN_SPEED);
    delay(TURN_DURATION);
  }
  
  if (right_distance < left_distance) {
    Serial.println("Turning left");
    turn_left(TURN_SPEED);
    delay(TURN_DURATION);
  }

  stop();
  Serial.println("Turning back to original");
}

/**
 * returns scalable speed value, gradually slower speed
 * @param  {double}  distance    The distance in cm
 * @return {int}        The speed between MIN_SPEED and 255
 */
int scale_speed(double distance) {
  return (int) (MAX_SPEED - MIN_SPEED) / (caution_distance - stopDistance) * distance - (MAX_SPEED - MIN_SPEED) / (caution_distance - stopDistance) * 10;
}

void go_straight() {
  adjust_left(left_speed);

  long left_rotation = 0;
  long right_rotation = 0;
  long difference;

  while(left_rotation < 300 || left_rotation > 500){
    left_rotation = get_one_time(left_sensor);
  }

  Serial.println("2");

  while(right_rotation < 300 || right_rotation > 500){
    right_rotation = get_one_time(right_sensor);
  }

  Serial.println("3");

  difference = right_rotation - left_rotation;

  Serial.print(left_rotation);
  Serial.print("      ");
  Serial.print(right_rotation);
  Serial.print("    difference:   ");
  Serial.print(difference);
  Serial.print("    left speed:   ");
  Serial.println(left_speed);


  if (difference > 10){
    // Slow down
      left_speed -= 3;
  }

  else if (difference < 10){
    // Speed up
    left_speed += 5;
    if(left_speed > MAX_SPEED){
      left_speed = MAX_SPEED;
    }
  }
  adjust_left(left_speed);
}

//////////////////////////////////////////
//========Distance (Function 1)=========//
//////////////////////////////////////////

/**
 * returns distance in double
 * @return {double} the distance measured by the Ultra-Sonic and the analog temp
 */
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
  double distance = duration / (2.0 / (sound * 100 * (1.0 / 1000000.0)));
  return distance;
}



/**
 * scans right and then retrieves its distance
 * @return {[double]} The distance in cm
 */
double scan_right() {
  double value = 1001;
  setServo(0);
  delay(250);
  while (value > maxMeasuring) {
    value = getDistance();
  }
  return value;
}

/**
 * scans left and then retrieves its distance
 * @return {[double]} The distance in cm
 */
double scan_left() {
  double value = 1001;
  setServo(180);
  delay(250);
  while (value > maxMeasuring) {
    value = getDistance();
  }
  return value;
} 


//////////////////////////////////////////////
//======== Hall Effect (Function 1)=========//
//////////////////////////////////////////////

long get_one_time(int pin) {
    int val = analogRead(pin);
    long initial_time;
    long lap;

    // Start the time when the wheel is at 0 position
    while (val > 10) {
        val = analogRead(pin);
    }
    initial_time = millis();

    // Ensure that the position is far away from 0
    while (val < 90) {
        val = analogRead(pin);
    }

    while (val > 90) {
        val = analogRead(pin);
    }

    lap = millis() - initial_time;
    return lap;
}


////////////////////////////////////////////////////////
//=========Functions for Robot Movement============== //
////////////////////////////////////////////////////////

/**
 * Makes the car move foward given the speed in int.
 * @param  {int} int speed         input speed
 */
void go_forward(int speed) {
  motor1(speed * 1.023, true);
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

void adjust_left(int speed){
    motor1(speed, true);
    motor2(MAX_SPEED, true);
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


void turn_left_hall(){
  go_forward(125);
    while (analogRead(0) < 50){
  }
  while (analogRead(0) > 50){
  }
  motor1(125, true);
  motor2(0, true);
  while(analogRead(0) < 50){
  }
  motor1(100, true); 
  while(analogRead(0) > 50){
  }
    while(analogRead(0) < 50){
  }
  delay(400);
 stop();
}

void turn_right_hall(){
  go_forward(125);
    while (analogRead(1) < 50){
  }
  while (analogRead(1) > 50){
  }
  motor2(125, true);
  motor1(0, true);
  while(analogRead(1) < 50){
  }
  motor2(100, true); 
  while(analogRead(1) > 50){
  }
    while(analogRead(1) < 50){
  }
  delay(400);
 stop();
}

