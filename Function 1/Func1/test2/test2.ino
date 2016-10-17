//////////////////////////////////////////////////////////
//----------This file is for Principal 1--------------- //
//////////////////////////////////////////////////////////



///////////////
// Libraries //
///////////////
#include <Servo.h>



///////////////
// Constants //
///////////////
#define MAX_SPEED 225		// Max Speed
#define TURN_SPEED 125		// Turning Speed
#define TURN_DURATION 680	// Turn Duration calculated using average of two RPM
#define MIN_SPEED 85		// Minimum speed that allows the 2WD to move at, and for crawling
#define SPEED_ADJUST 3		// Speed difference tweaker


////////////////////////////////////
//-------Initializing Pins------- //
////////////////////////////////////

//////////////
//Car Motor //
//////////////
int S1 = 5; // PWM1 Controls speed
int D1 = 4; // Direction 1  IN Controls Distance
int S2 = 6; // PWM2    EN Controls speed
int D2 = 7; // Direction 2 Controls Distance


////////////////
//ultra sonic //
////////////////
int temp = A3;
int trig = 3;
int echo = 2;


/////////////////////////
// Hall effect sensors //
/////////////////////////
int left_sensor = 0;
int right_sensor = 1;

//////////////////////////////////////
//========Global Variables========////
//////////////////////////////////////

/////////////
//Distance //
/////////////

long maxMeasuring = 1000.0; 	// Max Distance you want the Sensor to measure
int caution_distance = 70;		// Caution distance, starts looking to slow down at this distance
long stopDistance = 15.0; 		// Threshold value to stop
long duration, tempVal, sound;
double current_distance;

//////////
//Servo //
//////////
int pos = 0;
float curr = 0;
float left = 0;
float right = 0;
Servo myServo;


////////////////
//Hall Effect //
////////////////
int left_speed = MAX_SPEED;
int right_speed = MAX_SPEED;

////////////////////////////////////////////////
//=========== Start Set up ================== //
////////////////////////////////////////////////

void setup() {
	
	// For motor
	pinMode(D1, OUTPUT);
	pinMode(D2, OUTPUT);
	/*pinMode(S1, OUTPUT);
		pinMode(S2, OUTPUT);*/


	// For distance
	pinMode(temp, INPUT);
	pinMode(trig, OUTPUT);
	pinMode(echo, INPUT);



	// servo
	myServo.attach(9);

	// For motor
	Serial.begin(9600);

}
void loop() {
	straight_wall();
}


////////////////////////////////////////////////
//=========== Movement Functions ============ //
////////////////////////////////////////////////


/**
 * Makes the car move foward given the speed in int.
 * @param  {int} int speed         input speed
 */
void go_forward(int speed) {
	motor1(speed, true);
	motor2(speed, true);
}

/**
 * Makes the car move foward according to speeds stored it hall sensor global variables
 */
void go_forward() {
  motor1(left_speed, true);
  motor2(right_speed, true);
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
 * @param  {int} int speed         	   	   rotation speed
 * @param  {boolean} boolean reverse       rotation direction, true for reverse
 */
void motor2(int speed, boolean reverse) {
	analogWrite(S2, speed);
	if (reverse)
		digitalWrite(D2, HIGH);
	else
		digitalWrite(D2, LOW);
}

/**
 * B-Line to a surface, slows down, stops and turns
 */
void straight_wall() {
	int slow_speed; 				// slowed down speed

	current_distance = getDistance();	// Get current distance

	Serial.println(current_distance);

	// Enters the loop when the distance reaches a valid caution_distance
	while (current_distance <= caution_distance && current_distance < maxMeasuring) {

		// Priority1: Stopping at a stopDistance
		if (current_distance <= stopDistance) {
			Serial.println("Stopping");
			stop();
			Serial.println("Stopped");
			turn_to_best();						// Turn 90 degrees to the "best direction"
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
	forward(MAX_SPEED);
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
	setServoNow(90);

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

void adjust_left(int speed){
    motor1(speed, true);
    motor2(MAX_SPEED, true);
}

/* Adjusts the speed when going straight to account for differences in servo speed using hall sensor to detect time of 1 rotation
 */
void go_straight() {

	long left_rotation = 0;
	long right_rotation = 0;
	long difference;
  int  speed_difference;

	left_rotation = get_x_time(left_sensor, 10);
  right_rotation = get_x_time(right_sensor, 10);

	difference = right_rotation - left_rotation;
  speed_difference = right_speed - left_speed;
	Serial.print(left_rotation);
	Serial.print("      ");
	Serial.print(right_rotation);
	Serial.print("    difference:   ");
	Serial.println(difference);
  Serial.print("   speed difference:   ");
  Serial.println(speed_difference);



}




//////////////////////////////////////////
//=============== Distance ============ //
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
	setServoNow(0);
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
	setServoNow(180);
	delay(250);
	while (value > maxMeasuring) {
		value = getDistance();
	}
	return value;
}

/**
 * set servo direction
 * @param {int} targetPos 0 means right, 180 means left, 90 for middle
 */
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

//////////////////////////////////////////////
//=============== Hall Effect ============= //
//////////////////////////////////////////////

/**@params: pin: the pin number for hall sensor
 * 
 * returns: One spin of wheel as detected by hall sensor
 */
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

long get_x_time(int pin, int times) {
  long lap = 0;
  while(times > 0){
    lap += get_one_time(pin);
  }
  return lap;
}

