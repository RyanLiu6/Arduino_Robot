//use of this library disables the use of analogWrite()[PWM]
//functionality on pins 9 and 10
//whether or not those pins are connected to a servo motor
#include <Servo.h>
#define Range 50
#define colRange 5

///////////
// Servo //
///////////
// Create servo object to control a servo
Servo myServo;

// Variable to store the current position
int pos = 0;

// Variables to store the values of sensor readings
float curr = 0;
float left = 0;
float right = 0;

///////////////
// Car Motor //
///////////////
int S1 = 5; // PWM1 Controls speed
int D1 = 4; // Direction 1  IN Controls Distance
int S2 = 6; // PWM2    EN Controls speed
int D2 = 7; // Direction 2 Controls Distance
int MAX_SPEED = 255; // Max speed
int default_speed = 100; // Default speed for debugging purposes
int turn = 125;

///////////////////////
// Ultrasonic Sensor //
///////////////////////
int temp = A1;
int trig = 3;
int echo = 2;
int buzzer = 1;    // Possible Implementation of Piezzo buzzer
double D = 293.66;
long maxDistance = 50.0; // Max Distance you want the Sensor to measure
long buzzerDistance = 10.0; // Threshold value for the buzzer
long duration, sound, tempVal;
float distance; //This is in cm.

void setup() {
	// Attaches servo on pin 9 to servo object
	myServo.attach(9);

	pinMode(D1, OUTPUT);
	pinMode(D2, OUTPUT);
	// pinMode(S1, OUTPUT);
	// pinMode(S2, OUTPUT);

	pinMode(temp, INPUT);
	pinMode(trig, OUTPUT);
	pinMode(echo, INPUT);
	pinMode(buzzer, OUTPUT);
	Serial.begin(9600);
}

void loop() {
	go_forward(default_speed);        // Testing going forward for 1.5 seconds
	delay(1500);
	stop();
	delay(2000);
	char val;
	while (1) {
		val = Serial.read();
		if (val != -1) {
			switch (val) {
			case 'w':
				go_forward(default_speed);
				break;
			case 'x':
				go_backward(default_speed);
				break;
			case 'd':
				turn_right(default_speed);
				break;
			case 'a':
				turn_left(default_speed);
				break;
			case 's':
				stop();
				break;
			}
		}
	}

	curr = getDistance();

	if (curr <= Range)
	{
		while (curr >= colRange)
		{
			// Slow down till stop
		}

		if (curr < colRange)
		{
			setServo(90);
			right = getDistance();
			setServo(180);
			left = getDistance();
		}
	}

	while (left != 0 && right != 0)
	{
		if (left > right)
		{
			turn_left(turn);
		}
		if (right > left)
		{
			turn_right(turn);
		}
	}
}

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

float getDistance()
{
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


	// Calculating distance using duration
	// Getting rid of garbage values from sensor
	if (distance < maxDistance)
	{
		// Calculating distances with given formula and new Speed of Sound
		Serial.println(distance);
		return distance;
	}
}

/**
 * Makes the car move foward given the speed in int.
 * @param  {int} int speed         input speed
 */
void go_forward(int speed) {
	motor1(speed, false);
	motor2(speed, false);
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
	motor1(speed, true);
	motor2(speed, true);
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
void motor1(int speed, boolean reverse) {
	analogWrite(S1, speed);
	if (reverse)
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
