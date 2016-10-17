/**
 * This makes the car move, can be controlled with the keyboard
 */

////////////////////
//Initialize pins //
////////////////////

int S1 = 5; // PWM1 Controls speed
int D1 = 4; // Direction 1  IN Controls Distance
int S2 = 6; // PWM2    EN Controls speed
int D2 = 7; // Direction 2 Controls Distance
int MAX_SPEED = 255; // Max speed
int default_speed = 100; // Default speed for debugging purposes


void setup() {
	// put your setup code here, to run once:
	pinMode(D1, OUTPUT);
	pinMode(D2, OUTPUT);
	/*pinMode(S1, OUTPUT);
		pinMode(S2, OUTPUT);*/
	Serial.begin(9600);
}

void loop() {
	// put your main code here, to run repeatedly:
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

void turn_degrees(int degrees){

}
