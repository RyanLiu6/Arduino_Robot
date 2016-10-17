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
	int hallPin0 = 0;
	int hallPin1 = 1;


	void setup() {
		// put your setup code here, to run once:
		pinMode(D1, OUTPUT);
		pinMode(D2, OUTPUT);
		/*pinMode(S1, OUTPUT);
			pinMode(S2, OUTPUT);*/
		go_forward(MAX_SPEED);
		Serial.begin(9600);
	}

	void loop() {
		// put your main code here, to run repeatedly:
		
		int raw1 = analogRead(hallPin0);
		int raw2 = analogRead(hallPin1);

		Serial.print("Left wheel: ");
		Serial.println(raw1);

		Serial.print("Right wheel: ");
		Serial.println(raw2);

	}

	/**
	 * Makes the car move foward given the speed in int.
	 * @param  {int} int speed         input speed
	 */
	void go_forward(int speed) {
		motor1(speed, true);
		motor2(speed, true);
	}

	void tweak_left(int speed){
		motor1(MAX_SPEED-speed,true);
		motor2(MAX_SPEED, true);
	}

	void tweak_right(int speed){
		motor1(MAX_SPEED+speed,true);
		motor2(MAX_SPEED, true);
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
