//pin2 = right wheel = 1 = motor2 = avg time per rev = 476 ms, 0.456s slower
//pin1 = left wheel = 0 = motor1 = avg time per rev = 460 ms, 0.418s faster, @125, rpm = 832
//The formula. 1 turn = -1.685x + 865.52
#define hallPin 0 // analog pin for hall effect monitor, this must be an analog pin.

int left_sensor = 0;
int right_sensor = 1;

int global1 = 0;
int global2 = 0;

long left_average;
long right_average;

int S1 = 5; // PWM1 Controls speed
int D1 = 4; // Direction 1  IN Controls Distance
int S2 = 6; // PWM2    EN Controls speed
int D2 = 7; // Direction 2 Controls Distance
int MAX_SPEED = 225; // Max speed
int left_speed;
int speed_tweak = 5;

void setup()
{   left_speed = MAX_SPEED;
    pinMode(D1, OUTPUT);
    pinMode(D2, OUTPUT);
    /*pinMode(S1, OUTPUT);
      pinMode(S2, OUTPUT);*/
    go_forward(MAX_SPEED);
    Serial.begin(9600);
}

void go_forward(int speed) {
    motor1(speed, true);
    motor2(speed, true);
}

void adjust_left(int speed){
    motor1(speed, true);
    motor2(MAX_SPEED, true);
}

void motor1(int speed, boolean forward) {
    analogWrite(S1, speed);
    if (forward)
        digitalWrite(D1, HIGH);
    else
        digitalWrite(D1, LOW);
}

void motor2(int speed, boolean forward) {
    analogWrite(S2, speed);
    if (forward)
        digitalWrite(D2, HIGH);
    else
        digitalWrite(D2, LOW);
}
// returns 0 if there is no change in field, returns
int DoMeasurement()
{
// measure magnetic field
    int raw = analogRead(0);   // Range : 0..1024
    Serial.println(raw);
    return raw;

}


void loop() {
    /*
    printData();

    if (global1 > global2) {
        left_speed -= 5;
    }

    if (global1 < global2) {
        if (left_speed <= 250) {
            left_speed += 5;
        }
        else if (left_speed > 250) {
            left_speed = MAX_SPEED;
        }
    }

    motor2(MAX_SPEED, true);
    motor1(left_speed, true);

    Serial.print("The left speed is: ");
    Serial.println(left_speed);

    int speed = 125;
    Serial.println("Turning");
    turn_right(speed);
    delay(832);
    Serial.println("Stopping");
    stop();
    global1 = 0;
    global2 = 0;
    */

    long left_rotation = get_one_time(left_sensor);
    while(left_rotation < 400 || left_rotation > 600){
        left_rotation = get_one_time(left_sensor);
    }

    long right_rotation = get_one_time(right_sensor);
    while(right_rotation < 400 || right_rotation > 600){
        right_rotation = get_one_time(right_sensor);
    }

    long difference = right_rotation - left_rotation;

    if(difference > 20){
        left_speed -= speed_tweak;
    }

    else if(difference < -20){
        left_speed += speed_tweak;
    }

    if(left_speed > MAX_SPEED){
        left_speed = MAX_SPEED;
    }

    Serial.print(left_rotation);
    Serial.print("      ");
    Serial.print(right_rotation);
    Serial.print("    difference:   ");
    Serial.print("    Left speed:   ");
    Serial.print(left_speed);
    Serial.println(difference);

    adjust_left(left_speed);
}

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
 * Stops the car from moving
 */
void stop() {
    motor1(0, true);
    motor2(0, true);
}
