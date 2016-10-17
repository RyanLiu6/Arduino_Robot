//pin2 = right wheel = 1 = motor2 = avg time per rev =
//pin1 = left wheel = 0 = motor1 = avg time per rev = 407 ms
//motor2 -

#define hallPin 0 // analog pin for hall effect monitor, this must be an analog pin.

int global1 = 0;
int global2 = 0;
int S1 = 5; // PWM1 Controls speed
int D1 = 4; // Direction 1  IN Controls Distance
int S2 = 6; // PWM2    EN Controls speed
int D2 = 7; // Direction 2 Controls Distance
int MAX_SPEED = 255; // Max speed
void setup()
{
    pinMode(D1, OUTPUT);
    pinMode(D2, OUTPUT);
    /*pinMode(S1, OUTPUT);
      pinMode(S2, OUTPUT);*/
    Serial.begin(9600);
}

void go_forward(int speed) {
    motor1(speed, true);
    motor2(0, true);
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

void loop()
{
    go_forward(15);
    triggeredinx(10000, 0, 1);
    Serial.print("left wheel:");
    Serial.println(global1);
    Serial.print("right wheel:");
    Serial.println(global2);
    global1 = 0;
    global2 = 0;

    
}
void stop() {
    motor1(0, true);
    motor2(0, true);
}

/*
 * @param:timeused: time range to measure how many times the hall sensor was triggered
 *        pin: the pin number of the hall monitor to be used
 *
 * updates number of times hall sensor was triggered to the global variables(read value went from high to low)
 */

void triggeredinx(double timeused, int pin1, int pin2) {
    // boolean for when the device is triggered
    int trigger1 = 0;
    int trigger2 = 0;
    // the value read from hall sensor
    int readval1 = 0;
    int readval2 = 0;
    // the starting time
    long initial = millis();
    // the current time
    long actual = millis();

    long left_current = millis();
    long right_current = millis();

    // while the time hasn't passed time used
    while ((actual - initial) < timeused) {
        // read the value from pin
        readval1 = analogRead(pin1);
        readval2 = analogRead(pin2);
        // if the value is above 50 ( neutral field)
        if (readval1 > 50 && trigger1 == 0)
            // set trigger to 1, meaning it can be triggered again
            trigger1 = 1;
        // else if the value is below 50 and trigger is set ( trigger the device, set boolean, increment tally)
        else if (readval1 < 50 && trigger1 == 1) {
            trigger1 = 0;
            // updating global variable
            global1++;
            Serial.print("triggered left wheel:  ");
            Serial.println(millis() - left_current);
            left_current = millis();

        }
        if (readval2 > 50 && trigger2 == 0)
            // set trigger to 1, meaning it can be triggered again
            trigger2 = 1;
        // else if the value is below 50 and trigger is set ( trigger the device, set boolean, increment tally)
        else if (readval2 < 50 && trigger2 == 1) {
            trigger2 = 0;
            // updating global variable
            global2++;
            Serial.print("triggered right wheel: ");
            Serial.println(millis() - right_current);
            right_current = millis();
        }
        // prints to help debug, comment out if you like
        //Serial.print(actual);
        //Serial.print(":");
        //Serial.println(readval);
        actual = millis();
    }
    Serial.println("end");
}

