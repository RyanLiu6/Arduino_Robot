
/**
 * This reads the distance sensor and the analog temp to calculate the new distance
 */

///////////////////////////
// Initialize pin values //
///////////////////////////
int temp = A1;
int trig = 3;
int echo = 4;
int buzzer = 5    // Possible Implementation of Piezzo buzzer
double D = 293.66;
long maxDistance = 50.0; // Max Distance you want the Sensor to measure
long buzzerDistance = 10.0; // Threshold value for the buzzer
long duration, sound, tempVal;
float distance; //This is in cm.

void setup() {
    pinMode(temp, INPUT);
    pinMode(trig, OUTPUT);
    pinMode(echo, INPUT);

    pinMode(buzzer,OUTPUT);

    Serial.begin(9600);
}

void loop() {

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
        if(distance < buzzerDistance){
            buzz(); 
        }
        // Calculating distances with given formula and new Speed of Sound
        Serial.println(distance); 
    }


// Function to buzz when near a obstructing surface
void buzz(){

}


