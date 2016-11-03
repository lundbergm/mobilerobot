#include <Servo.h>
#define offset1 87
#define offset2 89
int linesensor = A0;  // analog pin used to connect the potentiometer
int trig = 2;
int echo = 7;
int val;    // variable to read the value from the analog pin
int flame;
long duration, distance;
Servo wheel1;  // create servo object to control a servo
Servo wheel2;

void setup() {
    Serial.begin(9600);
    //wheel1.attach(3);
    //wheel2.attach(5); // attaches the servo on pin 9 to the servo object
    pinMode(linesensor, INPUT);
    pinMode(trig, OUTPUT);
    digitalWrite(trig, LOW);
    pinMode(echo, INPUT);
    val = 2;
}

void loop() {
    //wheel1.write(offset1 + val);
    //wheel2.write(offset2 + val);                // sets the servo position according to the scaled value
    //line = analogRead(linesensor);
    //Serial.println(line);

    digitalWrite(trig, HIGH);
    delayMicroseconds(10);
    digitalWrite(trig, LOW);

    duration = pulseIn(echo, HIGH);
    distance = duration / 29 / 2;
    Serial.println(distance);
    delay(100);
}
