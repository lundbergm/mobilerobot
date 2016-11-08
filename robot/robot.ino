#include <Servo.h>
#define offsetR 87
#define offsetL 90
#define LEFT -1
#define NORMAL 0
#define RIGHT 1
int linesensorL = A0;  // analog pin used to connect the potentiometer
int linesensorR = A1;
int frontsensorR      = A2;
int frontsensorL      = A3;
int trig = 2;
int echo = 7;
int gripperPin = 6;
int button1Pin = 12;
int button2Pin = 13;
int val;    // variable to read the value from the analog pin
int lineL;
int lineR;
int frontL;
int frontR;
long duration, distance;
Servo wheelR;  // create servo object to control a servo
Servo wheelL;
Servo gripper;
int button1Val = 0;
int button2Val = 0;
int gripperVal;
int speed;
int turn;
int conf = 0;
int speedL = 0;
int speedR = 0;
int mode;

void initMotors(){
    //gripper.attach(gripperPin);

    wheelR.attach(3);
    wheelL.attach(5);
}

void setup() {
    Serial.begin(9600);
    initMotors();
    pinMode(linesensorL, INPUT);
    pinMode(linesensorR, INPUT);
    pinMode(frontsensorL, INPUT);
    pinMode(frontsensorR, INPUT);
    pinMode(trig, OUTPUT);
    pinMode(button1Pin, INPUT_PULLUP);
    pinMode(button2Pin, INPUT_PULLUP);
    digitalWrite(trig, LOW);
    pinMode(echo, INPUT);
    speed = 8 ;
    gripperVal = 90;
    turn = 8;
    mode = 0;
}

void buttons(){
    /*wheelR.write(offsetR + conf);
    if(button1Val == 0){
        conf ++;
    } else if(button2Val == 0){
        conf --;
    }
    Serial.print(button1Val);
    Serial.print("     ");
    Serial.print(button2Val);
    Serial.print("     ");
    Serial.println(conf);
    */



    button1Val = digitalRead(button1Pin);
    button2Val = digitalRead(button2Pin);
}

long getDistance(){
    digitalWrite(trig, HIGH);
    delayMicroseconds(10);
    digitalWrite(trig, LOW);
    duration = pulseIn(echo, HIGH);
    return duration / 29 / 2;
}

void checkLine(){
    lineL = analogRead(linesensorL);
    lineR = analogRead(linesensorR);
    frontL = analogRead(frontsensorL);
    frontR = analogRead(frontsensorR);

    Serial.print(frontL);
    Serial.print("     ");
    Serial.print(lineL);
    Serial.print("     ");
    Serial.print(lineR);
    Serial.print("     ");
    Serial.print(frontR);
    Serial.print("     ");
    Serial.println(" ");

    if(frontL > 600){
        mode = LEFT; // LEFT MODE
    } else if (frontR > 600){
        mode = RIGHT;
    }
    else {
        mode = NORMAL;
    }
    if(mode != RIGHT && lineL > 600){
        speedL = - turn;
        speedR = turn;
    } else if(mode != LEFT && lineR > 600){
        speedL = turn;
        speedR = - turn;
    } else {
        speedL = speed;
        speedR = speed;
    }
    wheelR.write(offsetR + speedR);
    wheelL.write(offsetL - speedL);

}
void still(){
    wheelR.write(offsetR);
    wheelL.write(offsetL);
}

void loop() {
    //still();
    checkLine();
    delay(100);
}
