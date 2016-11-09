#include <Servo.h>
#include <PinChangeInterrupt.h>
#include <PinChangeInterruptBoards.h>
#include <PinChangeInterruptPins.h>
#include <PinChangeInterruptSettings.h>
#include <Time.h>

/** DEFINE **/
#define offsetR 88
#define offsetL 90

/** PIN **/
#define linesensorL A0  // analog pin used to connect the potentiometer
#define linesensorR A1
#define senL 2
#define senR 3
#define motorL 5
#define motorR 6

int gripperPin = 6;
int trig = 8;
int echo = 7;

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
int countL = 0;
int temp = 0;
unsigned long time1 = 0;
unsigned long time2 = 0;
double time3 = 0;
unsigned long speedLSen = 0.0;
unsigned long holeDist = 20000000; // ändra till riktigt värde


void initMotors(){
    //gripper.attach(gripperPin);

    wheelR.attach(motorR);
    wheelL.attach(motorL);
}

void senLfunc(){

    if(countL== 11 ){
        time2 = millis();
        time3 =time2 -time1;
        speedLSen = holeDist/time3;
        Serial.println(time3);
        Serial.println(speedLSen);
        countL = 0;
        time1 = millis();
    }

    countL ++;

}

void setup() {
    /* Serial */
    Serial.begin(9600);

    //initMotors();
    pinMode(linesensorL, INPUT);
    pinMode(linesensorR, INPUT);

    pinMode(trig, OUTPUT);

    digitalWrite(trig, LOW);
    //pinMode(echo, INPUT);
    /* Enable interrupt */
    pinMode(senL, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(senL), senLfunc, RISING);
    pinMode(senR, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(senR), senRfunc, RISING);

    speed = 30 ;
    gripperVal = 90;
    turn = 6;
    mode = 0;
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
    //frontL = analogRead(frontsensorL);
    //frontR = analogRead(frontsensorR);

    //Serial.print(frontL);
    //Serial.print("     ");
    Serial.print(lineL);
    Serial.print("     ");
    Serial.print(lineR);
    Serial.print("     ");
    //Serial.print(frontR);
    //Serial.print("     ");
    Serial.println(" ");
    /*
    if(frontL > 600){
        mode = LEFT; // LEFT MODE
    } else if (frontR > 600){
        mode = RIGHT;
    }
    else {
        mode = NORMAL;
    }*/
    if(lineL > 600){
        speedL = - turn;
        speedR = turn;
    } else if(lineR > 600){
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
    //checkLine();
    //temp = digitalRead(senL);
    //Serial.println(temp);
    delay(100);
}
