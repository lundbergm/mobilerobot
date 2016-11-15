#include <Servo.h>
#include <PinChangeInterrupt.h>
#include <PinChangeInterruptBoards.h>
#include <PinChangeInterruptPins.h>
#include <PinChangeInterruptSettings.h>
#include <Time.h>

/** DEFINE **/
#define RIGHT 1
#define LEFT 2
#define STRAIGHT 3

/** PIN **/
#define linesensorL A0  // analog pin used to connect the potentiometer
#define linesensorR A1
#define senL 2
#define senR 3
#define motorL 5
#define motorR 6
#define frontSensorL 7
#define frontSensorR 8
#define frontSensorM 9

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
int countR = 0;
int temp = 0;
unsigned long time1 = 0;
unsigned long time2 = 0;
double time3 = 0;
unsigned long speedLSen = 0.0;
unsigned long holeDist = 20000000; // ändra till riktigt värde
int offsetR = 80;
int offsetL = 80;
int minVal = 50;
int maxVal = 900;
int ref    = (900+50)/2;
unsigned long timeFrontsensorL;
unsigned long timeFrontsensorR;
unsigned long timeFrontsensorM;


void initMotors(){
    //gripper.attach(gripperPin);

    wheelR.attach(motorR);
    wheelL.attach(motorL);
}

void offsetRFunc(){

    do{
        offsetR++;
        wheelR.write(offsetR);
        countR = 0;
        delay(1000);
        Serial.println(countR);

    }while(countR != 0);
    Serial.print("offsetR: ");
    Serial.println(offsetR);
}

void offsetLFunc(){

    do{
        offsetL++;
        wheelL.write(offsetL);
        countL = 0;
        delay(1000);
        Serial.println(countL);
    }while(countL != 0);
    Serial.print("offsetL: ");
    Serial.println(offsetL);
}

void senLfunc(){
    countL ++;
}
void senRfunc(){
    countR ++;
}

void setup() {
    /* Serial */
    Serial.begin(9600);

    initMotors();
    pinMode(linesensorL, INPUT);
    pinMode(linesensorR, INPUT);
    //pinMode(trig, OUTPUT);
    //digitalWrite(trig, LOW);
    //pinMode(echo, INPUT);
    /* Enable interrupt */
    pinMode(senL, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(senL), senLfunc, RISING);
    pinMode(senR, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(senR), senRfunc, RISING);

//tillagt av bigmike

	attachInterrupt(frontSensorL, frontSensorInterruptL, RISING);
	attachInterrupt(frontSensorM, frontSensorInterruptM, HIGH);
	attachInterrupt(frontSensorR, frontSensorInterruptR, RISING);


    speed = 30 ;
    gripperVal = 90;
    turn = 6;
    mode = 0;
    //offsetRFunc();
    //offsetLFunc();
    Serial.println("ALL DONE");
}

void frontSensorInterruptL(){
	timeFrontSensorL = millis();

	if(abs(timeFrontSensorL - timeFrontSensorR) < 100 || abs(timeFrontSensorL - timeFrontSensorM) < 100){
    //regga sväng.
	}
	else{
		//regga inte sväng
	}
}

void frontSensorInterruptR(){
	timeFrontSensorR = millis();

	if(abs(timeFrontSensorR - timeFrontSensorL) < 100 || abs(timeFrontSensorL - timeFrontSensorM) < 100){
    //regga sväng.
	}
	else{
		//regga inte sväng
	}
}

void frontSensorInterruptM(){
	timeFrontSensorM = millis();

  if(abs(timeFrontSensorM - timeFrontSensorR) < 100 || abs(timeFrontSensorM - timeFrontSensorL) < 100){
		//regga sväng.
	}
	else{
		//regga inte sväng
	}
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

    Serial.print(lineL);
    Serial.print("     ");
    Serial.println(lineR);
    if(lineL > 600 && lineR > 600){
        turn90(LEFT);
    } else if(lineL > 600){
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

void servo(int u){
    wheelR.write(offsetR + speedR + u);
    wheelL.write(offsetL - speedL + u);
}

void run(){

    lineL = analogRead(linesensorL);
    e = ref - lineL;
    u = kp * e;

}

void still(){
    wheelR.write(offsetR);
    wheelL.write(offsetL);
}

void loop() {
    //checkLine();

    if(ti)

    run();
    delay(100);
}

/** Antal counts du vill att hjulet ska snurra, forward = true get framåt*/
void moveWheelLCount(int count, boolean forward){
    countL = 0;
    Serial.println("moveWheelLCount");

    while(countL<count && forward){
        wheelL.write(offsetL+speedL);
    }

    count = -count;
    while(countL<count && !forward){
        wheelL.write(offsetL-speedL);
    }
}

void moveWheelRCount(int count, boolean forward){
    countR = 0;
    Serial.println("moveWheelRCount");

    while(countR<count && forward){
        wheelR.write(offsetR+speedR);
    }

    while(countR<count && !forward){
        wheelR.write(offsetR-speedR);
    }
}


void turn90(int direction){
    countL = 0;
    countR = 0;
    if(direction == LEFT){
        wheelR.write(offsetR + 10);
        wheelL.write(offsetL);
        while(countR < 33){
            Serial.print("L: ");
            Serial.print(countL);
            Serial.print("  R: ");
            Serial.println(countR);
        }
    } else if (direction == RIGHT){
        wheelR.write(offsetR);
        wheelL.write(offsetL - 10);
        while(countL < 33){
            Serial.print("L: ");
            Serial.print(countL);
            Serial.print("  R: ");
            Serial.println(countR);
        }
    }
    wheelR.write(offsetR);
    wheelL.write(offsetL);
}

void turn90Left(){
    double hjulOmkrets = 163; // mm
    double sensHjulDist = 30; // mm
    double halIHjul = 20;

    //kör fram avstånd mellan hjul och sensor
    while(countL<=(int)(sensHjulDist/hjulOmkrets*halIHjul)){
        wheelR.write(100);
        wheelL.write(80);
    }

    wheelR.write(offsetR);
    wheelL.write(offsetL);

    //vänd 90 grader vänster

    moveWheelRCount((int)(127/hjulOmkrets*halIHjul), true);
    moveWheelLCount((int)(127/hjulOmkrets*halIHjul), false);

    //kör fram samma avstånd

    while(countL<=(int)(sensHjulDist/hjulOmkrets*halIHjul)){
        wheelR.write(100);
        wheelL.write(80);
    }

}

void turn90Right(){
    Serial.println("turn90Right");
    double hjulOmkrets = 163; // mm
    double sensHjulDist = 30; // mm
    double halIHjul = 20;

    //kör fram avstånd mellan hjul och sensor
    countL = 0;
    while(countL<=(int)(sensHjulDist*halIHjul/hjulOmkrets)){
        wheelR.write(offsetR + 10);
        wheelL.write(offsetL - 10);
        Serial.print("CountL: ");
        Serial.println(countL);


    }
    Serial.println("Kört fram");
    //stå still
    wheelR.write(offsetR);
    wheelL.write(offsetL);
    Serial.println("Stå still");
    delay(1000);
    //vänd 90 grader vänster

    moveWheelRCount((int)(127*halIHjul)/hjulOmkrets, false);
    moveWheelLCount((int)(127*halIHjul)/hjulOmkrets, true);

    //kör fram samma avstånd
    countL = 0;
    while(countL<=(int)(sensHjulDist*halIHjul/hjulOmkrets)){
        wheelR.write(offsetR + 10);
        wheelL.write(offsetL - 10);
    }

}
