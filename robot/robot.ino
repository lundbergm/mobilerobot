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
int maxSpeed = 30;
int mode;
int turnMode;
int countL = 0;
int countR = 0;
int temp = 0;

unsigned long speedLSen = 0.0;
unsigned long holeDist = 20000000; // ändra till riktigt värde

unsigned long timeFrontSensorL;
unsigned long timeFrontSensorR;
unsigned long timeFrontSensorM;
long turnModeTime = 0;

int offsetR = 90;
int offsetL = 90;
int minR = 999;
int maxR = 0;
int minL = 999;
int maxL = 0;
int refR = (maxR+minR)/2;
int refL = (maxL+minL)/2;
int u = 0;
int e = 0;
double kpR = 0.3;
double kpL = 0.3;
double integral = 0;
double ti = 0.025;
double kp = 115.0;
int imax = 30;


void initMotors(){
    //gripper.attach(gripperPin);

    wheelR.attach(motorR);
    wheelL.attach(motorL);
}



void senLfunc(){
    countL ++;
}
void senRfunc(){
    countR ++;
}

void setup() {
    /* Serial */


    pinMode(linesensorL, INPUT);
    pinMode(linesensorR, INPUT);
    pinMode(frontSensorL, INPUT);
    pinMode(frontSensorR, INPUT);
    pinMode(frontSensorM, INPUT);
    //pinMode(trig, OUTPUT);
    //digitalWrite(trig, LOW);
    //pinMode(echo, INPUT);
    /* Enable interrupt */
    pinMode(senL, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(senL), senLfunc, RISING);
    pinMode(senR, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(senR), senRfunc, RISING);


	attachInterrupt(frontSensorL, frontSensorInterruptL, RISING);
	attachInterrupt(frontSensorM, frontSensorInterruptM, HIGH);
	attachInterrupt(frontSensorR, frontSensorInterruptR, RISING);


    speed = 20 ;

    gripperVal = 90;
    turn = 6;
    mode = LEFT;
    //offsetRFunc();
    //offsetLFunc();

    delay(1000);
    initMotors();
    calibrate();

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



void calibrate(){
    wheelR.write(offsetR + 40);
    wheelL.write(offsetL + 40);
    for(int i = 0; i<200; i++){
        lineR = analogRead(linesensorR);
        lineL = analogRead(linesensorL);
        if(lineR < minR){
            minR = lineR;
        }
        if(lineR > maxR){
            maxR = lineR;
        }
        if(lineL < minL){
            minL = lineL;
        }
        if(lineL > maxL){
            maxL = lineL;
        }
        if(i == 50){
            wheelR.write(offsetR - 40);
            wheelL.write(offsetL - 40);
        }
        if(i == 150){
            wheelR.write(offsetR + 40);
            wheelL.write(offsetL + 40);
        }
        delay(20);
    }

    wheelR.write(offsetR);
    wheelL.write(offsetL);

    refR    = (maxR+minR)/2;
    refL    = (maxL+minL)/2;

    kpR = kp/(maxR - minR);
    kpL = kp/(maxL - minL);

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
}


void checkCrossing(){
    frontL = analogRead(frontSensorL);
    frontR = analogRead(frontSensorR);
    frontM = analogRead(frontSensorM);
    //Serial.println(frontL);
    if((frontL > 600 || frontR > 600) && !turnMode){
        turnModeTime = millis();
        turnMode = 0;
        Serial.print("TurnMode: ");
        Serial.println(turnMode);
    }else if(turnMode && ((millis() - turnModeTime) > 1000 )){
        turnMode = 0;
        Serial.print("TurnMode: ");
        Serial.println(turnMode);
    }
}

void servo(int u){
    speedL = 50 + u;
    speedR = 130 + u;

    if(speedR > 130){
        speedR = 130;
    }else if(speedR < 50){
        speedR = 50;
    }
    if(speedL > 130){
        speedL = 130;
    } else if(speedL < 50){
        speedL = 50;
    }
    wheelL.write(speedL);
    wheelR.write(speedR);
}

void run(){

    if(mode == RIGHT){
        lineR = analogRead(linesensorR);
        e = refR - lineR;
        integral += e * ti;
        if(integral > imax) {integral = imax;}
        else if(integral < -imax) {integral = -imax;}
        u = kpR * e + integral;
    } else if(mode == LEFT){
        lineL = analogRead(linesensorL);
        e = refL - lineL;
        integral += e;
        if(integral > imax) {integral = imax;}
        else if(integral < -imax) {integral = -imax;}
        u = -(kpL * e + ti * integral);
    }

    servo(u);

}

void still(){
    wheelR.write(offsetR);
    wheelL.write(offsetL);
}

void loop() {
    //checkLine();

    run();
    //still();
    delay(100);
}
