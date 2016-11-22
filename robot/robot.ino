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
#define frontSensorL A4
#define frontSensorR A3
#define frontSensorM A2

#define ledL 9
#define ledR 10

int gripperPin = 6;
int trig = 8;
int echo = 7;

int val;    // variable to read the value from the analog pin
int lineL;
int lineR;
int frontL;
int frontR;
int frontM;
long duration, distance;
Servo wheelR;  // create servo object to control a servo
Servo wheelL;
Servo gripper;
int gripperVal;
int speed;
int turn;
int conf = 0;
int speedL = 0;
int speedR = 0;
int maxSpeed = 30;
int mode;
int turnMode;
int identifyMode = 0;
int countL = 0;
int countR = 0;
int temp = 0;

int roadL = 0;
int roadR = 0;
int roadM = 0;

unsigned long speedLSen = 0.0;
unsigned long holeDist = 20000000; // ändra till riktigt värde


long turnModeTime = 0;
long identifyTime = 0;

int threshold = 670;


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
double kpRturn = 0.6;
double kpLturn = 0.6;
double integral = 0;
double ti = 0.025;
double kp = 115.0;
int imax = 35;

char[] turns; // left = 'L', right = 'R', 180 = 'B', Straight = 'S'
int turnsPointer;

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

    pinMode(ledL, OUTPUT);
    pinMode(ledR, OUTPUT);

    pinMode(senL, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(senL), senLfunc, RISING);
    pinMode(senR, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(senR), senRfunc, RISING);

    attachInterrupt(frontSensorL, frontSensorInterruptL, RISING);
    attachInterrupt(frontSensorM, frontSensorInterruptM, FALLING);
    attachInterrupt(frontSensorR, frontSensorInterruptR, RISING);
    turns = ['L', 'B', 'S', 'B', 'R'];
    turnsPointer = 0;

    speed = 20 ;

    gripperVal = 90;
    turn = 6;
    mode = RIGHT;
    turnMode = 0;

    Serial.begin(9600);
    initMotors();
    calibrate();
}


void frontSensorInterruptL(){
	turnsPointer++;
}

void frontSensorInterruptR(){
	turnsPointer++;
}

void frontSensorInterruptM(){
	turnsPointer++;
}

void candlesFound(){
  result char[] = new char[sizeof(turns)];
  boolean done =
  while(i<turnsPointer){
    tempResult char[] = new char[sizeof(turns)];
    for(int i = 0; i < sizeof(turns); i++){
      int sum = turns[i]+turns[i+1]+turns[i+2];
      if(sum == 224 || sum == 232){
        result[i] = 'B';
        i += 2;
      }
      else if(sum == 225){
        result[i] = 'R';
        i += 2;
      }
      else if(sum == 218){
        result[i] = 'S';
        i += 2;
      }
      else{
        result[i] = turns[i];
      }
    }
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

void checkCrossing(){
    frontL = analogRead(frontSensorL);
    frontR = analogRead(frontSensorR);
    frontM = analogRead(frontSensorM);
    //Serial.println(frontL);
    if((frontL > 600 || frontR > 600) && !turnMode){
        turnModeTime = millis();
        turnMode = 1;
        identifyTime = millis();
        identifyMode = 1;
        Serial.println("turnMode");
    }else if(turnMode && ((millis() - turnModeTime) > 1000 )){
        turnMode = 0;
        Serial.println("NOT turnMode");
    }
}

void identify(){

    if(millis() - identifyTime < 300){
        frontL = analogRead(frontSensorL);
        frontR = analogRead(frontSensorR);
        frontM = analogRead(frontSensorM);
        if(frontL > threshold){
            roadL = 1;
        }
        if(frontR > threshold){
            roadR = 1;
        }
        if(front > threshold){
            roadM = 1;
        }
    }else if(roadL){
        mode = LEFT;
    }else if(roadR){
        mode = RIGHT;
    }
}

void LorR(){
    int threshold = 670;

    frontL = analogRead(frontSensorL);
    frontR = analogRead(frontSensorR);
    if(frontL > threshold){
        mode = LEFT;
    }
    if(frontR > threshold){
        mode = RIGHT;
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
        if(turnMode){
            u = kpR * e + integral;
        }else{
            u = kpR * e + integral;
        }
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
    wheelR.write(90);
    wheelL.write(90);
}

void loop() {

    //checkLine();
    if(turns[turnsPointer] == 'L'){
      mode = LEFT;
    }
    elseif(turns[turnsPointer] == 'R'){
      mode = RIGHT;
    }
    elseif(turns[turnsPointer] == 'S'){
      //typ en delay där vi kör framåt i någon sekund
    }



    LorR();
    checkCrossing();
    if(turnMode){
        identify();
    }

    run();

    //still();
    delay(100);
}
