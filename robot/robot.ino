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

#define ledL 11
#define ledM 10
#define ledR 12

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
long frontTime = 0;
long testTimer;
long chillTime;

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
double deltaR;
double deltaL;
double integral = 0;
double ti = 0.035;      //should be 0.035
double kp = 115.0;      // should be 115
int imax = 20;//35;          // should be 35
double ti_turn = 0.0015;      //should be 0.035
double kp_turn = 75.0;      // should be 115
int imax_turn = 10;          // should be 35
double ti_s = 0.00;      //should be 0.035
double kp_s = 75.0;      // should be 115 // 95
int imax_s = 0;          // should be 35

int maxSpeed = 40;

char turns[] = {'L', 'B', 'S', 'B', 'R'}; // left = 'L', right = 'R', 180 = 'B', Straight = 'S'
int turnsPointer;

int inst[] = {'R', 'U', 'L', 'U', 'R', 'U', 'L', 'U', 'R', 'U', 'L', 'U', 'R', 'U', 'L', 'U'};
int instC = 0;

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

    Serial.begin(9600);
    pinMode(linesensorL, INPUT);
    pinMode(linesensorR, INPUT);
    pinMode(frontSensorL, INPUT);
    pinMode(frontSensorR, INPUT);
    pinMode(frontSensorM, INPUT);

    pinMode(ledL, OUTPUT);
    pinMode(ledM, OUTPUT);
    pinMode(ledR, OUTPUT);

    pinMode(senL, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(senL), senLfunc, RISING);
    pinMode(senR, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(senR), senRfunc, RISING);

    attachInterrupt(frontSensorL, frontSensorInterruptL, RISING);
    attachInterrupt(frontSensorM, frontSensorInterruptM, FALLING);
    attachInterrupt(frontSensorR, frontSensorInterruptR, RISING);

    turnsPointer = 0;

    speed = 20 ;

    gripperVal = 90;
    turn = 6;
    mode = RIGHT;
    turnMode = 0;

    initMotors();
    calibrate();
    testTimer = millis();
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
/*
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
*/
void calibrate(){
    digitalWrite(ledM, HIGH);
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
    deltaR = (maxR - minR);
    deltaL = (maxL - minL);
    kpR = kp/deltaR;
    kpL = kp/deltaL;
    digitalWrite(ledM, LOW);
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
    if(frontM > threshold){
        frontTime = millis();
    }

    if(((frontL > 600 || frontR > 600 || (millis() - frontTime) > 500) && !turnMode && (millis() - chillTime) > 800 )){
        digitalWrite(ledM, HIGH);
        turnModeTime = millis();
        turnMode = 1;
        maxSpeed = 16;
        identifyTime = millis();
        identifyMode = 1;
        roadL = 0;
        roadR = 0;
        roadM = 0;
        //digitalWrite(ledL, LOW);
        //digitalWrite(ledR, LOW);
        kp = kp_turn;
        ti = ti_turn;
        imax = imax_turn;

    }else if(turnMode && ((millis() - turnModeTime) > 1000 ) && (frontM > threshold)){
        digitalWrite(ledM, LOW);
        turnMode = 0;
        maxSpeed = 40;
        kp = kp_s;
        ti = ti_s;
        imax = imax_s;
        if(inst[instC % 16] == 'R'){
            mode = RIGHT;
            digitalWrite(ledR, HIGH);
            digitalWrite(ledL, LOW);
        }
        else if(inst[instC % 16] == 'L'){
            mode = LEFT;
            digitalWrite(ledL, HIGH);
            digitalWrite(ledR, LOW);
        } else if(inst[instC % 16] == 'U'){
            if(inst[(instC + 1) % 16] == 'R'){
                mode = RIGHT;
                digitalWrite(ledR, HIGH);
                digitalWrite(ledL, LOW);
            }
            else if(inst[(instC + 1) % 16] == 'L'){
                mode = LEFT;
                digitalWrite(ledL, HIGH);
                digitalWrite(ledR, LOW);
            }
        }

        instC++;
        chillTime = millis();
    }
}

void identify(){

    if(millis() - identifyTime < 300){
        frontL = analogRead(frontSensorL);
        frontR = analogRead(frontSensorR);
        frontM = analogRead(frontSensorM);
        if(frontL > threshold){
            roadL = 1;
            //digitalWrite(ledL, HIGH);
        }
        if(frontR > threshold){
            roadR = 1;
            //digitalWrite(ledR, HIGH);
        }
        if(frontM > threshold){
            roadM = 1;
        }
    }
}

void LorR(){
    if((millis()/1000 % 2) == 0){
        mode = LEFT;
        digitalWrite(ledL, HIGH);
        digitalWrite(ledR, LOW);
    } else{
        mode = RIGHT;
        digitalWrite(ledL, LOW);
        digitalWrite(ledR, HIGH);
    }
}


void servo(int u){
    speedL = 90 - maxSpeed + u;
    speedR = 90 + maxSpeed + u;

    if(speedR > (90 + maxSpeed)){
        speedR = 90 + maxSpeed;
    }else if(speedR < (90 - maxSpeed)){
        speedR = 90 - maxSpeed;
    }
    if(speedL > (90 + maxSpeed)){
        speedL = 90 + maxSpeed;
    } else if(speedL < 90 - maxSpeed){
        speedL = 90 - maxSpeed;
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
        u = kp/deltaR * e + integral;
        Serial.print("u: ");
        Serial.print(u);
        Serial.print("  kp: ");
        Serial.print(kp);
        Serial.print("  deltaR: ");
        Serial.print(deltaR);
        Serial.print("  kp/deltaR: ");
        Serial.print(kp/deltaR);
        Serial.print("  I: ");
        Serial.print(integral);
        Serial.print("  e: ");
        Serial.println(e);

    } else if(mode == LEFT){
        lineL = analogRead(linesensorL);
        e = refL - lineL;
        integral += e * ti;
        if(integral > imax) {integral = imax;}
        else if(integral < -imax) {integral = -imax;}
        u = -(kp/deltaL * e + integral);
    }

    servo(u);

}

void still(){
    wheelR.write(90);
    wheelL.write(90);
}

void loop() {

    //checkLine();
    /*
    if(turns[turnsPointer] == 'L'){
      mode = LEFT;
    }
    elseif(turns[turnsPointer] == 'R'){
      mode = RIGHT;
    }
    elseif(turns[turnsPointer] == 'S'){
      //typ en delay där vi kör framåt i någon sekund
  }*/

    //LorR();
    checkCrossing();
    if(turnMode){
        identify();
    }
    //LorR();
    run();

    //still();
    delay(100);
}
