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
#define gripperPin 9
#define flipperPin 11
#define frontSensorL A4
#define frontSensorR A3
#define frontSensorM A2

#define flameSensor A5

#define ledL 13
#define ledM 10
#define ledR 12
#define triggerPin 4
#define echoPin 7

//int gripperPin = 6;


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
Servo flipper;
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
long gripperTimer;
long flipperTimer;

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

int flame = 0;
int gripperMode = 0;
int candleCount = 0;

void initMotors(){
    gripper.attach(gripperPin);

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
    pinMode(flameSensor, INPUT);

    pinMode(ledL, OUTPUT);
    pinMode(ledM, OUTPUT);
    pinMode(ledR, OUTPUT);

    pinMode(senL, INPUT_PULLUP);
    //attachInterrupt(digitalPinToInterrupt(senL), senLfunc, RISING);
    pinMode(senR, INPUT_PULLUP);
    pinMode(triggerPin, OUTPUT);
    pinMode(echoPin, INPUT);
    //attachInterrupt(digitalPinToInterrupt(senR), senRfunc, RISING);

    //attachInterrupt(frontSensorL, frontSensorInterruptL, RISING);
    //attachInterrupt(frontSensorM, frontSensorInterruptM, FALLING);
    //attachInterrupt(frontSensorR, frontSensorInterruptR, RISING);

    turnsPointer = 0;

    speed = 20 ;

    gripperVal = 90;
    turn = 6;
    mode = RIGHT;
    turnMode = 0;

    initMotors();
    gripper.attach(gripperPin);
    flipper.attach(flipperPin);
    flipper.write(85);
    gripper.write(85);
    calibrate();
    testTimer = millis();
    gripper.detach();
    flipper.detach();
    digitalWrite(ledL, LOW);
    digitalWrite(ledM, LOW);
    digitalWrite(ledR, LOW);
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
    digitalWrite(triggerPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(triggerPin, LOW);
    duration = pulseIn(echoPin, HIGH);
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
        mode = LEFT;
        /*
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
        */
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

void candle(){
    //flame = analogRead(flameSensor);
    //flame = digitalRead(flameSensor);
    if(gripperMode == 0){
        digitalWrite(triggerPin, HIGH);
        delayMicroseconds(10);
        digitalWrite(triggerPin, LOW);
        duration = pulseIn(echoPin, HIGH);
        if(duration < 270){
            gripper.attach(gripperPin);
            gripper.write(110);
            gripperMode = 1;
            gripperTimer = millis();
        }
    } else if(gripperMode == 1){
        if(millis() - gripperTimer > 200){
            flipper.attach(flipperPin);
            flipper.write(100);
            gripperMode = 2;
            flipperTimer = millis();
        }
    } else if(gripperMode == 2){
        if(millis() - flipperTimer > 2000){
            gripper.write(80);
            gripperMode = 3;
            gripperTimer = millis();
        }
    }else if(gripperMode == 3){
        if(millis() - gripperTimer > 300){
            gripper.write(87);
            flipper.write(85);
            gripper.detach();
            flipperTimer = millis();
            gripperMode = 4;
            /*
            candleCount ++;
            if((candleCount % 4) == 0){
                digitalWrite(ledL, LOW);
                digitalWrite(ledM, LOW);
                digitalWrite(ledR, LOW);
            }else if((candleCount % 4) == 1){
                digitalWrite(ledL, HIGH);
            }
            else if((candleCount % 4) == 2){
                digitalWrite(ledM, HIGH);
            }else if((candleCount % 4) == 3){
                digitalWrite(ledR, HIGH);
            }*/

        }
    }else if(gripperMode == 4){
        if(millis() - flipperTimer > 2000){
            flipper.write(87);
            flipper.detach();
            gripperMode = 0;
        }
    }
}

void printSensors(){
    Serial.print("frontL: ");
    Serial.print(analogRead(frontSensorL));
    Serial.print("  frontM: ");
    Serial.print(analogRead(frontSensorM));
    Serial.print("  frontR: ");
    Serial.print(analogRead(frontSensorR));
    Serial.print("  L: ");
    Serial.print(analogRead(linesensorL));
    Serial.print("  R: ");
    Serial.println(analogRead(linesensorR));
}

void loop() {

    checkCrossing();
    /*
    if(turnMode){
        identify();
    }*/
        candle();
    run();
    //printSensors();
    //still();
    delay(100);
}
