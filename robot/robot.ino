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

unsigned long speedLSen = 0.0;
unsigned long holeDist = 20000000; // ändra till riktigt värde
int offsetR = 90;
int offsetL = 90;
int min = 999;
int max = 0;
int ref = (max+min)/2;
int u = 0;
int e = 0;
double kp = 0.3;




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
    Serial.begin(9600);

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

    speed = 10 ;
    gripperVal = 90;
    turn = 6;
    mode = RIGHT;
    //offsetRFunc();
    //offsetLFunc();
    Serial.println("SETUP");
    delay(1000);
    initMotors();
    calibrate();
    Serial.println("ALL DONE");
}

void calibrate(){
    wheelR.write(offsetR + 40);
    wheelL.write(offsetL + 40);
    for(int i = 0; i<200; i++){
        lineR = analogRead(linesensorR);
        lineL = analogRead(linesensorL);
        if(lineR < min){
            min = lineR;
        }
        if(lineR > max){
            max = lineR;
        }
        if(i == 50){
            wheelR.write(offsetR - 20);
            wheelL.write(offsetL - 20);
        }
        if(i == 150){
            wheelR.write(offsetR + 20);
            wheelL.write(offsetL + 20);
        }
        delay(20);
    }

    wheelR.write(offsetR);
    wheelL.write(offsetL);
    Serial.println("max");
    Serial.println(max);
    Serial.println("min");
    Serial.println(min);
    ref    = (max+min)/2;
    Serial.println("ref    ");
    Serial.println(ref);
    kp = 80.0/(max - min);
    Serial.println("delta");
    Serial.println(max- min);
    Serial.println("kp");
    Serial.println(kp);
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

    Serial.print("L: ");
    Serial.print(lineL);
    Serial.print("     R: ");
    Serial.println(lineR);


}

void servo(int u){
    if((offsetR + speed + u) > 180){
        wheelR.write(180);
    }else if((offsetR + speed + u) < 0){
        wheelR.write(0);
    } else {
        wheelR.write(offsetR + speed + u);
    }
    if((offsetL - speed + u)> 180){
        wheelL.write(180);
    } else if((offsetL - speed + u)< 0){
        wheelL.write(0);
    }else {
        wheelL.write(offsetL - speed + u);
    }
    Serial.print("R: ");
    Serial.print(offsetR + speed + u);
    Serial.print("     R: ");
    Serial.println(offsetR - speed + u);
}



void run(){

    lineR = analogRead(linesensorR);
    e = ref - lineR;
    Serial.print("R: ");
    Serial.print(lineR);
    Serial.print("      e: ");
    Serial.print(e);
    u = kp * e;

    Serial.print("      u: ");
    Serial.println(u);
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
