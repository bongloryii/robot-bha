// Sensor Array 	Error Value
// 0 0 0 0 1	 4              
// 0 0 0 1 1	 3              
// 0 0 0 1 0	 2              
// 0 0 1 1 0	 1              
// 0 0 1 0 0	 0              
// 0 1 1 0 0	-1              
// 0 1 0 0 0	-2              
// 1 1 0 0 0	-3              
// 1 0 0 0 0	-4              

// 1 1 1 1 1        0 Robot found continuous line : STOPPED
// 0 0 0 0 0        0 Robot found no line: turn 180o

#include "TimerOne.h"
//use timer to read imu data every 500ms
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include "TimerOne.h"

extern Adafruit_BNO055 bno;

// Define motor pins and encoders
int motorRpin1 = 10; // Right motor IN3
int motorRpin2 = 11; // Right motor IN4//few 

int motorLpin1 = 8; // Left motor IN1
int motorLpin2 = 9; // Left motor IN2//fw

int enR = 13;  // Right motor enable pin (PWM)
int enL = 7; // Left motor enable pin (PWM)

// Encoders
int enLA = 2;  // Left motor encoder A
int enRA = 3; // Right motor encoder A

volatile unsigned int counterLA = 0;
volatile unsigned int counterRA = 0;

// Ultrasonic
int echoPin1 = 51; // Front ultrasonic
int trigPin1 = 50;
int echoPin2 = 37; // Left ultrasonic
int trigPin2 = 36;
// int echoPin3 = 53; // Right ultrasonic
// int trigPin3 = 52;

// volatile float maxFront = 15;
volatile float duration1, distance1, duration2, distance2;
volatile float maxDistance = 15;

const float WHEEL_DISTANCE = 0.182; // Distance between two wheels
const int PULSE_PER_REV = 940;
const float WHEEL_PERIMETER = 0.1413; // Circumference of the wheel

unsigned long previousMillis = 0;
const float T = 0.1; // Sampling rate

// Goal and current pose
float x = 0, y = 0, theta = 0;
float x_g = 1, y_g = 1, theta_g = 3.13;
int isReadIMU = 0;

// PID control parameters
const float kp = 0.05;
const float ki = 0;
const float kd = 0;
float error, sumError = 0, previousError = 0;
float v = 0.15, vr, vl;
float currentError, differenceError; 

// line following sensor pin
#define OUT1 41
#define OUT2 43
#define OUT3 45
#define OUT4 47
#define OUT5 49

// line follower variables
int lineL1;
int lineL2;
int line0;
int lineR1;
int lineR2;

const float alpha = 0.5; //line follower -- weight of outer eyes
const float beta = 0.5; //line follower -- weight of inner eyes
const float gamma = 0.1; //line follower -- weight of the middle eye

int readLineFollower;
int start = 0;
int target = 0;

void setup() 
{
  Timer1.initialize(500000);    //500ms     
  Timer1.attachInterrupt(readLineFollowerSensor);  

  setupIMU();
  ultraSetup();
  
  Serial.begin(9600);
  //Set up line follower sensor
  pinMode(OUT1, INPUT);
  pinMode(OUT2, INPUT);
  pinMode(OUT3, INPUT);
  pinMode(OUT4, INPUT);
  pinMode(OUT5, INPUT);

  // Set up motor and encoder pins
  pinMode(motorRpin1, OUTPUT);
  pinMode(motorRpin2, OUTPUT);
  pinMode(motorLpin1, OUTPUT);
  pinMode(motorLpin2, OUTPUT);
  
  pinMode(enLA, INPUT);
  pinMode(enRA, INPUT);

  attachInterrupt(digitalPinToInterrupt(enLA), countEnLA, RISING);
  attachInterrupt(digitalPinToInterrupt(enRA), countEnRA, RISING);

}

void ultraSetup() {
   // Set up 3 ultrasonic sensors
  pinMode(trigPin1, OUTPUT);
  pinMode(echoPin1, INPUT);
  pinMode(trigPin2, OUTPUT);
  pinMode(echoPin2, INPUT);
  // pinMode(trigPin3, OUTPUT);
  // pinMode(echoPin3, INPUT);
}

void readLineFollowerSensor(){ 
  readLineFollower = 1;
}

void loop() 
{
  if (readLineFollower == 1) {
    readLineFollower=0;
    getLineState();
    PID_LineFollower();
    obstacleAvoid();
  }
  
}

void checkFrontDistance() {
  
  digitalWrite(trigPin2, LOW);
  digitalWrite(echoPin2, LOW);

  digitalWrite(trigPin1, LOW);
  delayMicroseconds(5);
  digitalWrite(trigPin1, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin1, LOW);
  duration1 = pulseIn(echoPin1, HIGH);
  Serial.println(duration1);
  distance1 = duration1 * 0.037 / 2;
  delay(60); 
}

void checkLeftDistance() {
  
  digitalWrite(trigPin1, LOW);
  digitalWrite(echoPin1, LOW);

  digitalWrite(trigPin2, LOW);
  delayMicroseconds(5);
  digitalWrite(trigPin2, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin2, LOW);
  duration2 = pulseIn(echoPin2, HIGH);
  distance2 = duration2 * 0.037 / 2;
  delay(60);
}

// void checkRightDistance() {
  
//   digitalWrite(trigPin3, LOW);
//   delayMicroseconds(5);
//   digitalWrite(trigPin3, HIGH);
//   delayMicroseconds(10);
//   digitalWrite(trigPin3, LOW);
//   duration3 = pulseIn(echoPin3, HIGH);
//   distance3 = duration3 * 0.037 / 2;
//   delay(60);
// }

bool isObstacleInFront = false;

void obstacleAvoid() {
  // front distance check
  checkFrontDistance();
    // Serial.print("Distance 1 2 3:");
    // checkFrontDistance();
    // // checkLeftDistance();
    // // checkRightDistance();
    // Serial.println(distance1);
    // Serial.println(distance2);
    // Serial.println(distance3);
  if (distance1 < maxDistance){
    turnRight(); // xe xoay 90 do sang phai
    
    isObstacleInFront = true;
    // Serial.print("distance1 < maxDistance");
    //     Serial.println(distance2);
    checkLeftDistance();
    if (distance2 < maxDistance) {
        rightForward(v);
        leftForward(v);
        delay(1000);   
        } 
    }

    if(lineL1 == true || lineL2 == true || line0 == true || lineR1 == true || lineR2 == true){
      Serial.println(line0);
      isObstacleInFront = false;
    }

    if (isObstacleInFront == true) {
      // Serial.print("isObstacleInFront");
      // Serial.println(isObstacleInFront);
      checkLeftDistance();
      if (distance2 > maxDistance) {
        turnLeft();
        rightForward(v);
        leftForward(v);
        delay(1000); 
      }
    }
}

void getLineState() {
  lineL1 = digitalRead(OUT1);
  lineL2 = digitalRead(OUT2);
  line0 = digitalRead(OUT3);
  lineR1 = digitalRead(OUT4);
  lineR2 = digitalRead(OUT5);
}

void PID_LineFollower(){
  // Update error for PID
  calculatePIDError_line();

  // Control motor speeds
  if (vr > 0) {rightForward(vr);}
  else {rightBackward(-vr);}
  
  if (vl > 0) {leftForward(vl);}
  else {leftBackward(-vl);}
  
}

void calculatePIDError_line(){
  previousError = currentError;
  currentError = -alpha * lineL2 - beta * lineL1 + beta * lineR1 + alpha * lineR2;
  Serial.print("Current error:");Serial.println(currentError);
  if (line0 != 1) {
    currentError += (gamma) * (line0 + 1);
  }
  differenceError = currentError - previousError;
  sumError += currentError;

  // Calculate PID control output (error) for velocity difference between wheels
  error = kp * currentError + ki * sumError + kd * differenceError;
  
  // Adjust wheel velocities
  vr = v - error;
  vl = v + error;
}

// Motor control functions
void rightForward(float velocity) {
  int pwm = MPStoPWM(velocity);
  // Serial.print("pwmr:"); Serial.println(pwm);
  digitalWrite(motorRpin1, LOW);
  digitalWrite(motorRpin2, HIGH);
  analogWrite(enR, pwm);
}

void leftForward(float velocity) {
  int pwm = MPStoPWM(velocity);
    // Serial.print("pwml:"); Serial.println(pwm);
  digitalWrite(motorLpin1, LOW);
  digitalWrite(motorLpin2, HIGH);
  analogWrite(enL, pwm);
}

void rightBackward(float velocity) {
  int pwm = MPStoPWM(velocity);
  digitalWrite(motorRpin1, HIGH);
  digitalWrite(motorRpin2, LOW);
  analogWrite(enR, pwm);
}

void leftBackward(float velocity) {
  int pwm = MPStoPWM(velocity);
  digitalWrite(motorLpin1, HIGH);
  digitalWrite(motorLpin2, LOW);
  analogWrite(enL, pwm);
}


void turnLeft() {
  readIMU_pos();
  sensors_event_t event;
  bno.getEvent(&event);
  start = event.orientation.x;
  target = start + 85;
  digitalWrite(motorRpin1, HIGH);
  digitalWrite(motorRpin2, LOW);
  analogWrite(enR, 150);
  while(start < target)
    {
      readIMU_pos();
      start = event.orientation.x;
      Serial.println(start);
      Serial.println(theta);
    }
    stop();
}

void turnRight() {
  readIMU_pos();
  sensors_event_t event;
  bno.getEvent(&event);
  start = event.orientation.x;
  target = start + 85;
  digitalWrite(motorLpin1, HIGH);
  digitalWrite(motorLpin2, LOW);
  analogWrite(enL, 150);
  while(start < target)
    {
      readIMU_pos();
      start = event.orientation.x;
      Serial.println(start);
      Serial.println(theta);
    }
    stop();
}

// Stop motors
void stop() {
  digitalWrite(motorLpin1, LOW);
  digitalWrite(motorLpin2, LOW);
  digitalWrite(motorRpin1, LOW);
  digitalWrite(motorRpin2, LOW);
}

// Helper functions
float calculateRPM(int pulseCounter) {
  return (float)pulseCounter * 60 / PULSE_PER_REV / T;
}

float RPMtoMPS(float rpm) {
  return rpm / 60 * WHEEL_PERIMETER;
}

int MPStoPWM(float velocity) {
  return constrain((velocity / 0.2) * 255, 30, 255); // Mapping speed to PWM
}

void countEnLA() {
  counterLA++;
}

void countEnRA() {
  counterRA++;
}

void resetCounters() {
  counterLA = 0;
  counterRA = 0;
}
