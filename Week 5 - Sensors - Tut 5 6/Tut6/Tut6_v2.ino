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

const float WHEEL_DISTANCE = 0.182; // Distance between two wheels
const int PULSE_PER_REV = 940;
const float WHEEL_PERIMETER = 0.1413; // Circumference of the wheel

unsigned long previousMillis = 0;
const float T = 0.1; // Sampling rate

// Goal and current pose
float x = 0, y = 0, theta = 0;
float x_g = 2, y_g = 3, theta_g = 0;

// PID control parameters
const float kp = 0.05;
const float ki = 0;
const float kd = 0;
float error, sumError = 0, previousError = 0;

// Control motion variables
float v = 0.1, vr, vl;
float currentError, differenceError; 

int readLineFollower;

// line following sensor pin
#define OUT1 45
#define OUT2 47
#define OUT3 49
#define OUT4 51
#define OUT5 53

// line follower variables
int lineL1;
int lineL2;
int line0;
int lineR1;
int lineR2;

const float alpha = 0.5; //line follower -- weight of outer eyes
const float beta = 0.5; //line follower -- weight of inner eyes
const float gamma = 0.1; //line follower -- weight of the middle eye

void setup() 
{
  Timer1.initialize(500000);    //500ms     
  Timer1.attachInterrupt(readLineFollowerSensor);  
  
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

void readLineFollowerSensor(){ 
  readLineFollower = 1;
}

void loop() 
{
  if (readLineFollower =1) {
    readLineFollower=0;
    getLineState();
    Serial.print(lineL1);  Serial.print(lineL2);  Serial.print(line0);  Serial.print(lineR1);  Serial.println(lineR2);
    Serial.print(vr); Serial.print(";"); Serial.println(vl);
      }
    PID_LineFollower();

    // calculatePIDError_line();
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


// Update the robot's position (x, y, theta)
// void updatePose() {
//   float v1 = RPMtoMPS(calculateRPM(counterLA)); // Left wheel velocity in m/s
//   float v2 = RPMtoMPS(calculateRPM(counterRA)); // Right wheel velocity in m/s
  
//   x += (v1 + v2) / 2 * cos(theta) * T;
//   y += (v1 + v2) / 2 * sin(theta) * T;
//   theta += (v2 - v1) / WHEEL_DISTANCE * T;

//   // Print current pose for debugging
//   Serial.print("X: "); Serial.print(x); Serial.print(", Y: "); Serial.print(y);
//   Serial.print(", Theta: "); Serial.println(theta);

//   // Reset encoder counters
//   resetCounters();
// }

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