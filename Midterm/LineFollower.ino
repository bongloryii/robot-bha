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

void setupLineFollower() 
{
  Timer1.initialize(500000);    //500ms     
  Timer1.attachInterrupt(readLineFollowerSensor);  
  
  //Set up line follower sensor
  pinMode(OUT1, INPUT);
  pinMode(OUT2, INPUT);
  pinMode(OUT3, INPUT);
  pinMode(OUT4, INPUT);
  pinMode(OUT5, INPUT);

}

void readLineFollowerSensor(){ 
  readLineFollower = 1;
}

void FollowLine() //loop
{
  if (readLineFollower=1) {
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
  setSpeed(vr,vl);
  
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