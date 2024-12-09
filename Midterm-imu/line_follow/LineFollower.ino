// 1 1 1 1 1        0 Robot found continuous line : STOPPED
// 0 0 0 0 0        0 Robot found no line: turn 180o

#include "TimerOne.h"

// unsigned long previousMillis = 0;
// const float T = 0.1; // Sampling rate

// Goal and current pose
// float x = 0, y = 0, theta = 0;
// float x_g = 2, y_g = 3, theta_g = 0;

// PID control parameters
const float kp_line = 0.13;
const float ki_line = 0.008;
const float kd_line = 0.0;
float error, sumError = 0, previousError = 0;

// Control motion variables
float v = 0.15, vr, vl;
float currentError, differenceError; 

int readLineFollower;

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

const float alpha = 0.6; //line follower -- weight of outer eyes
const float beta = 0.2; //line follower -- weight of inner eyes
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
unsigned long stopTestDuration = 0;
unsigned long currentMillis = 0;
// unsigned long previousMillis;
void FollowLine() //loop
{
  if (readLineFollower=1) {
    readLineFollower=0;
    getLineState();
        Serial.print(lineL1);  Serial.print(lineL2);  Serial.print(line0);  Serial.print(lineR1);  Serial.println(lineR2);
}
    Serial.print(vr); Serial.print(";"); Serial.println(vl);
      if ((lineL1 ==1) & (lineL2==1) & (lineR1 ==1) & (lineR2==1) & (line0 ==1)) {
      stop();
      delay(3000);
    }
  // Update error for PID
    calculatePIDError_line();
    // Control motor speeds
    setSpeed(vr,vl);
}

void getLineState() {
  lineL1 = digitalRead(OUT1);
  lineL2 = digitalRead(OUT2);
  line0 = digitalRead(OUT3);
  lineR1 = digitalRead(OUT4);
  lineR2 = digitalRead(OUT5);
}
void calculatePIDError_line(){
  previousError = currentError;
  currentError = - alpha * lineL2 - beta * lineL1 + beta * lineR1 + alpha * lineR2 + gamma * (1-line0);
  
  // currentError = alpha * (1-lineL2) + beta * (1-lineL1) - beta * (1-lineR1) - alpha * (1-lineR2) + (gamma) * (1-line0);
  Serial.print("Current error:");Serial.println(currentError);
  
    
  sumError+=currentError;
  differenceError = currentError - previousError;

  // Serial.print("Difference error");Serial.println(differenceError);
  // Calculate PID control output (error) for velocity difference between wheels
  error = kp_line * currentError + kd_line  * differenceError;
  if((lineL1 ==0) & (lineL2==0) & (lineR1 ==0) & (lineR2==0) & (line0 ==0)){
    error   += ki_line * sumError; 
  }
  // Adjust wheel velocities
  vr = v - error;
  vl = v + error;
}