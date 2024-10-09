// 1 1 1 1 1        0 Robot found continuous line : STOPPED
// 0 0 0 0 0        0 Robot found no line: turn 180o

#include "TimerOne.h"
// PID control parameters
const float kp_line = 0.13;
const float ki_line = 0.009;
const float kd_line = 0.007;
float error_line, sumError_line= 0, previousError_line= 0, differenceError_line = 0, currentError_line;

// line following sensor pin
#define OUT1 41
#define OUT2 43
#define OUT3 45
#define OUT4 47
#define OUT5 49

const float outer_eye_weight = 0.8;  //line follower -- weight of outer eyes
const float inner_eye_weight = 0.2;   //line follower -- weight of inner eyes
const float middle_eye_weight = 0.1;  //line follower -- weight of the middle eye

void setupLineFollower() {
  
  //Set up line follower sensor
  pinMode(OUT1, INPUT);
  pinMode(OUT2, INPUT);
  pinMode(OUT3, INPUT);
  pinMode(OUT4, INPUT);
  pinMode(OUT5, INPUT);
}

// void readLineFollowerSensor() {
//   readLineFollower = 1;
//   getLineState();

// }
unsigned long stopTestDuration = 0;
unsigned long currentMillis = 0;
// unsigned long previousMillis;
void FollowLine()  //loop
{
  if (readLineFollower = 1) {
    readLineFollower = 0;
    getLineState();
    Serial.print(lineL1);
    Serial.print(lineL2);
    Serial.print(line0);
    Serial.print(lineR1);
    Serial.println(lineR2);
  }
  Serial.print(vr);
  Serial.print(";");
  Serial.println(vl);
  if ((lineL1 == 1) & (lineL2 == 1) & (lineR1 == 1) & (lineR2 == 1) & (line0 == 1)) {
    reachedGoal =1;
    stop();
    delay(3000);
  }
  // Update error_linefor PID
  calculatePIDError_line();
  // Control motor speeds
  setSpeed(vr, vl);
}

void getLineState() {
  lineL1 = digitalRead(OUT1);
  lineL2 = digitalRead(OUT2);
  line0 = digitalRead(OUT3);
  lineR1 = digitalRead(OUT4);
  lineR2 = digitalRead(OUT5);
}
void calculatePIDError_line() {
  previousError_line= currentError_line;
  currentError_line= -outer_eye_weight * lineL2 - inner_eye_weight * lineL1 + inner_eye_weight * lineR1 + outer_eye_weight * lineR2 + middle_eye_weight * (1 - line0);

  // currentError_line= outer_eye_weight * (1-lineL2) + inner_eye_weight * (1-lineL1) - inner_eye_weight * (1-lineR1) - outer_eye_weight * (1-lineR2) + (middle_eye_weight) * (1-line0);
  Serial.print("Current error:");
  Serial.println(currentError_line);


  sumError_line+= currentError_line;
  differenceError_line= currentError_line- previousError_line;

  // Serial.print("Difference error");Serial.println(differenceError);
  // Calculate PID control output (error) for velocity difference between wheels
  error_line= kp_line * currentError_line+ kd_line * differenceError_line;
  if ((lineL1 == 0) & (lineL2 == 0) & (lineR1 == 0) & (lineR2 == 0) & (line0 == 0)) {
    error_line+= ki_line * sumError_line;
  }
  Serial.print("error line: "); Serial.println(error_line);
  // Adjust wheel velocities
  vr = v - error_line;
  vl = v + error_line;
}