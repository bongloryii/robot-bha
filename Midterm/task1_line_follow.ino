// 1 1 1 1 1        0 Robot found continuous line : STOPPED
// 0 0 0 0 0        0 Robot found no line: turn 180o

#include "TimerOne.h"
// PID control parameters
const float kp_line = 0.13;
const float ki_line = 0.009;
const float kd_line = 0.0;
float error_line, sumError_line= 0, previousError_line= 0, differenceError_line = 0, currentError_line;

const float outer_eye_weight = 0.6;  //line follower -- weight of outer eyes
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
    // Serial.print(lineL2);
    // Serial.print(lineL1);
    // Serial.print(line0);
    // Serial.print(lineR1);
    // Serial.println(lineR2);
  }
  Serial.print("following line velocity: vr = ");
  Serial.print(vr);
  Serial.print("; ");
  Serial.println(vl);
  // Update error_linefor PID
  calculatePIDError_line();
  // Control motor speeds
  setSpeed(vr, vl);
  
  if ((lineL1 == 1) & (lineL2 == 1) & (lineR1 == 1) & (lineR2 == 1) & (line0 == 1)) {
        stop();
        followLine=0;
    reachedGoal =1;
    // Serial.println("REACHED GOAL. BACK TO DESTINATION.");
    delay(1000);
      // x=0;y=0;theta=0;
    setupIMU();
  }
}

void getLineState() {
  // lineL2 = digitalRead(OUT1);
  // lineL1 = digitalRead(OUT2);
  // line0 = digitalRead(OUT3);
  // lineR1 = digitalRead(OUT4);
  // lineR2 = digitalRead(OUT5);
  lineL1 = digitalRead(OUT1);
  lineL2 = digitalRead(OUT2);
  line0 = digitalRead(OUT3);
  lineR1 = digitalRead(OUT4);
  lineR2 = digitalRead(OUT5);
}

void calculatePIDError_line(){
  previousError_line = currentError_line;
  currentError_line = - outer_eye_weight * lineL2 - inner_eye_weight*1.1 * lineL1 + inner_eye_weight * lineR1 + outer_eye_weight * lineR2 + middle_eye_weight * (1-line0);
  if((lineL1 ==1) & (lineL2==0) & (lineR1 ==0) & (lineR2==0) & (line0 ==0)){
    currentError_line= - 1.18*outer_eye_weight * lineL1; 
    v=0.09;
  } 
  else {
    v=0.13;
  }
  if(((lineL1 ==0) & (lineL2==0) & (lineR1 ==0) & (lineR2==0) & (line0 ==1))||((lineL1 ==0) & (lineL2==0) & (lineR1 ==1) & (lineR2==0) & (line0 ==1))||((lineL1 ==0) & (lineL2==1) & (lineR1 ==0) & (lineR2==0) & (line0 ==1))){
    sumError_line=0;
    v=0.13;
  }
  
  // currentError = alpha * (1-lineL2) + beta * (1-lineL1) - beta * (1-lineR1) - alpha * (1-lineR2) + (gamma) * (1-line0);
  Serial.print("Current error:");Serial.println(currentError_line);
  
    
  sumError_line+=currentError_line;
  differenceError_line = currentError_line - previousError_line;

  // Serial.print("Difference error");Serial.println(differenceError);
  // Calculate PID control output (error) for velocity difference between wheels
  error_line = kp_line * currentError_line + kd_line  * differenceError_line;
  if((lineL1 ==0) & (lineL2==0) & (lineR1 ==0) & (lineR2==0) & (line0 ==0)){
    error_line   += ki_line * sumError_line; 
  }
  // Adjust wheel velocities
  vr = v - error_line;
  vl = v + error_line;
}
// int sensorState;
// void calculatePIDError_line() {
//   previousError_line= currentError_line;
     
// // Combine sensor readings into a unique value for switch-case
//   sensorState = (lineL2 << 4) | (lineL1 << 3) | (line0 << 2) | (lineR1 << 1) | lineR2;

//   switch (sensorState) {
//     case 0b10000:  // 1 0 0 0 0
//       currentError_line = -1.3;
//       break;
//     case 0b11110:  // 1 1 1 1 0
//       currentError_line = -0.8;
//       break;
//     case 0b11000:  // 1 1 0 0 0
//       currentError_line = -1;
//       break;
//     case 0b11100:  // 1 1 1 0 0
//       currentError_line = -0.85;
//       break;
//     case 0b01000:  // 0 1 0 0 0
//       currentError_line = -0.4;
//       break;
//     case 0b01100:  // 0 1 1 0 0
//       currentError_line = -0.2;
//       // sumError_line=0;
//       break;
//     case 0b00100:  // 0 0 1 0 0
//       currentError_line = 0;
//       // sumError_line=0;
//       break;
//     case 0b01110:  // 0 1 1 1 0
//       currentError_line = 0;
//       // sumError_line=0;
//       break;
//     case 0b11111:  // 1 1 1 1 1
//       currentError_line = 0;
//       break;
//     case 0b00010:  // 0 0 0 1 0
//       currentError_line = 0.4;
//       break;
//     case 0b00110:  // 0 0 1 1 0
//       currentError_line = 0.2;
//       // sumError_line=0;
//       break;
//     case 0b00111:  // 0 0 1 1 1
//       currentError_line = 0.9;
//       break;
//     case 0b00011:  // 0 0 0 1 1
//       currentError_line = 1;
//       break;
//     case 0b01111:  // 0 1 1 1 1
//       currentError_line = 0.8;
//       break;
//     case 0b00001:  // 0 0 0 0 1
//       currentError_line = 1.3;
//       break;
//     default:
//       // currentError_line = 0;  // Default error if no specific match
//       break;
//   }

    
//     // currentError_line= (1-(1-line0)*middle_eye_weight)*(-outer_eye_weight * lineL2 - inner_eye_weight * lineL1 + inner_eye_weight * lineR1 + outer_eye_weight * lineR2);

  

//   // if (lineL1+lineL2+lineR1+lineR2+line0)
//   // currentError_line= outer_eye_weight * (1-lineL2) + inner_eye_weight * (1-lineL1) - inner_eye_weight * (1-lineR1) - outer_eye_weight * (1-lineR2) + (middle_eye_weight) * (1-line0);
//   Serial.print("Current error:");
//   Serial.println(currentError_line);


//   sumError_line+= currentError_line;
//   differenceError_line= currentError_line- previousError_line;

//   // Serial.print("Difference error");Serial.println(differenceError);
//   // Calculate PID control output (error) for velocity difference between wheels
//   // error_line= kp_line * currentError_line+ kd_line * differenceError_line+ki_line * sumError_line;
//   error_line= kp_line * currentError_line+ kd_line * differenceError_line ;

//   if ((lineL1 == 0) & (lineL2 == 0) & (lineR1 == 0) & (lineR2 == 0) & (line0 == 0)) {
//     error_line+= ki_line * sumError_line;
//     // + middle_eye_weight * (1 - line0)
//   }
//   Serial.print("error line: ---------------------------------------------=      "); Serial.println(error_line);
//   // Adjust wheel velocities
//   vr = v - error_line;
//   vl = v + error_line;
// }