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
// Ultrasonic
int trigPin1 = 50; // Front ultrasonic
int echoPin1 = 51;
int trigPin2 = 36; // Left ultrasonic
int echoPin2 = 37;
int trigPin3 = 52; // Right ultrasonic
int echoPin3 = 53;


// // volatile float maxFront = 15;
// float duration1, frontDistance, duration2, leftDistance, duration3, rightDistance;
// const float maxDistance = 15;

void setupSonar() 
{
  // Set up 3 ultrasonic sensors
  pinMode(trigPin1, OUTPUT);
  pinMode(echoPin1, INPUT);
  pinMode(trigPin2, OUTPUT);
  pinMode(echoPin2, INPUT);
  pinMode(trigPin3, OUTPUT);
  pinMode(echoPin3, INPUT);
}
void goForward(float velocity = 0.2){
  setSpeed(velocity,velocity);
}
void followBoundary(){

    if (leftFollow) {
      setSpeed(0.06,0.2);
      if ((rightDistance>0)&&(rightDistance < 10)){
      setSpeed(0.2,0.05);
      } 
      // setSpeed(v-rightDistance*0.003,v);
    }else if (rightFollow) {
      // setSpeed(v,v-rightDistance*0.003);
        setSpeed(0.2,0.06);
    if ((leftDistance>0)&&(leftDistance < 10)){
      setSpeed(0.05,0.2);
      } 

    }
    else {

    if (!isLeftObstacle) {
      rotateLeft();
      setSpeed(0.1,0.15);
      leftFollow =1;
    } else if (!isRightObstacle) {
      rotateRight();
      setSpeed(0.15,0.1);
      rightFollow =1;
    }
    delay(300);
  // goForward(0.1);  
    }
}
void checkFrontDistance() {

  digitalWrite(trigPin1, LOW);
  delayMicroseconds(5);
  digitalWrite(trigPin1, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin1, LOW);
  duration1 = pulseIn(echoPin1, HIGH);
  frontDistance = duration1 * 0.037 / 2;
  delay(60); 
  Serial.print("Front distance: ");Serial.print(frontDistance);
}

void checkLeftDistance() {
  
  digitalWrite(trigPin2, LOW);
  delayMicroseconds(5);
  digitalWrite(trigPin2, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin2, LOW);
  duration2 = pulseIn(echoPin2, HIGH);
  leftDistance = duration2 * 0.037 / 2;
    Serial.print("Left distance: ");Serial.print(leftDistance);
  delay(60); 

}

void checkRightDistance() {
  
  digitalWrite(trigPin3, LOW);
  delayMicroseconds(5);
  digitalWrite(trigPin3, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin3, LOW);
  duration3 = pulseIn(echoPin3, HIGH);
  rightDistance = duration3 * 0.037 / 2;
    Serial.print("Right distance: ");Serial.print(rightDistance);
  delay(60); 

}
void checkObstacle() {
  checkFrontDistance();
  checkLeftDistance();
  checkRightDistance();
  if ((frontDistance>0)&&(frontDistance < maxDistance)) {
    isFrontObstacle = 1;
  } else {
        isFrontObstacle = 0;

  }
  if ((leftDistance>0)&&(leftDistance < maxDistance)) {
    isLeftObstacle = 1;
  } else {
        isLeftObstacle = 0;

  }
  if ((leftDistance>0)&&(rightDistance < maxDistance)) {
    isRightObstacle = 1;
  } else {
        isRightObstacle = 0;

  }
}
void rotateLeft() {
rightForward(0.2);
leftBackward(0.2);
delay(600);
}

void rotateRight() {
rightBackward(0.2);
leftForward(0.2);
delay(600);
}

// void ObstacleAvoid() {

//   // front distance check
//   checkFrontDistance();
//   if (frontDistance < maxDistance) {
//     checkLeftDistance();
//     delay(60);
//     checkRightDistance();
//     delay(60);
//     if (leftDistance < rightDistance)
//       turnRight();
//     else if (leftDistance > rightDistance) {
//       turnLeft();
//     }
//   }
//   else {
//     rightForward(vr);
//     leftForward(vl);
//   }

//   // left distance check
//   checkLeftDistance();
//   if (leftDistance < maxDistance)
//     checkRightDistance();
//     delay(60);
//     if (leftDistance > rightDistance)
//       rightForward(vr); 
//       leftForward(vl);
//     else if (leftDistance < rightDistance) {
//       turnRight();
//     }
  

//   // right distance check
//   checkRightDistance();
//   if (rightDistance < maxDistance) 
//     checkLeftDistance();
//     delay(60);
//     if (rightDistance > leftDistance)
//       rightForward(vr);
//       leftForward(vl);
//     else if (rightDistance < leftDistance) {
//       turnLeft();
//     }
  
// }
