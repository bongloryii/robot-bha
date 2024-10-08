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


// volatile float maxFront = 15;
float duration1, frontDistance, duration2, leftDistance, duration3, rightDistance;
const float maxDistance = 15;

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
void goCircle(float radius, float velocity =0.2) { //plus is clockwise
  //around a circle of 1m radius, vl=1.2vr
  if (radius > 0 ){
    float expected_speed_ratio = (radius + WHEEL_DISTANCE/2)/(radius - WHEEL_DISTANCE/2);
    setSpeed(velocity/expected_speed_ratio,velocity);}
    else {
    float expected_speed_ratio = (-radius + WHEEL_DISTANCE/2)/(-radius - WHEEL_DISTANCE/2);
    setSpeed(velocity, velocity/expected_speed_ratio);
    }
}
void goForward(float velocity = 0.2){
  setSpeed(velocity,velocity);
}
void followBoundary(){

  if (!isLeftObstacle) {
    rotateLeft();
    setSpeed(0.1,0.15);
    
  } else if (!isRightObstacle) {
    rotateRight();
    setSpeed(0.15,0.1);
  }
  goForward(0.1);  
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

}
void checkObstacle() {
  checkFrontDistance();
  checkLeftDistance();
  checkRightDistance();
  if (frontDistance < maxDistance) {
    isFrontObstacle = 1;
  }
  if (leftDistance < maxDistance) {
    isLeftObstacle = 1;
  }
  if (rightDistance < maxDistance) {
    isRightObstacle = 1;
  }
}
void rotateLeft() {
rightForward(0.2);
leftBackward(0.2);
delay(500);
}

void rotateRight() {
rightBackward(0.2);
leftForward(0.2);
delay(500);
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
