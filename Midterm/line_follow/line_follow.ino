// 1 1 1 1 1        0 Robot found continuous line : STOPPED
// 0 0 0 0 0        0 Robot found no line: turn 180o

#include "TimerOne.h"

unsigned long previousMillis = 0;
const float T = 0.1; // Sampling rate

// Goal and current pose
float x = 0, y = 0, theta = 0;
float x_g = 2, y_g = 3, theta_g = 0;

void setup() 
{
  Serial.begin(9600);
  //Set up line follower sensor
  setupLineFollower();
  setupMotors();
  // setupIMU();
}

void loop() 
{
  FollowLine();
  // readIMU();
}
