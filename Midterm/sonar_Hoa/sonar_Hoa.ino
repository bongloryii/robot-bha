// 1 1 1 1 1        0 Robot found continuous line : STOPPED
// 0 0 0 0 0        0 Robot found no line: turn 180o

#include "TimerOne.h"

unsigned long previousMillis = 0;
const float T = 0.1; // Sampling rate

// Goal and current pose
float x = 0, y = 0, theta = 0;
float x_g = 2, y_g = 3, theta_g = 0;

int followLine = 1;
int avoidObstacle= 0;
int isLeftObstacle =0;
int isRightObstacle=0;
int isFrontObstacle=0;
float obstacleAdjustment;

void setup() 
{
  Serial.begin(9600);
  //Set up line follower sensor
  setupLineFollower();
  setupMotors();
  setupSonar();
  // setupIMU();
}


int readLineFollower;

// line follower variables
int lineL1;
int lineL2;
int line0;
int lineR1;
int lineR2;
void loop() 
{
  if (readLineFollower ==1) {
    readLineFollower=0;
    getLineState();
    checkObstacle();    
  }
  if (isFrontObstacle) {
    followLine = 0;
    avoidObstacle= 1;
  }
  if (avoidObstacle) {
    followBoundary();
  }
  if ((followLine ==0) && (lineL1+lineL2+lineR1+lineR2+line0)>2){
    avoidObstacle = 0; 
    followLine =1;
    followBoundary();
  }
  if ((isFrontObstacle+isLeftObstacle+isRightObstacle)==0){
    followLine = 1;
    avoidObstacle= 0;
  }
  
  if (followLine==1) {
    FollowLine();
  }
  
  
}
// void loop() 
// {
//   FollowLine();
//   // readIMU();
// }
