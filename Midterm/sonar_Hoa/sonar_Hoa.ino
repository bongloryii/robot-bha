// 1 1 1 1 1        0 Robot found continuous line : STOPPED
// 0 0 0 0 0        0 Robot found no line: turn 180o

#include "TimerOne.h"

unsigned long previousMillis = 0;
const float T = 0.1; // Sampling rate

// Control motion variables
float v = 0.12, vr, vl;
float currentError, differenceError;

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
volatile int lineL1;
volatile int lineL2;
volatile int line0;
volatile int lineR1;
volatile int lineR2;


// volatile float maxFront = 15;
float duration1, frontDistance, duration2, leftDistance, duration3, rightDistance;
const float maxDistance = 30;
int leftFollow = 0;
int rightFollow =0;
void loop() 
{
  if (readLineFollower ==1) {
    readLineFollower=0;
    checkObstacle();
  // Serial.print("readLineFollower: "); Serial.println(readLineFollower);
  //   Serial.print("isFrontObstacle: "); Serial.println(isFrontObstacle);
  //   Serial.print("isLeftObstacle: "); Serial.println(isLeftObstacle);
  //   Serial.print("isRightObstacle: "); Serial.println(isRightObstacle);
  //   Serial.print("followLine: "); Serial.println(followLine);
  //   Serial.print("avoidObstacle: "); Serial.println(avoidObstacle);
  //   Serial.print("leftFollow: "); Serial.println(leftFollow);
  //   Serial.print("rightFollow: "); Serial.println(rightFollow);
  //   Serial.print("lineL1: "); Serial.println(lineL1);
  //   Serial.print("lineL2: "); Serial.println(lineL2);
  //   Serial.print("lineR1: "); Serial.println(lineR1);
  //   Serial.print("lineR2: "); Serial.println(lineR2);
  //   Serial.print("line0: "); Serial.println(line0);
  //   Serial.print("rightDistance: "); Serial.println(rightDistance);
  //   Serial.println("-----------------------------");
  }
  // if ((isFrontObstacle+isLeftObstacle+isRightObstacle)>0) {
  if ((isFrontObstacle)>0) {

    followLine = 0;
    avoidObstacle= 1;
  }
  if (avoidObstacle) {
    followBoundary();
  }
  
  if (((leftFollow+rightFollow)>0)&& (lineL1+lineL2+lineR1+lineR2+line0)>2){
    stop();
    delay(300);
    followLine =1;
    if (leftFollow) {
      // rotateLeft();
      setSpeed(0.2,0.05);
    } else {
      // rotateRight();
      setSpeed(0.05,0.2);
    }
    avoidObstacle = 0; 
    leftFollow = 0;
    rightFollow=0;
    delay(200);
  }
  if (followLine==1) {
    FollowLine();
  }
}