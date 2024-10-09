#include <TimerThree.h>

// 1 1 1 1 1        0 Robot found continuous line : STOPPED
// 0 0 0 0 0        0 Robot found no line: turn 180o

#include "TimerOne.h"

unsigned long previousMillis = 0;
const float T = 0.1; // Sampling rate

// Control motion variables
float v = 0.15, vr, vl;
float currentError, differenceError;

// Goal and current pose
// float x = 0, y = 0, theta = 0;
// float x_g = 2, y_g = 3, theta_g = 0;
int isPrint=0;
int followLine = 1;
int avoidObstacle= 0;
int isLeftObstacle =0;
int isRightObstacle=0;
int isFrontObstacle=0;
float obstacleAdjustment;
int reachedGoal = 0;
void setup() 
{
  Serial.begin(9600);
  //Set up line follower sensor
  setupLineFollower();
  setupMotors();
  setupSonar();
  setupIMU();
  Timer3.initialize(500000);  //500ms
  Timer3.attachInterrupt(printDebug);

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
int isRead=0;
void loop() 
{
  if (isRead ==1) {
    // readLineFollower=0;
    isRead=0;
    readIMU_noTimer();
    checkObstacle();
  }
  if (isPrint == 1){
    isPrint=0;
  // Serial.print("readLineFollower: "); Serial.println(readLineFollower);
    Serial.print("isFrontObstacle: "); Serial.println(isFrontObstacle);
    Serial.print("isLeftObstacle: "); Serial.println(isLeftObstacle);
    Serial.print("isRightObstacle: "); Serial.println(isRightObstacle);
    Serial.print("followLine: "); Serial.println(followLine);
    Serial.print("avoidObstacle: "); Serial.println(avoidObstacle);
    Serial.print("reachedGoal: "); Serial.println(reachedGoal);

    Serial.print("leftFollow: "); Serial.println(leftFollow);
    Serial.print("rightFollow: "); Serial.println(rightFollow);
    Serial.print("lineL1: "); Serial.println(lineL1);
    Serial.print("lineL2: "); Serial.println(lineL2);
    Serial.print("lineR1: "); Serial.println(lineR1);
    Serial.print("lineR2: "); Serial.println(lineR2);
    Serial.print("line0: "); Serial.println(line0);
    Serial.print("rightDistance: "); Serial.println(rightDistance);
    Serial.print("leftDistance: "); Serial.println(leftDistance);
    Serial.print("frontDistance: "); Serial.println(leftDistance);

    Serial.println("-----------------------------");
  
  } 
  // if ((isFrontObstacle+isLeftObstacle+isRightObstacle)>0) {
  if ((isFrontObstacle)>0) { // nếu có vật cản phía trước thì thoát mode dò line chuyển sang né vật cản
    followLine = 0;
    avoidObstacle= 1;
  }
  if (avoidObstacle) {
    followBoundary();
  }
  
  if (((leftFollow+rightFollow)>0) && (lineL1+lineL2+lineR1+lineR2+line0)>0){ //nếu đang follow boundary mà thấy ít nhất 2/5 đèn dò line sáng
    Serial.println("OBSTACLE FREE, BACK TO LINE");
    stop(); //thì ngừng lại khoảng 300ms
    delay(300);
    if (leftFollow) { //nếu mà đang ôm cua bên trái (gặp vật cản, quẹo trái, đi vòng qua vật cản và trở về line)
      // rotateLeft();
      setSpeed(v,-v);
      delay(600);
      setSpeed(0.2,0.05); //lúc này quay sang trái để hướng mặt về phía trước của line
    } else {
      // rotateRight();
      setSpeed(-v,v);
      delay(600);
      setSpeed(0.05,0.2); //tương tự cho trường hợp ôm cua bên phải
    }
    //thông báo cho con robot là sẽ về mode dò line
    followLine =1; 
    avoidObstacle = 0; 
    leftFollow = 0;
    rightFollow=0;
    // delay(200); //để robot quẹo trong khoảng 200ms
  }
  if (followLine==1) {
    FollowLine(); //trạng thái follow line
  }
  if (reachedGoal ==1) {
    positionControl();
  }
}
void printDebug(){
  isPrint=1;
}