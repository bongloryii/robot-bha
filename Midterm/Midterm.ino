#include <TimerThree.h>

// line following sensor pin
#define OUT1 41
#define OUT2 43
#define OUT3 45
#define OUT4 47
#define OUT5 49
// Define motor pins and encoders
int motorRpin1 = 10; // Right motor IN3
int motorRpin2 = 11; // Right motor IN4//fw 

int motorLpin1 = 8; // Left motor IN1
int motorLpin2 = 9; // Left motor IN2//fw

int enR = 13;  // Right motor enable pin (PWM)
int enL = 7; // Left motor enable pin (PWM)

// Encoders
int enLA = 2;  // Left motor encoder A
int enRA = 3; // Right motor encoder A

volatile unsigned int counterLA = 0;
volatile unsigned int counterRA = 0;

const float MAX_SPEED = 0.2;
const float WHEEL_DISTANCE = 0.182; // Distance between two wheels
const int PULSE_PER_REV = 940;
const float WHEEL_PERIMETER = 0.1413; // Circumference of the wheel
// 1 1 1 1 1        0 Robot found continuous line : STOPPED
// 0 0 0 0 0        0 Robot found no line: turn 180o

#include "TimerOne.h"

float rpmLA = 0; 
float rpmRA = 0; 

unsigned long previousMillis = 0;
const float T = 0.2; // Sampling rate

// Control motion variables
float v = 0.13, vr, vl;
float currentError, differenceError;

// Goal and current pose
// float x = 0, y = 0, theta = 0;
// float x_g = 2, y_g = 3, theta_g = 0;
int isPrint=0;
int isRead=0;
int followLine = 1;
int avoidObstacle= 0;
int isLeftObstacle =0;
int isRightObstacle=0;
int isFrontObstacle=0;
int reachedDestination=0;
float obstacleAdjustment;
int reachedGoal = 0;
void setup() 
{
  Serial.begin(9600);
  Timer1.initialize(T*100000);    //100ms     
  Timer1.attachInterrupt(isReadTime);  

  //Set up line follower sensor
  setupLineFollower();
  setupMotors();
  setupSonar();
  // setupIMU();
  // Timer3.initialize(500000);  //500ms
  // Timer3.attachInterrupt(printDebug);

}


void isReadTime(){ 
  isRead = 1;
  getLineState();
  // isPrint=1;
}
int readLineFollower;

// line follower variables
volatile int lineL1;
volatile int lineL2;
volatile int line0;
volatile int lineR1;
volatile int lineR2;

unsigned long startRecover; //time that startrecover mode is on
unsigned long durationRecover;
// volatile float maxFront = 15;
float duration1, frontDistance, duration2, leftDistance, duration3, rightDistance;
const float maxDistance = 30;
int leftFollow = 0;
int rightFollow =0;
// int isRead=0;
int recoverFromObstacle = 0;
void loop() 
{
  if (isRead ==1) {
    // readLineFollower=0;
    isRead=0;
    // if (reachedGoal) {
    //   readIMU_noTimer();
    // }    
    checkObstacle();
    // localize();
    //get current speed
    // rpmLA = calculateRPM(counterLA);    
    // Serial.print("RPM_L = "); Serial.print(rpmLA);
    // rpmRA = calculateRPM(counterRA);    
    // Serial.print("; RPM_R = "); Serial.println(rpmRA);
    // resetCounters();
    // localize();
    // getLineState();
  }
  if (isPrint == 1){
    isPrint=0;
  // Serial.print("readLineFollower: "); Serial.println(readLineFollower);
    Serial.print("isFrontObstacle: "); Serial.println(isFrontObstacle);
    Serial.print("isLeftObstacle: "); Serial.println(isLeftObstacle);
    Serial.print("isRightObstacle: "); Serial.println(isRightObstacle);
    Serial.print("followLine: "); Serial.println(followLine);
    Serial.print("avoidObstacle: "); Serial.println(avoidObstacle);
    // Serial.print("reachedGoal: "); Serial.println(reachedGoal);
    Serial.print("leftFollow: "); Serial.println(leftFollow);
    Serial.print("rightFollow: "); Serial.println(rightFollow);
    // Serial.print("lineL1: "); Serial.println(lineL1);
    // Serial.print("lineL2: "); Serial.println(lineL2);
    // Serial.print("lineR1: "); Serial.println(lineR1);
    // Serial.print("lineR2: "); Serial.println(lineR2);
    // Serial.print("line0: "); Serial.println(line0);
        Serial.print("frontDistance: "); Serial.println(frontDistance);
Serial.print("rightDistance: "); Serial.println(rightDistance);
    Serial.print("leftDistance: "); Serial.println(leftDistance);

    Serial.println("-----------------------------");
  
  } 
  // if ((isFrontObstacle+isLeftObstacle+isRightObstacle)>0) {
  if ((reachedGoal ==0)&&((isFrontObstacle)>0)) { // nếu có vật cản phía trước thì thoát mode dò line chuyển sang né vật cản
    followLine = 0;
    avoidObstacle= 1;
  }
  if ((reachedGoal ==0)&&(avoidObstacle)) {
    followBoundary();
  }
  
  if ((reachedGoal ==0)&&(((leftFollow+rightFollow)>0) && (lineL1+lineL2+lineR1+lineR2+line0)>0)){ //nếu đang follow boundary mà thấy ít nhất 2/5 đèn dò line sáng
    Serial.println("OBSTACLE FREE, BACK TO LINE");
    stop(); //thì ngừng lại khoảng 300ms
    delay(300);
    if (leftFollow) { //nếu mà đang ôm cua bên trái (gặp vật cản, quẹo trái, đi vòng qua vật cản và trở về line)
      // rotateLeft(); 
      // delay(800);
      setSpeed(0.19,-0.02);
      delay(300);
      while ((lineL1+lineL2+lineR1+lineR2+line0)<2){
      setSpeed(0.18,-0.02);
      }
      stop();
      setSpeed(0.1,0.1); //lúc này quay sang trái để hướng mặt về phía trước của line
    } else {
      // rotateRight();
      setSpeed(-0.02,0.19);
      delay(300);
      while ((lineL1+lineL2+lineR1+lineR2+line0)<2){
      setSpeed(-0.02,0.18);
      }
      stop();
      setSpeed(0.1,0.1);//tương tự cho trường hợp ôm cua bên phải
    }
    //thông báo cho con robot là sẽ về mode dò line
    followLine =1; 
    // recoverFromObstacle = 1;
    // startRecover = millis();
    avoidObstacle = 0; 
    leftFollow = 0;
    rightFollow=0;
    // v=0.1;
    // kp_line = kp_line+0.02;
    // delay(200); //để robot quẹo trong khoảng 200ms
  }
  // if (recoverFromObstacle == 1) { 
  //   Serial.println("recovering from obstacle");
  //   durationRecover = millis() - startRecover;
  //   if (durationRecover > 1000) {
  //     v = 0.15;
  //     recoverFromObstacle = 0;
  //     kp_line -=0.02;
  //     Serial.print("done recover, (kp, v)  ="); Serial.println(kp);Serial.println(v);

  //   }
  // }
  if ((reachedGoal ==0)&&(followLine==1)) {
    FollowLine(); //trạng thái follow line
  }
  if (reachedGoal ==1) {
    // v=0.15;
    followLine=0;
    avoidObstacle = 0;
    positionControl();
  }
  if (reachedDestination ==1) {
    stop();
    delay(5000);
    while(1);
  }
}
void printDebug(){
  isPrint=1;
}