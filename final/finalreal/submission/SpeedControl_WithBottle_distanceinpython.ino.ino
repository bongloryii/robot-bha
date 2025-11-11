#include <TimerOne.h>
#include <HCSR04.h>
#include <Wire.h>
#include <Servo.h>

const byte triggerPin = 48;
const byte echoPin = 49;
UltraSonicDistanceSensor distanceSensor(triggerPin, echoPin);
bool isFound=false;
Servo servo;

#define PI 3.1415926535897932384626433832795
float d = 0.182; // Khoảng cách giữa 2 bánh xe
float r = 0.0225; // Bán kính bánh xe
float samplingTime = 0.1; // Thời gian lấy mẫu
const int ENCODER_RESOLUTION = 1050; 
float M_PER_REV = 2 * PI * r; // Quãng đường đi được mỗi vòng quay

// Motor trái
int enL = 7;
int inL1 = 8;
int inL2 = 9;

// Motor phải
int enR = 13;
int inR1 = 10;
int inR2 = 11;
bool isDebug = false;
// Encoder chân A
int enLA = 2; // Encoder trái
int enRA = 3; // Encoder phải

// Bộ đếm encoder
volatile int counterLA = 0; 
volatile int counterRA = 0; 

float vR, vL;
int vR_sign, vL_sign;
// PID constants
// PID constants
float KpL = 200;
float KiL = 500;
float KdL = 100;

float KpR = 200;  // Proportional gain
float KiR = 500;  // Integral gain
float KdR =100;  // Derivative gain

float set_vL = 0, set_vR = 0;
float err_vL = 0, err_vR = 0, pre_err_vL = 0, pre_err_vR = 0;
float integralL = 0, integralR = 0, derivativeR = 0, derivativeL = 0;
float controlOutputL = 0, controlOutputR = 0;
int currentState = 1; // 1: Searching for bottle, 2: Bottle found, 3: Path following


void setup() {
  Serial.begin(9600);

   // Gán ngắt encoder
  attachInterrupt(digitalPinToInterrupt(enLA), countEnLA, RISING);
  attachInterrupt(digitalPinToInterrupt(enRA), countEnRA, RISING);

  servo.attach(5);
  servo.write(0);

  // Khởi tạo chân encoder
  pinMode(enL, OUTPUT);
  pinMode(enR, OUTPUT);

  // Khởi tạo chân điều khiển motor
  pinMode(inR1, OUTPUT);
  pinMode(inR2, OUTPUT);
  pinMode(inL1, OUTPUT);
  pinMode(inL2, OUTPUT);
 
  // Tắt motor ban đầu
  digitalWrite(inR1, LOW);
  digitalWrite(inR2, LOW);
  digitalWrite(inL1, LOW);
  digitalWrite(inL2, LOW);

  // Khởi tạo Timer1 để gọi hàm VelCtrlTimer mỗi samplingTime
  Timer1.initialize(1000000 * samplingTime); 
  Timer1.attachInterrupt(VelCtrlTimer); 
  stop();
  servo.write(0);
  isFound = false;
  set_vL=0;
  set_vR=0;
}


void loop() {
  if (Serial.available() > 0) {
    if (isFound){
      stop();
      delay(1000);
      set_vL=0;
      set_vR=0;
    }else{
      processCommand();
    }
  }

}



// void processCommand() {

//   String data = Serial.readStringUntil('\n');
//   data.trim();
//   int separatorIndex = data.indexOf(' ');

//   if (separatorIndex != -1) {
//     set_vR = data.substring(0, separatorIndex).toFloat();
//     set_vL = data.substring(separatorIndex + 1).toFloat();
//   }
// }
void processCommand() {
  String command = Serial.readStringUntil('\n');
// Serial.print(command);
  command.trim(); // Remove whitespace
  char state = command.charAt(0);

  if (state == '1' || state == '3') {
    // Parse velocities: format "STATE,vr,vl"
    int firstComma = command.indexOf(',');
    int secondComma = command.lastIndexOf(',');

    set_vR = command.substring(firstComma + 1, secondComma).toFloat();
    set_vL = command.substring(secondComma + 1).toFloat();

  } else if (state == '2') {
    // Stop command
    stop();
  }
}

void VelCtrlTimer() {
  float distance = distanceSensor.measureDistanceCm();

  // if ((distance=-1)||(distance>3)){
  if (currentState == 1) { // Searching for bottle
    // Serial.println(distance);
    if (distance > 0 && distance < 5) { // Example threshold
      Serial.println("X");
      // stop();
      // delay(1000);
      isFound=true;
      servo.write(25);
      // delay(1000);
      currentState = 2;

      stop(); 
      // while(1)
      // {
        controlOutputL = 0, controlOutputR = 0;
      setMotorSpeedR((int)controlOutputR);
      setMotorSpeedL((int)controlOutputL);
      // }
      
      delay(100000);
    }
  } else if (currentState == 2) { // Bottle found
    stop();
    delay(1000); // Simulate path planning
    currentState = 3;
  } else if (currentState == 3) { // Path following
    // Continue moving based on set velocities from Python
  }
  // Tính vận tốc bánh xe
  vR = (float (counterRA) * M_PER_REV+0) / (samplingTime * ENCODER_RESOLUTION);
  vL = (float (counterLA) * M_PER_REV+0) / (samplingTime * ENCODER_RESOLUTION);

  // Serial.print("vL: ");
  // Serial.print(counterLA);
  // Serial.print("vR: ");
  // Serial.println(counterRA);
  // Reset bộ đếm encoder
  counterRA = 0;
  counterLA = 0;

  // PID cho bánh trái
  // err_vL = set_vL - vL;
  // integralL += err_vL * samplingTime;
  // derivativeL = (err_vL - pre_err_vL) / samplingTime;
  // controlOutputL = KpL * err_vL + KiL * integralL + KdL * derivativeL;
  // pre_err_vL = err_vL;
  
  // // PID cho bánh phải
  // err_vR = set_vR - vR;
  // integralR += err_vR * samplingTime;
  // derivativeR = (err_vR - pre_err_vR) / samplingTime;
  // controlOutputR = KpR * err_vR + KiR * integralR + KdR * derivativeR;
  // pre_err_vR = err_vR;
  controlOutputR=set_vR;
  controlOutputL=set_vL;
  // Set the speed = 0 if the set value = 0
  if (set_vL == 0) {
    controlOutputL = 0;
  }

  if (set_vR == 0) {
    controlOutputR = 0;
  }
  if (isDebug)
  {
  //   Serial.print("Set vL: ");
  // Serial.print(set_vL);
  // Serial.print("Set vR: ");
  // Serial.println(set_vR);

  // Print actual velocities

  // Print control outputs
  Serial.print("Control Output L: ");
  Serial.print(controlOutputL);
  Serial.print("Control Output R: ");
  Serial.println(controlOutputR);
  }

  // delay(100);
  // Cập nhật tốc độ motor
  setMotorSpeedR((int)controlOutputR);
  setMotorSpeedL((int)controlOutputL);
}

void setMotorSpeedL(int speed) {
  if (speed == 0) {
    digitalWrite(inL1, LOW);
    digitalWrite(inL2, LOW);   
  }

  if (speed > 0) {
    digitalWrite(inL1, LOW);
    vL_sign=1;
    digitalWrite(inL2, HIGH);
  }
  
  if (speed < 0) {
    digitalWrite(inL1, HIGH);
    digitalWrite(inL2, LOW);
    vL_sign=-1;
  }
  
  speed = abs(speed);
  if (speed > 255) {
    speed = 255;
  }
  if (speed < 55 && speed > 0) {
    speed = 55;
  }
  
  analogWrite(enL, speed);
}

void setMotorSpeedR(int speed) {
  if (speed == 0) {
    digitalWrite(inR1, LOW);
    digitalWrite(inR2, LOW);   
  }

  if (speed > 0) {
    digitalWrite(inR1, LOW);
    vR_sign=1;
    digitalWrite(inR2, HIGH);
  }
  
  if (speed < 0) {
    digitalWrite(inR1, HIGH);
    digitalWrite(inR2, LOW);
    vL_sign=-1;
  }
  
  speed = abs(speed);
  if (speed > 255) {
    speed = 255;
  }
  if (speed < 55 && speed > 0) {
    speed = 55;
  }

  analogWrite(enR, speed);
}

void stop() {
  digitalWrite(inR1, LOW);
  digitalWrite(inR2, LOW); 
  digitalWrite(inL1, LOW);
  digitalWrite(inL2, LOW);
  controlOutputL = 0, controlOutputR = 0;

  // delay(5000);
}


void countEnLA() {
  counterLA=counterLA+vL_sign;
}

void countEnRA() {
  counterRA=counterRA+vR_sign;
}

