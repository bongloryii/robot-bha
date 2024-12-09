#include <TimerOne.h>
#include <HCSR04.h>
#include <Wire.h>
#include <Servo.h>

const byte triggerPin = 48;
const byte echoPin = 49;
UltraSonicDistanceSensor distanceSensor(triggerPin, echoPin);

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

// Encoder chân A
int enLA = 2; // Encoder trái
int enRA = 3; // Encoder phải

// Bộ đếm encoder
volatile int counterLA = 0; 
volatile int counterRA = 0; 

float vR, vL;

// PID constants
float Kp = 300;
float Ki = 800;
float Kd = 100;
float set_vL = 0, set_vR = 0;
float err_vL = 0, err_vR = 0, pre_err_vL = 0, pre_err_vR = 0;
float integralL = 0, integralR = 0, derivativeR = 0, derivativeL = 0;
float controlOutputL = 0, controlOutputR = 0;


void setup() {
  Serial.begin(9600);

   // Gán ngắt encoder
  attachInterrupt(digitalPinToInterrupt(enLA), countEnLA, RISING);
  attachInterrupt(digitalPinToInterrupt(enRA), countEnRA, RISING);

  servo.attach(6);
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

  // Begin Serial communication at a baud rate of 9600:
  Serial.begin(9600);
}

void loop() {
  // Nhận dữ liệu vận tốc từ serial (vd: "0.1 0.15")
  // bottleDetect();
  Serial.println(!bottleDetect());
  if (Serial.available() > 0) {
    if(!bottleDetect()){
      String data = Serial.readStringUntil('\n');
      data.trim();
      int separatorIndex = data.indexOf(' ');

      if (separatorIndex != -1) {
        set_vR = data.substring(0, separatorIndex).toFloat();
        set_vL = data.substring(separatorIndex + 1).toFloat();
      }
    }
  }
}


bool bottleDetect() {
  float distance = distanceSensor.measureDistanceCm();
    Serial.print(distance);

  if (distance > 1 && distance < 7) {
    Serial.print("true");
    set_vR = 0;
    set_vL = 0;
    delay(1000);
    servo.write(10);

    return true;
  } else {
    servo.write(0);    
    Serial.print("false");

    return false;
  }
}


void VelCtrlTimer() {
  // Tính vận tốc bánh xe
  vR = (float(counterRA) * M_PER_REV+0) / (samplingTime * ENCODER_RESOLUTION);
  vL = (float(counterLA) * M_PER_REV+0) / (samplingTime * ENCODER_RESOLUTION);

  // Reset bộ đếm encoder
  counterRA = 0;
  counterLA = 0;

  // PID cho bánh trái
  err_vL = set_vL - vL;
  integralL += err_vL * samplingTime;
  derivativeL = (err_vL - pre_err_vL) / samplingTime;
  controlOutputL = Kp * err_vL + Ki * integralL + Kd * derivativeL;
  pre_err_vL = err_vL;

  // PID cho bánh phải
  err_vR = set_vR - vR;
  integralR += err_vR * samplingTime;
  derivativeR = (err_vR - pre_err_vR) / samplingTime;
  controlOutputR = Kp * err_vR + Ki * integralR + Kd * derivativeR;
  pre_err_vR = err_vR;

  
  // Set the speed = 0 if the set value = 0
  if (set_vL == 0) {
    controlOutputL = 0;
  }

  if (set_vR == 0) {
    controlOutputR = 0;
  }

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
    digitalWrite(inL2, HIGH);
    digitalWrite(inL1, LOW);
  }
  
  if (speed < 0) {
    digitalWrite(inL2, LOW);
    digitalWrite(inL1, HIGH);
  }
  
  speed = abs(speed);
  if (speed > 255) {
    speed = 255;
  }
  if (speed < 20 && speed > 0) {
    speed = 20;
  }
  
  analogWrite(enL, speed);
}

void setMotorSpeedR(int speed) {
  if (speed == 0) {
    digitalWrite(inR1, LOW);
    digitalWrite(inR2, LOW);   
  }

  if (speed > 0) {
    digitalWrite(inR2, HIGH);
    digitalWrite(inR1, LOW);
  }
  
  if (speed < 0) {
    digitalWrite(inR2, LOW);
    digitalWrite(inR1, HIGH);
  }
  
  speed = abs(speed);
  if (speed > 255) {
    speed = 255;
  }
  if (speed < 20 && speed > 0) {
    speed = 20;
  }

  analogWrite(enR, speed);
}

void stop() { 
	digitalWrite(inR1, LOW);
	digitalWrite(inR2, LOW);
	digitalWrite(inL1, LOW);
	digitalWrite(inL2, LOW);
}

void countEnLA() {
  counterLA++;
}

void countEnRA() {
  counterRA++;
}

