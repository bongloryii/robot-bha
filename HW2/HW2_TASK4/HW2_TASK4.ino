//the front of the robot is the L298N, the back is the battery case
int motorRpin1 = 8; //right motor IN1
int motorRpin2 = 9; //right motor IN2

int motorLpin1 = 10; //left motor IN3
int motorLpin2 = 11; //left motor IN4

int enLA = 2; //left motor C1
int enLB = 3; //left motor C2

int enRB = 20; //right motor C2
int enRA = 21; //right motor C1

volatile int counterLA = 0; 
volatile int counterRA = 0; 
volatile int totalCounterLA = 0; 
volatile int totalCounterRA = 0; 
int pwmValue = 255; //preset value for right wheel in circle conytol
int interval = 1000; //delay between rpm calculations

volatile float rpmLA = 0; 
volatile float rpmRA = 0; 

const float WHEEL_DISTANCE = 0.182; //Distance between 2 driving wheels
const int PULSE_PER_REV = 940;
const float WHEEL_PERIMETER = 0.1413;
unsigned long previousMillis = 0;
void setup() {
  Serial.begin(9600);

  pinMode(enLA, INPUT);
  // pinMode(enLB, INPUT);
  pinMode(enRA, INPUT);
  // pinMode(enRB, INPUT);
  // Setup interrupt 
  attachInterrupt(digitalPinToInterrupt(enLA), countEnLA, RISING);
  attachInterrupt(digitalPinToInterrupt(enRA), countEnRA, RISING);

  // attachInterrupt(digitalPinToInterrupt(enRA), rightEnISRA, RISING);
  // attachInterrupt(digitalPinToInterrupt(enRB), rightEnISRB, RISING);

  pinMode(motorRpin1, OUTPUT);
  pinMode(motorRpin2, OUTPUT);
  pinMode(motorLpin1, OUTPUT);
  pinMode(motorLpin2, OUTPUT);
}

void loop() {
  digitalWrite(motorLpin1, LOW);
  analogWrite(motorLpin2, 255);
  digitalWrite(motorRpin1, LOW);
  unsigned long currentMillis = millis();  // Get current time
  // Check if 1 second has passed
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;  // Save the last time pulse count was checked
    rpmLA = calculateRPM(counterLA);    Serial.print("RPM_L = "); Serial.print(rpmLA);
    rpmRA = calculateRPM(counterRA);    Serial.print("; RPM_R = "); Serial.println(rpmRA);
    resetCounter();
    goCircle_clockwise(1);

  }
  
}

void goCircle_clockwise(float radius) {
  //around a circle of 1m radius, vl=1.2vr
  float expected_speed_ratio = (radius + WHEEL_DISTANCE/2)/(radius - WHEEL_DISTANCE/2);
  Serial.print("expect ratio:");  Serial.println(expected_speed_ratio);
  float expected_rpmRA = rpmLA /expected_speed_ratio;
  Serial.print("expect rpmRA:");  Serial.println(expected_rpmRA);
  if (rpmRA < expected_rpmRA) {
      pwmValue += 10; // Increase PWM if speed is too low
      pwmValue = constrain(pwmValue, 0, 255);
    } else if (rpmRA > expected_rpmRA) {
      pwmValue -= 10; // Decrease PWM if speed is too high
      pwmValue = constrain(pwmValue, 0, 255);
    }
  analogWrite(motorRpin2, pwmValue);
  Serial.println(pwmValue);
}

void goForward(float length) {
  int maxPulse = round(7000*length);
  while ((counterLA <= maxPulse) & (counterRA <= maxPulse)) {
      // Serial.print("counterLA: "); Serial.print(counterLA); Serial.print("; counterRA: "); Serial.println(counterRA);
      digitalWrite(motorLpin1, LOW);
      analogWrite(motorLpin2, 255);
      digitalWrite(motorRpin1, LOW);
      analogWrite(motorRpin2, 245);
    }
  stop();
}
void goForward() {
      digitalWrite(motorLpin1, LOW);
      analogWrite(motorLpin2, 255);
      digitalWrite(motorRpin1, LOW);
      analogWrite(motorRpin2, 245);
}
void stop() {
  digitalWrite(motorLpin1, LOW);
  digitalWrite(motorLpin2, LOW);
  digitalWrite(motorRpin1, LOW);
  digitalWrite(motorRpin2, LOW);
}

void countEnLA() {
  counterLA++;
  totalCounterLA++;
}
void countEnRA() {
  counterRA++;
  totalCounterRA++;
}
void resetCounter() {
  counterRA = 0;
  counterLA = 0;
}
int calculateRPM(int pulseCounter) {
  float rpm = (float)pulseCounter * 60 / PULSE_PER_REV;
  return rpm;
}