// // Define motor pins and encoders
// int motorRpin1 = 10; // Right motor IN3
// int motorRpin2 = 11; // Right motor IN4//fw 

// int motorLpin1 = 8; // Left motor IN1
// int motorLpin2 = 9; // Left motor IN2//fw

// int enR = 13;  // Right motor enable pin (PWM)
// int enL = 7; // Left motor enable pin (PWM)

// // Encoders
// int enLA = 2;  // Left motor encoder A
// int enRA = 3; // Right motor encoder A

// volatile unsigned int counterLA = 0;
// volatile unsigned int counterRA = 0;

// const float MAX_SPEED = 0.2;
// const float WHEEL_DISTANCE = 0.182; // Distance between two wheels
// const int PULSE_PER_REV = 940;
// const float WHEEL_PERIMETER = 0.1413; // Circumference of the wheel
void setupMotors(){
  
  // Set up motor and encoder pins
  pinMode(motorRpin1, OUTPUT);
  pinMode(motorRpin2, OUTPUT);
  pinMode(motorLpin1, OUTPUT);
  pinMode(motorLpin2, OUTPUT);
  
  pinMode(enLA, INPUT);
  pinMode(enRA, INPUT);

  attachInterrupt(digitalPinToInterrupt(enLA), countEnLA, RISING);
  attachInterrupt(digitalPinToInterrupt(enRA), countEnRA, RISING);
}
void setSpeed(float vr, float vl){
  // Control motor speeds
  if (vr > 0) {rightForward(vr);}
  else {rightBackward(-vr);}
  
  if (vl > 0) {leftForward(vl);}
  else {leftBackward(-vl);}
}
// Motor control functions
void rightForward(float velocity) {
  int pwm = MPStoPWM(velocity);
  Serial.print("pwmr:"); Serial.println(pwm);
  digitalWrite(motorRpin1, LOW);
  digitalWrite(motorRpin2, HIGH);
  analogWrite(enR, pwm);
}

void leftForward(float velocity) {
  int pwm = MPStoPWM(velocity);
  Serial.print("pwml:"); Serial.println(pwm);
  digitalWrite(motorLpin1, LOW);
  digitalWrite(motorLpin2, HIGH);
  analogWrite(enL, pwm);
}

void rightBackward(float velocity) {
  int pwm = MPStoPWM(velocity);
  Serial.print("pwmr:-"); Serial.println(pwm);

  digitalWrite(motorRpin1, HIGH);
  digitalWrite(motorRpin2, LOW);
  analogWrite(enR, pwm);
}

void leftBackward(float velocity) {
  int pwm = MPStoPWM(velocity);
  Serial.print("pwml:-"); Serial.println(pwm);

  digitalWrite(motorLpin1, HIGH);
  digitalWrite(motorLpin2, LOW);
  analogWrite(enL, pwm);
}

void goForward() {
  digitalWrite(motorLpin1, LOW);
  analogWrite(motorLpin2, 255);
  digitalWrite(motorRpin1, LOW);
  analogWrite(motorRpin2, 255);
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
// Stop motors
void stop() {
  digitalWrite(motorLpin1, LOW);
  digitalWrite(motorLpin2, LOW);
  digitalWrite(motorRpin1, LOW);
  digitalWrite(motorRpin2, LOW);
}

// Helper functions
float calculateRPM(int pulseCounter) {
  return (float)pulseCounter * 60 / PULSE_PER_REV / T;
}

float RPMtoMPS(float rpm) {
  return rpm / 60 * WHEEL_PERIMETER;
}

int MPStoPWM(float velocity) {
  return constrain((velocity / MAX_SPEED) * 255, 30, 255); // Mapping speed to PWM
}

void countEnLA() {
  counterLA++;
}

void countEnRA() {
  counterRA++;
}

void resetCounters() {
  counterLA = 0;
  counterRA = 0;
}

void goCircle(float radius, float velocity =MAX_SPEED) { //plus is clockwise
  //around a circle of 1m radius, vl=1.2vr
  if (radius > 0 ){
    float expected_speed_ratio = (radius + WHEEL_DISTANCE/2)/(radius - WHEEL_DISTANCE/2);
    setSpeed(velocity/expected_speed_ratio,velocity);
    }
    else {
    float expected_speed_ratio = (-radius + WHEEL_DISTANCE/2)/(-radius - WHEEL_DISTANCE/2);
    setSpeed(velocity, velocity/expected_speed_ratio);
    }
}