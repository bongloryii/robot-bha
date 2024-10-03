//position control only, no orientation pose control
//the front of the robot is the L298N, the back is the battery case
int motorRpin1 = 8; //right motor IN1
int motorRpin2 = 9; //right motor IN2

int motorLpin1 = 10; //left motor IN3
int motorLpin2 = 11; //left motor IN4

int enR = 7; 
int enL = 12;

int enLA = 2; //left motor C1
int enLB = 3; //left motor C2

int enRB = 20; //right motor C2
int enRA = 21; //right motor C1

volatile unsigned int counterLA = 0; 
volatile unsigned int counterRA = 0; 
volatile unsigned long totalCounterLA = 0; 
volatile unsigned long totalCounterRA = 0; 
int pwmValue = 150; //preset value for right wheel in circle control
float MAX_VELOCITY = 0.2;
float rpmLA = 0; 
float rpmRA = 0; 

const float WHEEL_DISTANCE = 0.182; //Distance between 2 driving wheels
const int PULSE_PER_REV = 940;
const float WHEEL_PERIMETER = 0.1413;
const float WHEEL_RADIUS = 0.0225;

unsigned long previousMillis = 0;

// motion control
// // pose
float x = 0;
float y = 0;
float theta = 0;
const float T = 0.1; //sampling rate

int interval = T*1000; //delay between rpm calculations in ms

// // goal
float x_g = 1;
float y_g = 1;
float theta_g = 0;

float theta_heading = atan2(y_g-y,x_g-x); 
// control parameter
float gamma = 0.2;
float lamda = 3;
float h = 14;

//motion 
float deltaX, deltaY, rho, phi, alpha, w, vr, vl, wr, wl, v1, v2;
float v = 0.15; //default linear velocity

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
  totalCounterRA = 0; 
  totalCounterLA = 0; 
}

void loop() {
  positionControl_PID();
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
  digitalWrite(motorLpin1, LOW);
  analogWrite(motorLpin2, 255);
  digitalWrite(motorRpin1, LOW);
  analogWrite(motorRpin2, pwmValue);
  Serial.println(pwmValue);
}
void positionControl_feedback_law(){
  unsigned long currentMillis = millis();  // Get current time
  transformCoordinate();
  calculateMotion();
  //set speed
  rightForward(vr);
  leftForward(vl);
  if (currentMillis - previousMillis >= interval) {  // Check if 1 second has passed
    previousMillis = currentMillis;  // Save the last time pulse count was checked
    //get current speed
    rpmLA = calculateRPM(counterLA);    Serial.print("RPM_L = "); Serial.print(rpmLA);
    rpmRA = calculateRPM(counterRA);    Serial.print("; RPM_R = "); Serial.println(rpmRA);
    resetCounter();
    localize();
  }
  if (rho<0.05) {
    // if ((atan2(deltaX, deltaY)<(theta_g+0.1))&&(atan2(deltaX, deltaY)>(theta_g-0.1))) {
    stop();
    delay(3000);
  // }
  }
}

float maxSpeed;
float scalingFactor;
int pwmRight;  // Right motor speed (-255 to 255)
int pwmLeft;// Left motor speed (-255 to 255)
void positionControl_PID(){
  unsigned long currentMillis = millis();  // Get current time
  caculatePIDerror();
  calculateMotion_PID();
  //set speed
  if (pwmRight>0) {
    rightForward(pwmRight);
  } else {
    rightBackward(-pwmRight);
  }
  if (pwmLeft>0) {
    leftForward(pwmLeft);
  } else {
    leftBackward(-pwmLeft);
  }
  if (currentMillis - previousMillis >= interval) {  // Check if 1 second has passed
    previousMillis = currentMillis;  // Save the last time pulse count was checked
    //get current speed
    rpmLA = calculateRPM(counterLA);    
    // Serial.print("RPM_L = "); Serial.print(rpmLA);
    rpmRA = calculateRPM(counterRA);    
    // Serial.print("; RPM_R = "); Serial.println(rpmRA);
    resetCounter();
    localize();
  }
  if (rho<0.05) {
    // if ((atan2(deltaX, deltaY)<(theta_g+0.1))&&(atan2(deltaX, deltaY)>(theta_g-0.1))) {
    stop();
    delay(3000);
  // }
  }
}
float error;
float differenceError;
float currentError;
const float kp = 0.1;
const float ki = 0;
const float kd = 0;
float sumError = 0;
//calculate PID error
float theta_p; //current orientation heading difference
void caculatePIDerror(){
  deltaX = x_g - x;
  deltaY = y_g - y;
  rho = sqrt(pow(deltaX, 2) + pow(deltaY, 2));
  differenceError =  (theta_heading - theta) - currentError;
  currentError  = theta_heading - theta;
  sumError +=currentError;
  error = kp*currentError + ki*sumError + kd *differenceError;
}
// void PID_positionControl()
// w = (v2 - v1) / WHEEL_DISTANCE * T;
void calculateMotion_PID(){
  vr = v + error;
  vl = v - error;
  // Compare the absolute values of the wheel velocities
  maxSpeed = max(abs(vr), abs(vl));


  // Calculate the scaling factor
  scalingFactor = 255.0 / maxSpeed;

  // Scale both velocities, keeping their signs intact
  vr *= scalingFactor;
  vl *= scalingFactor;
  // Now, vr or vl will not exceed 255, and both are scaled proportionally
  pwmRight = int(vr);  // Right motor speed (-255 to 255)
  pwmLeft = int(vl);   // Left motor speed (-255 to 255)
  // wr = vr / WHEEL_RADIUS * 60 / (2 * M_PI);  // angular velocity in rpm
  // wl = vl / WHEEL_RADIUS * 60 / (2 * M_PI);
}
void localize() { //update pose
  //rpm to m/s
  v1 = RPMtoMPS(rpmLA); //left motor
  v2 = RPMtoMPS(rpmRA); //right motor
  x = x + (v1 + v2) / 2 * cos(theta) * T;
  y = y + (v1 + v2) / 2 * sin(theta) * T;
  theta = theta + (v2 - v1) / WHEEL_DISTANCE * T;
}
//feedback control law
void transformCoordinate() {
  deltaX = x_g - x;
  deltaY = y_g - y;
  rho = sqrt(pow(deltaX, 2) + pow(deltaY, 2));
  phi = atan2(deltaY, deltaX) - theta_g; //end pose 
  alpha = atan2(deltaY, deltaX) - theta; //heading to goal
}

void calculateMotion() {
  v = gamma * cos(alpha) * rho;
  w = lamda * alpha + gamma * cos(alpha) * sin(alpha) * (alpha + h * phi) / alpha;
  vr = v + WHEEL_DISTANCE * w / 2;
  vl = v - WHEEL_DISTANCE * w / 2;
  // wr = vr / WHEEL_RADIUS * 60 / (2 * M_PI);  // angular velocity in rpm
  // wl = vl / WHEEL_RADIUS * 60 / (2 * M_PI);
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

void countEnLA() { //update counter and total counter LA
  counterLA++;
  totalCounterLA++;
}
void countEnRA() {//update counter and total counter RA
  counterRA++;
  totalCounterRA++;
}
void resetCounter() {//reset counter LA and LB
  counterRA = 0;
  counterLA = 0;
}
float calculateRPM(int pulseCounter) { //pulse counter to RPM
  float rpm = (float)pulseCounter * 60 / PULSE_PER_REV /T;
  return rpm;
}
int MPStoPWM(float velocity) { //velocity to PWM value
    float pwm = (velocity / MAX_VELOCITY) * 255;
    pwm = (int)constrain(pwm, 30, 255);  
    return pwm;
}
float RPMtoMPS(float rpm){
  float v = rpm / 60 * WHEEL_PERIMETER;
  return v;
}
// float RPMtoPWM(float rpm){
//   float pwm = MPStoPWM(RPMtoMPS(rpm));
//   return pwm;
// }
float RPMtoPWM(float rpm){
  float pwm = (rpm / 90) * 255;
  pwm = (int)constrain(pwm, 30, 255);  
  return pwm;
}
// void rightForward (float velocity) { //set speed of right motor
//   int pwm = (int)MPStoPWM(velocity)*0.95;
//   digitalWrite(motorRpin2, HIGH);
//   digitalWrite(motorRpin1, LOW);
//   analogWrite(enR, pwm);
// }
// void leftForward (float velocity) {//set speed of left motor
//   int pwm = MPStoPWM(velocity);
//   digitalWrite(motorLpin2, HIGH);
//   digitalWrite(motorLpin1, LOW);
//   analogWrite(enL, pwm);
// }
// void rightBackward (float velocity) { //set speed of right motor
//   int pwm = (int)MPStoPWM(velocity)*0.95;
//   digitalWrite(motorRpin1, HIGH);
//   digitalWrite(motorRpin2, LOW);
//   analogWrite(enR, pwm);
// }
// void leftBackward (float velocity) {//set speed of left motor
//   int pwm = MPStoPWM(velocity);
//   digitalWrite(motorLpin1, HIGH);
//   digitalWrite(motorLpin2, LOW);
//   analogWrite(enL, pwm);
// }
void rightForward (int pwm) { //set speed of right motor
  digitalWrite(motorRpin2, HIGH);
  digitalWrite(motorRpin1, LOW);
  analogWrite(enR, pwm);
}
void leftForward (int pwm) {//set speed of left motor
  digitalWrite(motorLpin2, HIGH);
  digitalWrite(motorLpin1, LOW);
  analogWrite(enL, pwm);
}
void rightBackward (int pwm) { //set speed of right motor
  digitalWrite(motorRpin1, HIGH);
  digitalWrite(motorRpin2, LOW);
  analogWrite(enR, pwm);
}
void leftBackward (int pwm) {//set speed of left motor
  digitalWrite(motorLpin1, HIGH);
  digitalWrite(motorLpin2, LOW);
  analogWrite(enL, pwm);
}
double calculateDistance(long counterL, long counterR){
  double pulseVelocity = 0.5*(counterL+counterR);
  double distance = pulseVelocity / (double)PULSE_PER_REV * WHEEL_PERIMETER;
  return distance; 
}