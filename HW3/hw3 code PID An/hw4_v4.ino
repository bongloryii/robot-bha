const int goalX = 1;
const int goalY = 1;
const int goaltheta = 0;
float x, y, theta;
float KpDistance = 1.0, KiDistance = 0.0, KdDistance = 0.0;
float KpAngle = 1.0, KiAngle = 0.0, KdAngle = 0.0;

float derivAngleError = 0.0;
float sumAngleError = 0.0;
float AngleError = 0.0;

float controlSignal = 0;

float derivDistanceError = 0.0;
float sumDistanceError = 0.0;
float DistanceError = 0.0;
// Encoder settings
int enRC1 = 21; //right motor C1
int enRC2 = 20; //right motor C2

int enLC1 = 2; // left motor C1
int enLC2 = 3; // left motor C2
const int maxPWMRight = 225; // Maximum PWM value for right motor
const int minSpeed = 30;  // Minimum speed constraint for both motors
volatile unsigned int counterL = 0;
volatile unsigned int counterR = 0;
volatile unsigned long totalCounterL = 0;
volatile unsigned long totalCounterR = 0;
void countEnRC1();
void countEnRC2();
void countEnLC1();
void countEnLC2();

// Motor control pins
const int enableR = 7;   // Right motor enable pin (PWM)
const int forwardPinRight = 9;  // Right motor forward
const int backwardPinRight = 8; // Right motor backward

const int enableL = 12;   // Left motor enable pin (PWM)
const int forwardPinLeft = 11;  // Left motor forward
const int backwardPinLeft = 10; // Left motor backward
const int maxPWM = 255; // Maximum PWM value for motor speed
const float tolerance = 0.01;  // Tolerance to consider the controlSignal as zero (aligned)
float vleft = 0;
float vright = 0;
float vbase = 0;
float wbase = 0;
// Constants for RPM calculation
float revL = 0;
float revR = 0;
float rpmL = 0;
float rpmR = 0;
const int pulsesPerRevolution = 2000;  // 2000 pulses per revolution due to 2 encoders
unsigned long lastRPMCheckTime = 0;
float currentRPM = 0;
const int deltaT = 0.1;

float gamma = 3;
float lambda = 6;
float h = 0.2;

const int wheelRadius = 0.0225;
const int wheelLength = 0.182;

const int leftPulsesPerM = 14000;  // Number of pulses to travel 1 meter for the left wheel
const int rightPulsesPerM = 14000; // Number of pulses to travel 1 meter for the right wheel

// Time management
unsigned long previousTime = 0;
unsigned long currentTime = 0;

void setup() {
  // Set motor pins as outputs
  pinMode(enableR, OUTPUT);
  pinMode(forwardPinRight, OUTPUT);
  pinMode(backwardPinRight, OUTPUT);

  pinMode(enableL, OUTPUT);
  pinMode(forwardPinLeft, OUTPUT);
  pinMode(backwardPinLeft, OUTPUT);

  pinMode(enRC1, INPUT);
  pinMode(enRC2, INPUT);
  pinMode(enLC1, INPUT);
  pinMode(enLC2, INPUT);


  // Set up encoder interrupt
  attachInterrupt(digitalPinToInterrupt(enRC1), countEnRC1, RISING);
  attachInterrupt(digitalPinToInterrupt(enRC2), countEnRC2, RISING);
  attachInterrupt(digitalPinToInterrupt(enLC1), countEnLC1, RISING);
  attachInterrupt(digitalPinToInterrupt(enLC2), countEnLC2, RISING);


  Serial.begin(9600);
  totalCounterR = 0;
  totalCounterL = 0;
}

void loop() {
  positionControl();
}
void positionControl() {
 controlSignal = calculatepidAngle(derivAngleError, sumAngleError, AngleError);
  currentTime = millis();
  if (currentTime - previousTime >= 100) { // Update every 0.1 seconds
    previousTime = currentTime;

    // Calculate wheel speeds based on encoder counters
    vleft = (counterL / pulsesPerRevolution) * 2 * PI * wheelRadius / deltaT;
    vright = (counterR / pulsesPerRevolution) * 2 * PI * wheelRadius / deltaT;

    // Reset counters after using them
    counterL = 0;
    counterR = 0;

    // Constrain speeds to ensure they are above the minimum speed
    if (vleft < minSpeed) vleft = minSpeed;
    if (vright < minSpeed) vright = minSpeed;

    // Calculate base linear and angular velocities
    vbase = (vleft + vright) / 2;
    wbase = (vright - vleft) / wheelLength;

    // Calculate motion
    calculateMotion();

    // Update motor speeds using PID control
    updateMotors(vleft, vright);

    // Update the robot's position
    updatePosition();
  }
}

void updatePosition() {
  x += (vleft + vright) / 2 * cos(theta) * deltaT;
  y += (vleft + vright) / 2 * sin(theta) * deltaT;
  theta += (1 / 0.182) * (vright - vleft);
}

// Function to calculate PID for distance
float calculatepidDistance(float derivDistanceError, float sumDistanceError, float DistanceError) {
  // PID constants for distance

  // Use passed DistanceError for derivative calculation
  float prevDistanceError = DistanceError; // Store previous error

  // Update the proportional (DistanceError)
  DistanceError = sqrt(pow(goalX - x, 2) + pow(goalY - y, 2)); // Proportional

  // Update the derivative (rate of change of error)
  derivDistanceError = DistanceError - prevDistanceError; // Derivative

  // Update the integral (accumulation of errors)
  sumDistanceError += DistanceError; // Integral

  // Calculate PID for distance
  float pidDistance = KpDistance * DistanceError   + KiDistance * sumDistanceError  + KdDistance * derivDistanceError;
  return pidDistance;
}

// Function to calculate PID for angle
float calculatepidAngle(float derivAngleError, float sumAngleError, float AngleError) {
  // PID constants for angle

  // Use passed AngleError for derivative calculation
  float prevAngleError = AngleError; // Store previous error

  // Update the proportional (AngleError)
  AngleError = goaltheta - theta; // Proportional

  // Update the derivative (rate of change of error)
  AngleError = AngleError - prevAngleError; // Derivative

  // Calculate PID for angle
  float pidAngle = KpAngle * AngleError + KiAngle * sumAngleError  + KdAngle * derivAngleError;
  return pidAngle;
}

void calculateMotion() {
  // Calculate the angle PID output (assuming you update errors elsewhere)
  float pidAngle = calculatepidAngle(derivAngleError, sumAngleError, AngleError);

  // Calculate the distance PID output
  float pidDistance = calculatepidDistance(derivDistanceError, sumDistanceError, DistanceError);

  vbase = gamma * cos(pidAngle) * pidDistance;
  wbase = lambda * pidAngle + gamma * cos(pidAngle) * sin(pidAngle); // ignoring phi for now
}
void updateMotors(float vleft, float vright) {
  int pwmL = constrain(vleft, 30, maxPWMRight);
  int pwmR = constrain(vright, 30, maxPWMRight);

  analogWrite(enableL, pwmL);
  analogWrite(enableR, pwmR);
}
void countEnRC1() { //update counter and total counter R
  counterR++;
  totalCounterR++;
}
void countEnRC2() {
  counterR++;
  totalCounterR++;
}
void countEnLC1() {//update counter and total counter L
  counterL++;
  totalCounterL++;
}
void countEnLC2() {
  counterL++;
  totalCounterL++;
}
