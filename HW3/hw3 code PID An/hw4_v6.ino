// Goal position
const float goalX = 1.0;
const float goalY = 1.0;
const float goalTheta = 0.0;

// PID values for angle
float controlSignal = 0;

const int Kp = 10;

float angleError = 0;

// Encoder settings
int enRC1 = 21; //right motor C1
int enRC2 = 20; //right motor C2
int enLC1 = 2; // left motor C1
int enLC2 = 3; // left motor C2
volatile unsigned int counterL = 0;
volatile unsigned int counterR = 0;

// Motor control pins
const int enableR = 7, forwardPinRight = 9, backwardPinRight = 8;
const int enableL = 12, forwardPinLeft = 11, backwardPinLeft = 10;

// Robot state
float x, y, theta;
float vleft = 0; // speed in mps
float vright = 0;
float vbase = 0.1;
float wbase = 0;

const int wheelRadius = 0.0225;
const int wheelLength = 0.182;

// Timing
const int deltaT = 0.1;

// Random variables
float deltaX = 0;
float deltaY = 0;
float rho = 0;

void setup() {
  // put your setup code here, to run once:
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
}

void loop() {
  // put your main code here, to run repeatedly:
  calculateError();
  calculateMotion();
  controlMotor(vleft, vright);
  updatePosition();

  if (rho < 0.05) {
    stop();
    delay(3000);
  }
}

void calculateError() {
  deltaX = goalX - x;
  deltaY = goalY - y;
  rho = sqrt(pow(deltaX, 2) + pow(deltaY, 2));
  angleError = goalTheta - (atan2(deltaY, deltaX));
  controlSignal = Kp * angleError;
}

void calculateMotion() {
  vleft = vbase - controlSignal;
  vright = vbase + controlSignal;
}

void updatePosition() {
  unsigned long currentTime = millis();
  unsigned long previousTime = 0;
  if (currentTime - previousTime >= 100) {
    previousTime = currentTime;

    vleft = (counterL / 2000) * 2 * PI * wheelRadius / 0.1;
    vright = (counterR / 2000) * 2 * PI * wheelRadius / 0.1;

    resetCounter();

    x += ((vleft + vright) / 2) * cos(theta) * 0.1;
    y += ((vleft + vright) / 2) * sin(theta) * 0.1;
    theta += ((vright - vleft) / wheelLength) * 0.1;
  }
}

float convertMPStoRPM(float mps) {
  float rpm = float(mps * 60 / 0.1413);
  return rpm;
}

int convertRPMtoPWM(float rpm) {
  // Convert meters per second to PWM value (example conversion)
  int pwm = (int)(-0.0017 * pow(rpm, 2) + 0.7764 * rpm + 18.387); // Adjust this scaling factor based on your motor specs
  pwm = constrain(pwm, 0, 255); // Constrain between min and max PWM
  if (pwm < 30) {
    pwm = 0;
  };
  return pwm;
}

void controlMotor(float vleft, float vright) {
  int pwmLeft = convertRPMtoPWM(convertMPStoRPM(abs(vleft)));
  int pwmRight = convertRPMtoPWM(convertMPStoRPM(abs(vright)));

  // Control the left motor
  if (vleft > 0) {
    // Move left wheel forward
    analogWrite(enableL, pwmLeft);
    digitalWrite(forwardPinLeft, HIGH);
    digitalWrite(backwardPinLeft, LOW);
  } else if (vleft < 0) {
    // Move left wheel backward
    analogWrite(enableL, pwmLeft);
    digitalWrite(forwardPinLeft, LOW);
    digitalWrite(backwardPinLeft, HIGH);
  } else {
    // Stop left wheel
    analogWrite(enableL, 0);
  }

  // Control the right motor
  if (vright > 0) {
    // Move right wheel forward
    analogWrite(enableR, pwmRight);
    digitalWrite(forwardPinRight, HIGH);
    digitalWrite(backwardPinRight, LOW);
  } else if (vright < 0) {
    // Move right wheel backward
    analogWrite(enableR, pwmRight);
    digitalWrite(forwardPinRight, LOW);
    digitalWrite(backwardPinRight, HIGH);
  } else {
    // Stop right wheel
    analogWrite(enableR, 0);
  }
}

void stop() {
  digitalWrite(forwardPinRight, LOW);
  digitalWrite(backwardPinRight, LOW);
  digitalWrite(forwardPinLeft, LOW);
  digitalWrite(backwardPinLeft, LOW);
}

void countEnRC1() { //update counter and total counter R
  counterR++;
}
void countEnRC2() {
  counterR++;
}
void countEnLC1() {//update counter and total counter L
  counterL++;
}
void countEnLC2() {
  counterL++;
}
void resetCounter() {//reset counter LA and LB
  counterR = 0;
  counterL = 0;
}
