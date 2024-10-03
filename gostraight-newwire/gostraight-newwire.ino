int motorRpin1 = 10; // Right motor IN3
int motorRpin2 = 11; // Right motor IN4

int motorLpin1 = 8; // Left motor IN1
int motorLpin2 = 9; // Left motor IN2

int enR = 12;  // Right motor enable pin (PWM)
int enL = 7; // Left motor enable pin (PWM)

const float T = 0.1; // Sampling rate

const float WHEEL_DISTANCE = 0.182; // Distance between two wheels
const int PULSE_PER_REV = 940;
const float WHEEL_PERIMETER = 0.1413; // Circumference of the wheel

void setup() {
  // put your setup code here, to run once:

  // Set up motor and encoder pins
  pinMode(motorRpin1, OUTPUT);
  pinMode(motorRpin2, OUTPUT);
  pinMode(motorLpin1, OUTPUT);
  pinMode(motorLpin2, OUTPUT);
  
}

void loop() {
  leftForward(0.2);
  rightForward(0.2);
  // put your main code here, to run repeatedly:

}

void rightForward(float velocity) {
  int pwm = MPStoPWM(velocity);
  digitalWrite(motorRpin1, LOW);
  digitalWrite(motorRpin2, HIGH);
  analogWrite(enR, pwm);
}

void leftForward(float velocity) {
  int pwm = MPStoPWM(velocity);
  digitalWrite(motorLpin1, LOW);
  digitalWrite(motorLpin2, HIGH);
  analogWrite(enL, pwm);
}

void rightBackward(float velocity) {
  int pwm = MPStoPWM(velocity);
  digitalWrite(motorRpin1, HIGH);
  digitalWrite(motorRpin2, LOW);
  analogWrite(enR, pwm);
}

void leftBackward(float velocity) {
  int pwm = MPStoPWM(velocity);
  digitalWrite(motorLpin1, HIGH);
  digitalWrite(motorLpin2, LOW);
  analogWrite(enL, pwm);
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
  return constrain((velocity / 0.2) * 255, 30, 255); // Mapping speed to PWM
}