#include <TimerOne.h>

// Pin Definitions
int motorRpin1 = 10; // Right motor IN1
int motorRpin2 = 11; // Right motor IN2
int motorLpin1 = 8;  // Left motor IN3
int motorLpin2 = 9;  // Left motor IN4
int enR = 13; 
int enL = 7;
int enLA = 2; // Left motor encoder C1
int enRA = 3; // Right motor encoder C1

// Encoder Counters
volatile unsigned int counterLA = 0; 
volatile unsigned int counterRA = 0; 

// Constants
const float WHEEL_DISTANCE = 0.182; // Distance between two driving wheels
const int PULSE_PER_REV = 1050;
const float WHEEL_PERIMETER = 0.1413;
const float T = 0.1; // Sampling rate (seconds)

// Variables
float x = 0, y = 0, theta = 0;
float distanceL = 0, distanceR = 0;

void setup() {
  Serial.begin(9600);

  // Pin Modes
  pinMode(enLA, INPUT);
  pinMode(enRA, INPUT);
  pinMode(motorRpin1, OUTPUT);
  pinMode(motorRpin2, OUTPUT);
  pinMode(motorLpin1, OUTPUT);
  pinMode(motorLpin2, OUTPUT);

  // Interrupts for Encoders
  attachInterrupt(digitalPinToInterrupt(enLA), countEnLA, RISING);
  attachInterrupt(digitalPinToInterrupt(enRA), countEnRA, RISING);

  // Timer Initialization
  Timer1.initialize(T * 200000); // Set timer interval (T seconds in microseconds)
  Timer1.attachInterrupt(positionControl); // Attach positionControl() to the timer
}

void loop() {
  recPath(); // Continue executing recPath()
  while(1);
  x=0;
  y=0;
  theta=0;
}

void recPath() {
  goForward(4000); // Move forward for 4 seconds
  stop();
  delay(1000); // Small pause
  turnRight(); // Turn right
  goForward(2500); // Move forward for 2.5 seconds
  stop();
  delay(1000);
  turnRight();
  goForward(4000);
  stop();
  delay(1000);
  turnRight();
  goForward(2000);
  stop();
  delay(1000);
  turnRight();
  stop();
  while(1);
}

void positionControl() {
  // Calculate distances
  distanceL = (float)counterLA / PULSE_PER_REV * WHEEL_PERIMETER;
  distanceR = (float)counterRA / PULSE_PER_REV * WHEEL_PERIMETER;

  // Update pose
  x += (distanceL + distanceR) / 2 * cos(theta);
  y += (distanceL + distanceR) / 2 * sin(theta);
  theta += (distanceR - distanceL) / WHEEL_DISTANCE;

  // Reset counters
  counterLA = 0;
  counterRA = 0;

  // Print updated position
  Serial.print(x);
  Serial.print(",");
  Serial.println(y);
  // Serial.println("new");
}

void goForward(int duration) {
  digitalWrite(motorRpin2, HIGH);
  digitalWrite(motorRpin1, LOW);
  analogWrite(enR, 255);
  digitalWrite(motorLpin2, HIGH);
  digitalWrite(motorLpin1, LOW);
  analogWrite(enL, 255);
  delay(duration);
}

void turnRight() {
  digitalWrite(motorRpin2, HIGH);
  digitalWrite(motorRpin1, LOW);
  analogWrite(enR, 0);
  digitalWrite(motorLpin2, HIGH);
  digitalWrite(motorLpin1, LOW);
  analogWrite(enL, 240);
  delay(1400);
}

void stop() {
  digitalWrite(motorLpin1, LOW);
  digitalWrite(motorLpin2, LOW);
  digitalWrite(motorRpin1, LOW);
  digitalWrite(motorRpin2, LOW);
}

void countEnLA() { counterLA++; }
void countEnRA() { counterRA++; }
