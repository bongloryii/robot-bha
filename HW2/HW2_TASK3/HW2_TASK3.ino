//the front of the robot is the L298N, the back is the battery case
int motorRpin1 = 10; // Right motor IN3
int motorRpin2 = 11; // Right motor IN4

int motorLpin1 = 8; // Left motor IN1
int motorLpin2 = 9; // Left motor IN2

int enR = 13;  // Right motor enable pin (PWM)
int enL = 7; // Left motor enable pin (PWM)

// Encoders
int enLA = 2;  // Left motor encoder A
int enRA = 3; // Right motor encoder A

volatile unsigned int counterLA = 0;
volatile unsigned int counterRA = 0;

volatile float rpmLA = 0; 
volatile float rpmRA = 0; 
const int PULSE_PER_REV = 940;
float length = 1;
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
  unsigned long currentMillis = millis();  // Get current time
  // Check if 1 second has passed
  if (currentMillis - previousMillis >= 1000) {
    previousMillis = currentMillis;  // Save the last time pulse count was checked
    rpmLA = calculateRPM(counterLA);
    Serial.print("RPM_L = "); Serial.print(rpmLA);
    rpmRA = calculateRPM(counterRA);
    Serial.print("; RPM_R = "); Serial.println(rpmRA);
    // resetEncoder();
  }
  goForward();
  Serial.print("counterLA: "); Serial.print(counterLA); Serial.print("; counterRA: "); Serial.println(counterRA);

  // delay(1000);
  
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
      analogWrite(motorRpin2, 100);
}
void stop() {
  digitalWrite(motorLpin1, LOW);
  digitalWrite(motorLpin2, LOW);
  digitalWrite(motorRpin1, LOW);
  digitalWrite(motorRpin2, LOW);
}

void countEnLA() {
  counterLA++;
}
void countEnRA() {
  counterRA++;
}
void resetEncoder() {
  counterRA = 0;
  counterLA = 0;
}
float calculateRPM(int pulseCounter) {
  float rpm = (float)pulseCounter * 60 / PULSE_PER_REV;
  return rpm;
}

