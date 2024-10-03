//the front of the robot is the L298N, the back is the battery case
int motor1pin1 = 8; //right motor IN1
int motor1pin2 = 9; //right motor IN2

int motor2pin1 = 10; //left motor IN3
int motor2pin2 = 11; //left motor IN4

int enLA = 2; //left motor C1
int enLB = 3; //left motor C2

int enRB = 20; //right motor C2
int enRA = 21; //right motor C1

volatile int counterLA = 0; 
volatile int counterRA = 0; 

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

  pinMode(motor1pin1, OUTPUT);
  pinMode(motor1pin2, OUTPUT);
  pinMode(motor2pin1, OUTPUT);
  pinMode(motor2pin2, OUTPUT);
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
    resetEncoder();
  }
  goForward();
  Serial.print("counterLA: "); Serial.print(counterLA); Serial.print("; counterRA: "); Serial.println(counterRA);

  // delay(1000);
  
}
void goForward(float length) {
  int maxPulse = round(7000*length);
  while ((counterLA <= maxPulse) & (counterRA <= maxPulse)) {
      // Serial.print("counterLA: "); Serial.print(counterLA); Serial.print("; counterRA: "); Serial.println(counterRA);
      digitalWrite(motor2pin1, LOW);
      analogWrite(motor2pin2, 255);
      digitalWrite(motor1pin1, LOW);
      analogWrite(motor1pin2, 245);
    }
  stop();
}
void goForward() {
      digitalWrite(motor2pin1, LOW);
      analogWrite(motor2pin2, 255);
      digitalWrite(motor1pin1, LOW);
      analogWrite(motor1pin2, 100);
}
void stop() {
  digitalWrite(motor2pin1, LOW);
  digitalWrite(motor2pin2, LOW);
  digitalWrite(motor1pin1, LOW);
  digitalWrite(motor1pin2, LOW);
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

