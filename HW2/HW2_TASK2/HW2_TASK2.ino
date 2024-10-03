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

float length = 1;
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
  resetEncoder();

  // put your main code here, to run repeatedly:
  goForward(1);
  delay(2000);
  
}
void goForward(float length) {
  int maxPulse = round(7000*length);
  while ((counterLA <= maxPulse) & (counterRA <= maxPulse)) {
      Serial.print("counterLA: "); Serial.print(counterLA); Serial.print("; counterRA: "); Serial.println(counterRA);
      digitalWrite(motor2pin1, LOW);
      analogWrite(motor2pin2, 255);
      digitalWrite(motor1pin1, LOW);
      analogWrite(motor1pin2, 245);
    }
  stop();
}
void countEnLA() {
  counterLA++;
}
void stop() {
  digitalWrite(motor2pin1, LOW);
  digitalWrite(motor2pin2, LOW);
  digitalWrite(motor1pin1, LOW);
  digitalWrite(motor1pin2, LOW);
}
// void countEnLB() {
//   counterLB++;
// }
void countEnRA() {
  counterRA++;
}
void resetEncoder() {
  counterRA = 0;
  counterLA = 0;
}
