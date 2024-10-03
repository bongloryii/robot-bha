int motor1pin1 = 8;
int motor1pin2 = 9;

int motor2pin1 = 10;
int  motor2pin2 = 11;

int enLA = 2;
int enLB = 3;

int enRA = 20;
int enRB = 21;

int counter = 0; 

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(enLA, INPUT);
  // pinMode(enLB, INPUT);
  // pinMode(enRA, INPUT);
  // pinMode(enRB, INPUT);
  // Setup interrupt 
  attachInterrupt(digitalPinToInterrupt(enLA), myInterruptFunction, RISING);
  // attachInterrupt(digitalPinToInterrupt(enLB), leftEnISRA, RISING);

  // attachInterrupt(digitalPinToInterrupt(enRA), rightEnISRA, RISING);
  // attachInterrupt(digitalPinToInterrupt(enRB), rightEnISRB, RISING);

  pinMode(motor1pin1, OUTPUT);
  pinMode(motor1pin2, OUTPUT);
  pinMode(motor2pin1,  OUTPUT);
  pinMode(motor2pin2, OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  counter = 0;
  while (counter <= 940) {
    Serial.println(counter);
    digitalWrite(motor2pin1, 128);
    digitalWrite(motor2pin2, LOW);
  }
  digitalWrite(motor2pin1, LOW);
  digitalWrite(motor2pin2, LOW);
  delay(2000);
  
}

void myInterruptFunction() {
  counter++;
}
