int motor1Fw = 8;
int motor1Rv = 9;

int motor2Fw = 10;
int motor2Rv = 11;


int encoderC1 = 2; //Encoder Output 'A' must connected with intreput pin of arduino.
int encoderC2 = 3; //Encoder Otput 'B' must connected with intreput pin of arduino.
volatile int lastEncoded = 0; // Here updated value of encoder store.
volatile long encoderValue = 0; // Raw encoder value


void setup() {
  // put your setup code here, to run once:
  pinMode(motor1Fw, OUTPUT);
  pinMode(motor1Rv, OUTPUT);
  pinMode(motor2Fw,  OUTPUT);
  pinMode(motor2Rv, OUTPUT);
  Serial.begin(9600); //initialize serial comunication

  pinMode(encoderC1, INPUT_PULLUP); 
  pinMode(encoderC2, INPUT_PULLUP);

  //call updateEncoder() when any high/low changed seen
  //on interrupt 0 (pin 2), or interrupt 1 (pin 3) 
  attachInterrupt(0, updateEncoder, CHANGE); 
  attachInterrupt(1, updateEncoder, CHANGE);

  //(Optional)
  // pinMode(9,   OUTPUT); 
  // pinMode(10, OUTPUT);
  // //(Optional)
}

void loop() {
   // put your main code here, to run repeatedly:

  //Controlling speed (0   = off and 255 = max speed):     
  //(Optional)
  // analogWrite(9, 100); //ENA   pin
  // analogWrite(10, 200); //ENB pin
  //(Optional)
  
  digitalWrite(motor1pin1,   HIGH);
  digitalWrite(motor1pin2, LOW);

  digitalWrite(motor2pin1, HIGH);
   digitalWrite(motor2pin2, LOW);
  delay(3000);

  digitalWrite(motor1pin1,   LOW);
  digitalWrite(motor1pin2, HIGH);

  digitalWrite(motor2pin1, HIGH);
   digitalWrite(motor2pin2, LOW);
  delay(3000);


  for (int i = 0; i <= 500; i++){
    digitalWrite(MotFwd, LOW); 
    digitalWrite(MotRev, HIGH);
    Serial.print("Forward  ");
    Serial.println(encoderValue);
  }

  delay(1000);

  for (int i = 0; i <= 500; i++){
    digitalWrite(MotFwd, HIGH); 
    digitalWrite(MotRev, LOW);
    Serial.print("Reverse  ");
    Serial.println(encoderValue);
  }

  delay(1000);  

}

void updateEncoder(){
  // int MSB = digitalRead(encoderPin1); //MSB = most significant bit
  // int LSB = digitalRead(encoderPin2); //LSB = least significant bit

  // int encoded = (MSB << 1) |LSB; //converting the 2 pin value to single number
  // int sum  = (lastEncoded << 2) | encoded; //adding it to the previous encoded value

  // if(sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) encoderValue --;
  // if(sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) encoderValue ++;

  // lastEncoded = encoded; //store this value for next time
  int sum 
}

