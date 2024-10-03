// Mega2560
// external interrupt int.0    int.1    int.2   int.3   int.4   int.5            
// pin                  2         3      21      20      19      18

// Left Motor
int enL = 7;
int inL1 = 8;
int inL2 = 9;

// Right motor
int enR = 12;
int inR1 = 10;
int inR2 = 11;

// For encoder
int enLA = 2;
int enLB = 3;

int enRA = 18;
int enRB = 19;

volatile int leftEnCount = 0;
volatile int rightEnCount = 0;
float rpm1 = 0;
float rpm2 = 0;

const int K = 30;  //adjust K for smooth response

void setup()
{
  Serial.begin(9600);

  // Setup interrupt 
  attachInterrupt(digitalPinToInterrupt(enLA), leftEnISRA, RISING);
  attachInterrupt(digitalPinToInterrupt(enLB), leftEnISRB, RISING);

  attachInterrupt(digitalPinToInterrupt(enRA), rightEnISRA, RISING);
  attachInterrupt(digitalPinToInterrupt(enRB), rightEnISRB, RISING);

  // Set all the motor control pins to outputs
	pinMode(enR, OUTPUT);
	pinMode(enL, OUTPUT);
	pinMode(inR1, OUTPUT);
	pinMode(inR2, OUTPUT);
	pinMode(inL1, OUTPUT);
	pinMode(inL2, OUTPUT);
	
	// Turn off motors - Initial state
	digitalWrite(inR1, LOW);
	digitalWrite(inR2, LOW);
	digitalWrite(inL1, LOW);
	digitalWrite(inL2, LOW);

}

void loop() {
  Serial.println(leftEnCount);
  Serial.println(rightEnCount);
  
//  turnLeft();
  goForward(128);
  delay(100);
}

void goForward(int speed) {
  // Reset encoder counter
  rightEnCount = 0;
  leftEnCount = 0;

	// For PWM maximum possible values are 0 to 255
	analogWrite(enR, speed);

  int motor_L_speed = speed + K*(rightEnCount-leftEnCount);  
  analogWrite(enL, motor_L_speed);

	// Turn on motor A & B
	digitalWrite(inL1, LOW);
	digitalWrite(inL2, HIGH);
	digitalWrite(inR1, HIGH);
	digitalWrite(inR2, LOW);
}

void stop() {
	// Turn off motors 
	digitalWrite(inR1, LOW);
	digitalWrite(inR2, LOW);
	digitalWrite(inL1, LOW);
	digitalWrite(inL2, LOW);
}


void turnRight() {
  // Reset encoder counter
  rightEnCount = 0;
  leftEnCount = 0;

  int speed = 100;
  const int turnWeight = 2;
	analogWrite(enR, speed);

  int motor_L_speed = turnWeight*speed + K*(turnWeight*rightEnCount-leftEnCount);  
  analogWrite(enL, motor_L_speed);

	// Turn on motor A & B
	digitalWrite(inL1, LOW);
	digitalWrite(inL2, HIGH);
	digitalWrite(inR1, HIGH);
	digitalWrite(inR2, LOW);  
}

void turnLeft() {
  // Reset encoder counter
  rightEnCount = 0;
  leftEnCount = 0;

  int speed = 100;
  const int turnWeight = 2;
	analogWrite(enL, speed);

  int motor_R_speed = turnWeight*speed + K*(turnWeight*leftEnCount-rightEnCount);  
  analogWrite(enR, motor_R_speed);

	// Turn on motor A & B
	digitalWrite(inL1, LOW);
	digitalWrite(inL2, HIGH);
	digitalWrite(inR1, HIGH);
	digitalWrite(inR2, LOW);  
}

void leftEnISRA() {
  leftEnCount++;
}

void leftEnISRB() {
  leftEnCount++;
}
void rightEnISRA() {
  rightEnCount++;
}

void rightEnISRB() {
  rightEnCount++;
}