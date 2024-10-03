const int eye1 = 45;
const int eye2 = 47;
const int eye3 = 49;
const int eye4 = 51;
const int eye5 = 53;

void setup() {
  pinMode(eye1, INPUT);
  pinMode(eye2, INPUT);
  pinMode(eye3, INPUT);
  pinMode(eye4, INPUT);
  pinMode(eye5, INPUT);

  Serial.begin(9600);
}

void loop() {
  int eye1State = digitalRead(eye1);
  int eye2State = digitalRead(eye2);
  int eye3State = digitalRead(eye3);
  int eye4State = digitalRead(eye4);
  int eye5State = digitalRead(eye5);

  Serial.print(eye1State == HIGH ? "ON   " : "OFF  ");

  Serial.print(eye2State == HIGH ? "ON   " : "OFF  ");

  Serial.print(eye3State == HIGH ? "ON   " : "OFF  ");

  Serial.print(eye4State == HIGH ? "ON   " : "OFF  ");

  Serial.print(eye5State == HIGH ? "ON   " : "OFF  ");

  Serial.println();
  
  delay(500);
}
