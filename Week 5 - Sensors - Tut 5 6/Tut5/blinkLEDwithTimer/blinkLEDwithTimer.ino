#include "TimerOne.h"
unsigned long countSeconds = 0;
int ledState = 0;
void setup()
{
  pinMode(LED_BUILTIN, OUTPUT);
  Timer1.initialize(1000000);         // initialize timer1, and set a 1/2 second period
  Timer1.pwm(LED_BUILTIN, 512);                // setup pwm on pin 13, 50% duty cycle
  Timer1.attachInterrupt(blinkLED);  // attaches callback() as a timer overflow interrupt
  Serial.begin(9600);
}
 
void blinkLED()
{
  countSeconds +=1;
  Serial.println(countSeconds);
  if (ledState == 0){
    digitalWrite(LED_BUILTIN, HIGH);
    ledState =1;
  } else {
    digitalWrite(LED_BUILTIN, LOW);
    ledState =0;
  }
}
// void printHello(){
//   Serial.println("Hello");
// }
 
void loop()
{  // your program here...
}
 