/*
 * Kết nối:
            HCSR04                          Arduino
            VCC                               5V
            GND                               GND
            TRIG                              8
            ECHO                              7

   Nạp code mở Serial Monitor chọn No line ending, baud 9600.
 */
 #include <Servo.h>
Servo servo;
#include <HCSR04.h>

const int trig = 49;     // chân trig của HC-SR04
const int echo = 48;     // chân echo của HC-SR04
 UltraSonicDistanceSensor distanceSensor(trig, echo);

void setup()
{
    servo.attach(5);
    servo.write(0);
    Serial.begin(9600);     // giao tiếp Serial với baudrate 9600
    // pinMode(trig,OUTPUT);   // chân trig sẽ phát tín hiệu
    // pinMode(echo,INPUT);    // chân echo sẽ nhận tín hiệu
}
 
void loop()
{
  //   unsigned long duration; // biến đo thời gian
    float distance;           // biến lưu khoảng cách
    
  //   // /* Phát xung từ chân trig */
  //   // digitalWrite(trig,0);   // tắt chân trig
  //   // delayMicroseconds(2);
  //   // digitalWrite(trig,1);   // phát xung từ chân trig
  //   // delayMicroseconds(5);   // xung có độ dài 5 microSeconds
  //   // digitalWrite(trig,0);   // tắt chân trig
    
  //   // /* Tính toán thời gian */
  //   // // Đo độ rộng xung HIGH ở chân echo. 
  //   // duration = pulseIn(echo,HIGH);  
  //   // Tính khoảng cách đến vật.
  distance = distanceSensor.measureDistanceCm();
  // delay(100);
    if (distance >0 && distance <5)
    {
     servo.write(25);
      delay(2000);
    } else{
      servo.write(1);
      delay(1000);
    }
    // /* In kết quả ra Serial Monitor */
    Serial.print(distance);
    Serial.println("cm");
    // delay(200);
}
