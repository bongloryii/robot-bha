//use timer to read imu data every 500ms
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include "TimerOne.h"
int readIMU = 0;
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x29, &Wire);
float normalizeAngle(float angle) {
  // Normalize the angle to the range [-π, π]
  while (angle > M_PI) {
    angle -= 2 * M_PI;
  }
  while (angle < -M_PI) {
    angle += 2 * M_PI;
  }
  return round(angle * 100) / 100.0;
}
void setup() 
{
  Timer1.initialize(100000);    //500ms     
  Timer1.attachInterrupt(readIMUdata);  
  
  Serial.begin(9600);
  Serial.println("Orientation Sensor Test"); Serial.println("");
  
  /* Initialise the sensor */
  if(!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */

    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }    
  bno.setExtCrystalUse(true);
}
void loop(void) 
{
  if (readIMU ==1) {
    readIMU=0;
  sensors_event_t event;
  bno.getEvent(&event);
  /* Display the floating point data */
  Serial.print("X: ");
  Serial.print(normalizeAngle(-event.orientation.x/360*2*3.14), 4);
  // Serial.print("\tY: ");
  // Serial.print(event.orientation.y, 4);
  // Serial.print("\tZ: ");
  // Serial.print(event.orientation.z, 4);
  Serial.println("");
  }
  
}
void readIMUdata(){ 
  readIMU = 1;
}