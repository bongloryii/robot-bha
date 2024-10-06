
//use timer to read imu data every 500ms
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include "TimerOne.h"
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x29, &Wire);

float x_imu, y_imu, z_imu;

void setupIMU() 
{
  Timer1.initialize(200000);    //500ms     
  Timer1.attachInterrupt(isReadIMUdata);  

  Serial.begin(9600);
  Serial.println("Orientation Sensor Test"); Serial.println("");
  
  // Initialise the sensor 
  if(!bno.begin())
  {
    //There was a problem detecting the BNO055 ... check your connections 

    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }    
  bno.setExtCrystalUse(true);
  // delay(1000);
}

void readIMU_pos() //loop
{
  if (isReadIMU ==1) {
    isReadIMU=0;
  sensors_event_t event;
  bno.getEvent(&event);
  // Display the floating point data 
  Serial.print("X: ");
  x_imu = event.orientation.x; 
  y_imu = event.orientation.y;
  z_imu = event.orientation.z;
  // Serial.print(x_imu, 4);
  // Serial.print("\tY: ");
  // Serial.print(y_imu, 4);
  // Serial.print("\tZ: ");
  // Serial.print(z_imu, 4);
  // Serial.println("");
  }
  
}
void isReadIMUdata(){ 
  isReadIMU = 1;
  // isPrint=1;
}