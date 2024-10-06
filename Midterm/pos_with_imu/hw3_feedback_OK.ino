// //the front of the robot is the L298N, the back is the battery case
// // #include "TimerOne.h"
// // Define motor pins and encoders
// int motorRpin1 = 10; // Right motor IN3
// int motorRpin2 = 11; // Right motor IN4

// int motorLpin1 = 8; // Left motor IN1
// int motorLpin2 = 9; // Left motor IN2

// int enR = 13;  // Right motor enable pin (PWM)
// int enL = 7; // Left motor enable pin (PWM)

// // Encoders
// int enLA = 2;  // Left motor encoder A
// int enRA = 3; // Right motor encoder A

// float x_imu;
// float y_imu;
// float z_imu;

// volatile unsigned int counterLA = 0; 
// volatile unsigned int counterRA = 0; 
// volatile unsigned long totalCounterLA = 0; 
// volatile unsigned long totalCounterRA = 0; 
// int pwmValue = 150; //preset value for right wheel in circle control
// int isReadIMU = 0;

// float rpmLA = 0; 
// float rpmRA = 0; 

// const float WHEEL_DISTANCE = 0.182; //Distance between 2 driving wheels
// const int PULSE_PER_REV = 940;
// const float WHEEL_PERIMETER = 0.1413;
// const float WHEEL_RADIUS = 0.0225;

// unsigned long previousMillis = 0;

// // motion control
// // // pose
// float x = 0;
// float y = 0;
// float theta = 0;
// const float T = 0.1; //sampling rate

// int interval = T*1000; //delay between rpm calculations in ms

// // // goal
// float x_g = 0.75;
// float y_g = 2.9;
// float theta_g = 3.14;

// // control parameter
// float gamma = 3;
// float lamda = 6;
// float h = 0.2;

// //motion 
// float deltaX, deltaY, rho, phi, alpha, v, w, vr, vl, wr, wl, v1, v2;

// void setup() {
//   Serial.begin(9600);
//   setupIMU();
//   pinMode(enLA, INPUT);
//   // pinMode(enLB, INPUT);
//   pinMode(enRA, INPUT);
//   // pinMode(enRB, INPUT);
//   // Setup interrupt 
//   attachInterrupt(digitalPinToInterrupt(enLA), countEnLA, RISING);
//   attachInterrupt(digitalPinToInterrupt(enRA), countEnRA, RISING);

//   // attachInterrupt(digitalPinToInterrupt(enRA), rightEnISRA, RISING);
//   // attachInterrupt(digitalPinToInterrupt(enRB), rightEnISRB, RISING);

//   pinMode(motorRpin1, OUTPUT);
//   pinMode(motorRpin2, OUTPUT);
//   pinMode(motorLpin1, OUTPUT);
//   pinMode(motorLpin2, OUTPUT);
//   totalCounterRA = 0; 
//   totalCounterLA = 0; 
// }

// void loop() {
//   readIMU_pos();
//   positionControl();
// }

// void positionControl(){
//   unsigned long currentMillis = millis();  // Get current time
//   transformCoordinate();
//   calculateMotion();
//   //set speed
//   rightForward(vr);
//   leftForward(vl);
//   if (currentMillis - previousMillis >= interval) {  // Check if 1 second has passed
//     previousMillis = currentMillis;  // Save the last time pulse count was checked
//     //get current speed
//     rpmLA = calculateRPM(counterLA);    
//     // Serial.print("RPM_L = "); Serial.print(rpmLA);
//     rpmRA = calculateRPM(counterRA);    
//     // Serial.print("; RPM_R = "); Serial.println(rpmRA);
//     resetCounter();
//     localize();
//   }
//   if (rho<0.05) {
//     // if ((atan2(deltaX, deltaY)<(theta_g+0.1))&&(atan2(deltaX, deltaY)>(theta_g-0.1))) {
//     stop();
//     delay(3000);
//   // }
//   }
// }
// void localize() { //update pose
//   //rpm to m/s
//   v1 = RPMtoMPS(rpmLA); //left motor
//   v2 = RPMtoMPS(rpmRA); //right motor
//   x = x + (v1 + v2) / 2 * cos(theta) * T;
//   y = y + (v1 + v2) / 2 * sin(theta) * T;
//   // theta = theta + (v2 - v1) / WHEEL_DISTANCE * T;
//   theta = normalizeAngle(x_imu/360*2*3.14);
  
//   Serial.print("X: "); Serial.print(x); Serial.print(", Y: "); Serial.print(y);
//   Serial.print(", Theta: "); Serial.println(theta);
// }

// void transformCoordinate() {
//   deltaX = x_g - x;
//   deltaY = y_g - y;
//   rho = sqrt(pow(deltaX, 2) + pow(deltaY, 2));
//   phi = atan2(deltaY, deltaX) - theta_g;
//   alpha = atan2(deltaY, deltaX) - theta;
// }

// void calculateMotion() {
//   v = gamma * cos(alpha) * rho;
//   w = lamda * alpha + gamma * cos(alpha) * sin(alpha) * (alpha + h * phi) / alpha;
//   vr = v + WHEEL_DISTANCE * w / 2;
//   vl = v - WHEEL_DISTANCE * w / 2;
//   // wr = vr / WHEEL_RADIUS * 60 / (2 * M_PI);  // angular velocity in rpm
//   // wl = vl / WHEEL_RADIUS * 60 / (2 * M_PI);
// }

// void goForward(float length) {
//   int maxPulse = round(7000*length);
//   while ((counterLA <= maxPulse) & (counterRA <= maxPulse)) {
//       // Serial.print("counterLA: "); Serial.print(counterLA); Serial.print("; counterRA: "); Serial.println(counterRA);
//       digitalWrite(motorLpin1, LOW);
//       analogWrite(motorLpin2, 255);
//       digitalWrite(motorRpin1, LOW);
//       analogWrite(motorRpin2, 245);
//     }
//   stop();
// }
// void goForward() {
//       digitalWrite(motorLpin1, LOW);
//       analogWrite(motorLpin2, 255);
//       digitalWrite(motorRpin1, LOW);
//       analogWrite(motorRpin2, 245);
// }
// void stop() {
//   digitalWrite(motorLpin1, LOW);
//   digitalWrite(motorLpin2, LOW);
//   digitalWrite(motorRpin1, LOW);
//   digitalWrite(motorRpin2, LOW);
// }

// void countEnLA() { //update counter and total counter LA
//   counterLA++;
//   totalCounterLA++;
// }
// void countEnRA() {//update counter and total counter RA
//   counterRA++;
//   totalCounterRA++;
// }
// void resetCounter() {//reset counter LA and LB
//   counterRA = 0;
//   counterLA = 0;
// }
// float calculateRPM(int pulseCounter) { //pulse counter to RPM
//   float rpm = (float)pulseCounter * 60 / PULSE_PER_REV /T;
//   return rpm;
// }
// int MPStoPWM(float velocity, float maxVelocity = 0.2) { //velocity to PWM value
//     float pwm = (velocity / maxVelocity) * 255;
//     pwm = (int)constrain(pwm, 0, 255);  
//     return pwm;
// }
// float RPMtoMPS(float rpm){
//   float v = rpm / 60 * WHEEL_PERIMETER;
//   return v;
// }
// void rightForward (float velocity) { //set speed of right motor
//   int pwm = (int)MPStoPWM(velocity)*0.95;
//   digitalWrite(motorRpin2, HIGH);
//   digitalWrite(motorRpin1, LOW);
//   analogWrite(enR, pwm);
// }
// void leftForward (float velocity) {//set speed of left motor
//   int pwm = MPStoPWM(velocity);
//   digitalWrite(motorLpin2, HIGH);
//   digitalWrite(motorLpin1, LOW);
//   analogWrite(enL, pwm);
// }
// double calculateDistance(long counterL, long counterR){
//   double pulseVelocity = 0.5*(counterL+counterR);
//   double distance = pulseVelocity / (double)PULSE_PER_REV * WHEEL_PERIMETER;
//   return distance; 
// }
// float normalizeAngle(float angle) {
//   // Normalize the angle to the range [-π, π]
//   while (angle > M_PI) {
//     angle -= 2 * M_PI;
//   }
//   while (angle < -M_PI) {
//     angle += 2 * M_PI;
//   }
//   return angle;
// }