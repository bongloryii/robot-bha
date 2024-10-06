// #include "TimerOne.h"
// // Define motor pins and encoders
// int motorRpin1 = 10; // Right motor IN3
// int motorRpin2 = 11; // Right motor IN4

// int motorLpin1 = 8; // Left motor IN1
// int motorLpin2 = 9; // Left motor IN2

// int enR = 12;  // Right motor enable pin (PWM)
// int enL = 7; // Left motor enable pin (PWM)

// // Encoders
// int enLA = 2;  // Left motor encoder A
// int enRA = 3; // Right motor encoder A

// volatile unsigned int counterLA = 0;
// volatile unsigned int counterRA = 0;

// const float WHEEL_DISTANCE = 0.182; // Distance between two wheels
// const int PULSE_PER_REV = 940;
// const float WHEEL_PERIMETER = 0.1413; // Circumference of the wheel

// unsigned long previousMillis = 0;
// const float T = 0.1; // Sampling rate

// // Goal and current pose
// float x = 0, y = 0, theta = 0;
// float x_g = 2, y_g = 3, theta_g = 0;

// // PID control parameters
// const float kp = 0.1;
// const float ki = 0;
// const float kd = 0.05;
// float error, sumError = 0, previousError = 0;

// // Control motion variables
// float rho, v = 0.1, vr, vl;
// float currentError, differenceError;

// // line following sensor pin
// #define OUT1 45
// #define OUT2 47
// #define OUT3 49
// #define OUT4 51
// #define OUT5 53

// // line follower variables
// int lineL1;
// int lineL2;
// int line0;
// int lineR1;
// int lineR2;

// const float alpha =0.5; //line follower -- weight of outer eyes
// const float beta = 0.1; //line follower -- weight of inner eyes
// const float gamma = 0.1; //line follower -- weight of the middle eye

// void setup() 
// {
//   Timer1.initialize(500000);    //500ms     
//   Timer1.attachInterrupt(readLineFollowerSensor);  
  
//   Serial.begin(9600);
//   //Set up line follower sensor
//   pinMode(OUT1, INPUT);
//   pinMode(OUT2, INPUT);
//   pinMode(OUT3, INPUT);
//   pinMode(OUT4, INPUT);
//   pinMode(OUT5, INPUT);

//   // Set up motor and encoder pins
//   pinMode(motorRpin1, OUTPUT);
//   pinMode(motorRpin2, OUTPUT);
//   pinMode(motorLpin1, OUTPUT);
//   pinMode(motorLpin2, OUTPUT);
  
//   pinMode(enLA, INPUT);
//   pinMode(enRA, INPUT);

//   attachInterrupt(digitalPinToInterrupt(enLA), countEnLA, RISING);
//   attachInterrupt(digitalPinToInterrupt(enRA), countEnRA, RISING);
// }

// void loop() 
// {
//   if (readLineFollower ==1) {
//     readLineFollower=0;
//     getLineState();
//     Serial.print(lineL1);  Serial.print(lineL2);  Serial.print(line0);  Serial.print(lineR1);  Serial.println(lineR2);
//       }
    
// }

// void getLineState() {
//   lineL1 = digitalRead(OUT1);
//   lineL2 = digitalRead(OUT2);
//   line0 = digitalRead(OUT3);
//   lineR1 = digitalRead(OUT4);
//   lineR2 = digitalRead(OUT5);
//   // // approach 1: Combine the sensor values into a single binary number
//   // lineState = (lineL2 << 4) | (lineL1 << 3) | (line0 << 2) | (lineR1 << 1) | lineR2;
//   // switch (lineState) {
//   //   case 0b11111:
//   //     // All sensors are detecting white (off the line)
//   //     break;
//   //   case 0b00100:
//   //     // Only the middle sensor is on the line (perfectly centered)
//   //     break;
//   //   case 0b00110:
//   //     // Slightly off to the left --> turn right
//   //     break;
//   //   case 0b01100:
//   //     // Slightly off to the right --> turn left
//   //     break;
//   //   // Add more cases as needed for different scenarios
//   //   default:
//   //     // if timeout then stop, else continue evaluating lineState
//   //     break;
// }
// void PID_LineFollower(){
//   // Update error for PID
//   calculatePIDError_line();

//   // Control motor speeds
//   if (vr > 0) rightForward(vr);
//   else rightBackward(-vr);
  
//   if (vl > 0) leftForward(vl);
//   else leftBackward(-vl);
  
// }
// void calculatePIDError_line(){
//   previousError = currentError;
//   currentError = -alpha * lineL2 - beta * lineL2 + beta * lineR1 + alpha * lineR2;
//   if (line0 == 1) {
//     currentError -= gamma * line0;
//   } else {
//     currentError += (gamma) * (line0 + 1)
//   }
//   differenceError = currentError - previousError;
//   sumError += currentError;

//   // Calculate PID control output (error) for velocity difference between wheels
//   error = kp * currentError + ki * sumError + kd * differenceError;
  
//   // Adjust wheel velocities
//   vr = v + error;
//   vl = v - error;
// }

// void turnLeft(int turnWeight) {

// }

// void turnRight(int turnWeight) {
  
// }

// void readLineFollowerSensor(){ 
//   readLineFollower = 1;
// }

// // Update the robot's position (x, y, theta)
// void updatePose() {
//   float v1 = RPMtoMPS(calculateRPM(counterLA)); // Left wheel velocity in m/s
//   float v2 = RPMtoMPS(calculateRPM(counterRA)); // Right wheel velocity in m/s
  
//   x += (v1 + v2) / 2 * cos(theta) * T;
//   y += (v1 + v2) / 2 * sin(theta) * T;
//   theta += (v2 - v1) / WHEEL_DISTANCE * T;

//   // Print current pose for debugging
//   Serial.print("X: "); Serial.print(x); Serial.print(", Y: "); Serial.print(y);
//   Serial.print(", Theta: "); Serial.println(theta);

//   // Reset encoder counters
//   resetCounters();
// }

// // Motor control functions
// void rightForward(float velocity) {
//   int pwm = MPStoPWM(velocity);
//   digitalWrite(motorRpin1, LOW);
//   digitalWrite(motorRpin2, HIGH);
//   analogWrite(enR, pwm);
// }

// void leftForward(float velocity) {
//   int pwm = MPStoPWM(velocity);
//   digitalWrite(motorLpin1, LOW);
//   digitalWrite(motorLpin2, HIGH);
//   analogWrite(enL, pwm);
// }

// void rightBackward(float velocity) {
//   int pwm = MPStoPWM(velocity);
//   digitalWrite(motorRpin1, HIGH);
//   digitalWrite(motorRpin2, LOW);
//   analogWrite(enR, pwm);
// }

// void leftBackward(float velocity) {
//   int pwm = MPStoPWM(velocity);
//   digitalWrite(motorLpin1, HIGH);
//   digitalWrite(motorLpin2, LOW);
//   analogWrite(enL, pwm);
// }

// // Stop motors
// void stop() {
//   digitalWrite(motorLpin1, LOW);
//   digitalWrite(motorLpin2, LOW);
//   digitalWrite(motorRpin1, LOW);
//   digitalWrite(motorRpin2, LOW);
// }

// // Helper functions
// float calculateRPM(int pulseCounter) {
//   return (float)pulseCounter * 60 / PULSE_PER_REV / T;
// }

// float RPMtoMPS(float rpm) {
//   return rpm / 60 * WHEEL_PERIMETER;
// }

// int MPStoPWM(float velocity) {
//   return constrain((velocity / 0.2) * 255, 30, 255); // Mapping speed to PWM
// }

// void countEnLA() {
//   counterLA++;
// }

// void countEnRA() {
//   counterRA++;
// }

// void resetCounters() {
//   counterLA = 0;
//   counterRA = 0;
// }