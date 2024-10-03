
#include <math.h>
#include <Arduino.h>

int motorRpin1 = 8; //right motor IN1
int motorRpin2 = 9; //right motor IN2

int motorLpin1 = 10; //left motor IN3
int motorLpin2 = 11; //left motor IN4

int enR = 7; 
int enL = 12;

// Control parameters
float gamma = 1;// Control gain for linear velocity
float lambda = 1.5;  // Control gain for angular velocity (alpha)
float h = 1.3;  // Control gain for orientation error (beta)

// Time step for simulation
float dt = 0.1;  // Time step in seconds

// Robot's initial pose (x, y, theta)
float x_r = 0.0;
float y_r = 0.0;
float theta_r = 0.0;  // Initial orientation in radians

// Goal pose (x, y, theta)
float x_g = 1;
float y_g = 1;
float theta_g = 0.0;  // Goal orientation in radians

// Distance between the wheels (wheelbase)
float L = 0.182;  
// float WHEEL_RADIUS = 0.0225;

unsigned long previousMillis = 0;

int velocityToPWM(float velocity) {
    // int pwmValue = (int)((velocity / maxVelocity) * 255);
    int pwmValue = (int)((-0.0017*pow((velocity*424),2) + 0.7764*424*velocity +18.387)*20);
    pwmValue = constrain(pwmValue, 0, 255);  
    return pwmValue;
}

void Rightforward (int pwmRight) {
  analogWrite(enR, pwmRight);
  digitalWrite(motorRpin1, LOW);
  digitalWrite(motorRpin2, HIGH);
}

void Leftforward (int pwmLeft) {
  analogWrite(enL, pwmLeft);
  digitalWrite(motorLpin1, LOW);
  digitalWrite(motorLpin2, HIGH);
}

void stop() {
  digitalWrite(motorLpin1, LOW);
  digitalWrite(motorLpin2, LOW);
  digitalWrite(motorRpin1, LOW);
  digitalWrite(motorRpin2, LOW);
}


void setup() {
    // Start serial communication to print the output
    Serial.begin(9600);

    // Set motor pins as output
    pinMode(motorLpin1, OUTPUT);
    pinMode(motorLpin2, OUTPUT);
    pinMode(motorRpin1, OUTPUT);
    pinMode(motorRpin2, OUTPUT);
    previousMillis = millis();
}

void loop() {
  // Compute the distance to the goal
  float rho = sqrt(pow((x_g - x_r), 2) + pow((y_g - y_r), 2));

  float alpha = atan2(y_g - y_r, x_g - x_r) - theta_r;
  float phi = atan2(y_g - y_r, x_g - x_r) - theta_g;

  // Control laws
  float v = (gamma * cos(alpha) )* rho; // Linear velocity
  float w = lambda * alpha + ((gamma * (cos(alpha) * sin(alpha)/ alpha))* (alpha + h * phi));  // Angular velocity

  float v_left = v - (w * L / 2);  
  float v_right = v + (w * L / 2);  

  Serial.print("v_left: ");  
  Serial.print(v_left, 4);

  Serial.print("v_right: ");  
  Serial.print(v_right, 4);

  // float wr = v_right / WHEEL_RADIUS * 60 / (2 * M_PI);  // angular velocity in rpm
  // float wl = v_left / WHEEL_RADIUS * 60 / (2 * M_PI);

  // Map the wheel velocities to PWM (0 to 255)
  int pwmLeft = velocityToPWM(v_left);  // Assuming 0.2 m/s is the max velocity
  int pwmRight = velocityToPWM(v_right);
  
  // if (pwmLeft < 50 && pwmLeft != 0) {
  // pwmLeft = 50;
  // }

  // if (pwmRight < 50 && pwmRight != 0) {
  // pwmRight = 50;
  // }  

  Serial.print("pwmLeft: ");  Serial.println(pwmLeft);
  Serial.print("pwmRight: ");  Serial.println(pwmRight);

  Rightforward(pwmRight);
  // Rightbackward(pwmRight);
  Leftforward(pwmLeft);
  // Leftbackward(pwmLeft);

  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= 100) {
    theta_r = theta_r + ((v_right - v_left)/L * dt);
    x_r = x_r + (v_left + v_right)/2 * cos(theta_r) * dt;
    y_r = y_r + (v_left + v_right)/2 * sin(theta_r) * dt;
    previousMillis = currentMillis;
  
  Serial.print("x_r: ");  
  Serial.print(x_r, 4);  // 4 decimal places for precision
  Serial.print(" , y_r: ");  
  Serial.print(y_r, 4);  // 4 decimal places for precision
  Serial.print(" , theta_r: ");
  Serial.println(theta_r, 4); 
}

  
  if (rho < 0.01) {  // If the distance is less than 1 cm
      Serial.println("Goal reached!");
      stop();  // Stop the motors
      while (true);  // Stop the simulation
  }

}

