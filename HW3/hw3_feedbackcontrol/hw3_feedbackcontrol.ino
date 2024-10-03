
#include <math.h>
#include <Arduino.h>

int motorRpin1 = 8; //right motor IN1
int motorRpin2 = 9; //right motor IN2

int motorLpin1 = 10; //left motor IN3
int motorLpin2 = 11; //left motor IN4

int enR = 7; 
int enL = 12;

// Control parameters
float gamma = 0.5;  // Control gain for linear velocity
float lambda = 2;  // Control gain for angular velocity (alpha)
float h = 1;  // Control gain for orientation error (beta)

// Time step for simulation
float dt = 0.2;  // Time step in seconds

// Robot's initial pose (x, y, theta)
float x_r = 0.0;
float y_r = 0.0;
float theta_r = 0.0;  // Initial orientation in radians

// Goal pose (x, y, theta)
float x_g = 1.0;
float y_g = 1.0;
float theta_g = 0.0;  // Goal orientation in radians

// Distance between the wheels (wheelbase)
float L = 0.182;  
// float WHEEL_RADIUS = 0.0225;

unsigned long previousMillis = 0;


float compute_rho(float x_r, float y_r, float x_g, float y_g) {
    return sqrt(pow((x_g - x_r), 2) + pow((y_g - y_r), 2));
}

float compute_alpha(float x_r, float y_r, float theta_r, float x_g, float y_g) {
    float alpha = atan2(y_g - y_r, x_g - x_r) - theta_r;
    // alpha = fmod((alpha + M_PI), (2 * M_PI)) - M_PI;
    return alpha;
}

// Function to compute the orientation error beta
float compute_phi(float x_r, float y_r, float theta_g, float x_g, float y_g) {
    float phi = atan2(y_g - y_r, x_g - x_r) - theta_g;
    // float phi = theta_g - theta_r;
    // phi = fmod((phi + M_PI), (2 * M_PI)) - M_PI;
    return phi;
}

int velocityToPWM(float velocity, float maxVelocity) {
    int pwmRight = (int)((velocity / maxVelocity) * 255);
    pwmRight = constrain(pwmRight, 0, 255);  
    return pwmRight;

    int pwmLeft = (int)((velocity / maxVelocity) * 255);
    pwmLeft = constrain(pwmLeft, 0, 255);  
    return pwmLeft;
}

void Rightforward (int pwmRight) {
  digitalWrite(motorRpin1, HIGH);
  digitalWrite(motorRpin2, LOW);
  analogWrite(enR, pwmRight);
}

void Rightbackward (int pwmRight) {
  digitalWrite(motorRpin1, LOW);
  digitalWrite(motorRpin2, HIGH);
  analogWrite(enR, pwmRight);
}

void Leftforward (int pwmLeft) {
  digitalWrite(motorLpin1, HIGH);
  digitalWrite(motorLpin2, LOW);
  analogWrite(enL, pwmLeft);
}

void Leftbackward (int pwmLeft) {
  digitalWrite(motorLpin1, LOW);
  digitalWrite(motorLpin2, HIGH);
  analogWrite(enL, pwmLeft);
}

void stop() {
  digitalWrite(motorLpin1, LOW);
  digitalWrite(motorLpin2, LOW);
  digitalWrite(motorRpin1, LOW);
  digitalWrite(motorRpin2, LOW);
}
// void driveMotor(int pwmLeft, int pwmRight) {
//     // Drive the left motor
//     if (pwmLeft >= 0) {
//         analogWrite(motorLpin1, pwmLeft);
//         analogWrite(motorLpin2, 0);
//     } else {
//         analogWrite(motorLpin1, 0);
//         analogWrite(motorLpin2, pwmLeft);
//     }

//     // Drive the right motor
//     if (pwmRight >= 0) {
//         analogWrite(motorRpin1, pwmRight);
//         analogWrite(motorRpin2, 0);
//     } else {
//         analogWrite(motorRpin1, 0);
//         analogWrite(motorRpin2, pwmRight);
//     }
// }

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
    float rho = compute_rho(x_r, y_r, x_g, y_g);

    float alpha = compute_alpha(x_r, y_r, theta_r, x_g, y_g);
    float phi = compute_phi(x_r, y_r, theta_g, x_g, y_g);

    // Control laws
    float v = (gamma * cos(alpha) )* rho; // Linear velocity
    float w = lambda * alpha + ((gamma * (cos(alpha) * sin(alpha)/ alpha))* (alpha + h * phi));  // Angular velocity

    float v_left = v + (w * L / 2);  
    float v_right = v - (w * L / 2);  

    // float wr = v_right / WHEEL_RADIUS * 60 / (2 * M_PI);  // angular velocity in rpm
    // float wl = v_left / WHEEL_RADIUS * 60 / (2 * M_PI);

    // Map the wheel velocities to PWM (0 to 2255)
    int pwmLeft = velocityToPWM(v_left, 0.2);  // Assuming 0.2 m/s is the max velocity
    int pwmRight = velocityToPWM(v_right, 0.2);
    
    if (pwmLeft < 50 && pwmLeft != 0) {
    pwmLeft = 50;
    }

    if (pwmRight < 50 && pwmRight != 0) {
    pwmRight = 50;
    }  

    Serial.print("pwmLeft: ");  Serial.println(pwmLeft);
    Serial.print("pwmRight: ");  Serial.println(pwmRight);
  
    Rightforward(pwmRight);
    Rightbackward(pwmRight);
    Leftforward(pwmLeft);
    Leftbackward(pwmLeft);

    unsigned long currentMillis = millis();
    if (currentMillis - previousMillis >= 200) {
      theta_r = theta_r + ((v_right - v_left)/L * dt);
      x_r = x_r + (v_left + v_right)/2 * cos(theta_r) * dt;
      y_r = y_r + (v_left + v_right)/2 * sin(theta_r) * dt;
      previousMillis = currentMillis;
  }

    
    if (rho < 0.01) {  // If the distance is less than 1 cm
        Serial.println("Goal reached!");
        stop();  // Stop the motors
        while (true);  // Stop the simulation
    }
  
}

