
unsigned long previousMillis = 0;
const float T = 0.1; // Sampling rate

// Goal and current pose
float x = 0, y = 0, theta = 0;
float x_g = 1, y_g = 1, theta_g = 3.13;
int isReadIMU = 0;

// PID control parameters
const float kp = 0.1;
const float ki = 0.0;
const float kd = 0.05;
float error_position, sumError_position = 0, previousError_position = 0;
const float kp_theta = 0.09;
// const float ki_theta = 0;
const float kd_theta = 0.03;
// Control motion variables
float rho, alpha, v = 0.1, vr, vl;
float currentError_position, differenceError_position;

// Control state: 0 = moving to position, 1 = adjusting orientation
int controlState = 0;

float v1 ; // Left wheel velocity in m/s
float v2 ; // Right wheel velocity in m/s
int isPrint =0;
void setup() {
  Serial.begin(9600);
  // setupIMU();
  // Display the floating point data 
  
  // Set up motor and encoder pins
  pinMode(motorRpin1, OUTPUT);
  pinMode(motorRpin2, OUTPUT);
  pinMode(motorLpin1, OUTPUT);
  pinMode(motorLpin2, OUTPUT);
  
  pinMode(enLA, INPUT);
  pinMode(enRA, INPUT);

  attachInterrupt(digitalPinToInterrupt(enLA), countEnLA, RISING);
  attachInterrupt(digitalPinToInterrupt(enRA), countEnRA, RISING);
}
void loop() {
  // readIMU_pos();
    if (controlState == 0) {
    // Phase 1: Position control
    positionControl_PID();
  } else if (controlState == 1) {
    // Phase 2: Orientation control
    orientationControl_PID();
  }
}

void positionControl_PID() {
  unsigned long currentMillis = millis();
  
  // Update error_position for PID
  calculatePIDError_position();

  // Control motor speeds
  if (vr > 0) rightForward(vr);
  else rightBackward(-vr);
  
  if (vl > 0) leftForward(vl);
  else leftBackward(-vl);

  // Update RPM and pose periodically
  if (currentMillis - previousMillis >= T * 1000) {
    previousMillis = currentMillis;
    updatePose();
  }

  // Stop when goal is reached
  if (rho < 0.05) {
    stop();
    controlState =1; // Hold position at goal
    Serial.print("Reach destination");
  }
}

// Calculate PID error_position for orientation
void calculatePIDError_position() {
  float deltaX = x_g - x;
  float deltaY = y_g - y;
  
  // Calculate distance to goal (rho) and heading angle (alpha)
  rho = sqrt(deltaX * deltaX + deltaY * deltaY);
  float theta_p = atan2(deltaY, deltaX);
  
  previousError_position = currentError_position;
  currentError_position = theta_p - theta;
  
  differenceError_position = currentError_position - previousError_position;
  sumError_position += currentError_position;

  // Calculate PID control output (error_position) for velocity difference between wheels
  error_position = kp * currentError_position + ki * sumError_position + kd * differenceError_position;
  
  // Adjust wheel velocities
  vr = v + error_position;
  vl = v - error_position;
}
float normalizeAngle(float angle) {
  // Normalize the angle to the range [-π, π]
  while (angle > M_PI-0.05) {
    angle -= 2 * M_PI;
  }
  while (angle < -M_PI+0.05) {
    angle += 2 * M_PI;
  }
  return angle;
}
void orientationControl_PID() {
  // Calculate the orientation error_position
  float orientationError_position = theta_g - theta;

  // If the orientation error_position is small, stop the robot
  if (abs(orientationError_position) < 0.2) {
    stop();
    Serial.println("Goal reached with correct orientation!");
    delay(3000);
    while(1);
  } else {
    // Control motors to rotate and adjust orientation
    float correction = kp_theta * orientationError_position + kd_theta * (orientationError_position - previousError_position);
    previousError_position = orientationError_position;

    // Adjust wheel velocities for rotation
    vr = correction;
    vl = -correction;

    if (vr > 0) rightForward(vr);
    else rightBackward(-vr);
    
    if (vl > 0) leftForward(vl);
    else leftBackward(-vl);
    // Update RPM and pose periodically
    Serial.print("correction: "); Serial.print(correction);

  }
  v1 = RPMtoMPS(calculateRPM(counterLA)); // Left wheel velocity in m/s
  v2 = RPMtoMPS(calculateRPM(counterRA)); // Right wheel velocity in m/s
  
  theta += (v2 - v1) / WHEEL_DISTANCE*T;
  theta = normalizeAngle(theta);
  Serial.print(", Theta: "); Serial.print(theta);
  Serial.print(", orientationError_position: "); Serial.println(orientationError_position);
}
// Update the robot's position (x, y, theta)
void updatePose() {
  v1 = RPMtoMPS(calculateRPM(counterLA)); // Left wheel velocity in m/s
  v2 = RPMtoMPS(calculateRPM(counterRA)); // Right wheel velocity in m/s
  
  x += (v1 + v2) / 2 * cos(theta) * T;
  y += (v1 + v2) / 2 * sin(theta) * T;
  theta += (v2 - v1) / WHEEL_DISTANCE * T;
  theta =normalizeAngle(theta);
  // Print current pose for debugging
  // if (isPrint ==1) {
  //   isPrint ==0;
  Serial.print("X: "); Serial.print(x); Serial.print(", Y: "); Serial.print(y);
  Serial.print(", Theta: "); Serial.println(theta);
  // }
  // Reset encoder counters
  resetCounters();
}

// Motor control functions
void rightForward(float velocity) {
  int pwm = MPStoPWM(velocity);
  digitalWrite(motorRpin1, LOW);
  digitalWrite(motorRpin2, HIGH);
  analogWrite(enR, pwm);
}

void leftForward(float velocity) {
  int pwm = MPStoPWM(velocity);
  digitalWrite(motorLpin1, LOW);
  digitalWrite(motorLpin2, HIGH);
  analogWrite(enL, pwm);
}

void rightBackward(float velocity) {
  int pwm = MPStoPWM(velocity);
  digitalWrite(motorRpin1, HIGH);
  digitalWrite(motorRpin2, LOW);
  analogWrite(enR, pwm);
}

void leftBackward(float velocity) {
  int pwm = MPStoPWM(velocity);
  digitalWrite(motorLpin1, HIGH);
  digitalWrite(motorLpin2, LOW);
  analogWrite(enL, pwm);
}

// Stop motors
void stop() {
  digitalWrite(motorLpin1, LOW);
  digitalWrite(motorLpin2, LOW);
  digitalWrite(motorRpin1, LOW);
  digitalWrite(motorRpin2, LOW);
}

// Helper functions
float calculateRPM(int pulseCounter) {
  return (float)pulseCounter * 60 / PULSE_PER_REV / T;
}

float RPMtoMPS(float rpm) {
  return rpm / 60 * WHEEL_PERIMETER;
}

int MPStoPWM(float velocity) {
  return constrain((velocity / 0.2) * 255, 30, 255); // Mapping speed to PWM
}

void countEnLA() {
  counterLA++;
}

void countEnRA() {
  counterRA++;
}

void resetCounters() {
  counterLA = 0;
  counterRA = 0;
}