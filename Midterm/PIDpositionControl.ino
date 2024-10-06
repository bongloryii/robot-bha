unsigned long previousMillis = 0;
const float T = 0.1; // Sampling rate

// Goal and current pose
float x = 0.5, y = 0, theta = 0;
float x_g = 1, y_g = 1, theta_g = 0.758;

// PID control parameters
const float kp_pos = 0.1;
const float ki_pos = 0.0;
const float kd_pos = 0.05;
float error, sumError = 0, previousError = 0;
const float kp_theta = 0.07;
// const float ki_theta = 0;
const float kd_theta = 0.01;
// Control motion variables
float rho, v = 0.1, vr, vl;
float currentError, differenceError;

// Control state: 0 = moving to position, 1 = adjusting orientation
int controlState = 0;

float v1 ; // Left wheel velocity in m/s
float v2 ; // Right wheel velocity in m/s
  
void setup() {
  Serial.begin(9600);
  
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

void position_and_orientation_control() { //loop
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
  
  // Update error for PID
  calculatePIDError_pos();

  // Control motor speeds
  setSpeed(vr,vl);

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

// Calculate PID error for orientation
void calculatePIDError_pos() {
  float deltaX = x_g - x;
  float deltaY = y_g - y;
  
  // Calculate distance to goal (rho) and heading angle
  rho = sqrt(deltaX * deltaX + deltaY * deltaY);
  float theta_p = atan2(deltaY, deltaX);
  
  previousError = currentError;
  currentError = theta_p - theta;
  
  differenceError = currentError - previousError;
  sumError += currentError;

  // Calculate PID control output (error) for velocity difference between wheels
  error = kp * currentError + ki * sumError + kd * differenceError;
  
  // Adjust wheel velocities
  vr = v + error;
  vl = v - error;
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
  // Calculate the orientation error
  float orientationError = theta_g - theta;

  // If the orientation error is small, stop the robot
  if (abs(orientationError) < 0.05) {
    stop();
    Serial.println("Goal reached with correct orientation!");
    delay(3000);
    while(1);
  } else {
    // Control motors to rotate and adjust orientation
    float correction = kp_theta * orientationError + kd_theta * (orientationError - previousError);
    previousError = orientationError;

    // Adjust wheel velocities for rotation
    vr = correction;
    vl = -correction;

    setSpeed(vr,vl);
    // Update RPM and pose periodically
    Serial.print("correction: "); Serial.print(correction);

  }
  theta += (v2 - v1) / WHEEL_DISTANCE*T;
  theta = normalizeAngle(theta);
  Serial.print(", Theta: "); Serial.print(theta);
  Serial.print(", orientationError: "); Serial.println(orientationError);


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
  Serial.print("X: "); Serial.print(x); Serial.print(", Y: "); Serial.print(y);
  Serial.print(", Theta: "); Serial.println(theta);

  // Reset encoder counters
  resetCounters();
}
