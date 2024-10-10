float x_imu;
// float y_imu;
// float z_imu;

volatile unsigned long totalCounterLA = 0; 
volatile unsigned long totalCounterRA = 0; 
int pwmValue = 150; //preset value for right wheel in circle control
int isReadIMU = 0;

// unsigned long previousMillis = 0;

// motion control
// // pose
float x = 0;
float y = 0;
float theta = 0;
// const float T = 0.1; //sampling rate

int interval = T*1000; //delay between rpm calculations in ms

// // goal
float x_g = 0.5;
float y_g = 3;
float theta_g = 3.12;

// float x_g = 0;
// float y_g = 0;
// float theta_g = 0;
// control parameter
float gamma = 0.12;
float lamda = 0.4;
float h = 0.03;

//motion 
float rho, phi, alpha, w, wr, wl, v1, v2;
void positionControl(){
  unsigned long currentMillis = millis();  // Get current time
  transformCoordinate();
  // calculatePIDError_position();

  calculateMotion();
  setSpeed(vr,vl);
  if (currentMillis - previousMillis >= interval) {  // Check if 0.1 second has passed
    previousMillis = currentMillis;  // Save the last time pulse count was checked
    //get current speed
    rpmLA = calculateRPM(counterLA);    
    // Serial.print("RPM_L = "); Serial.print(rpmLA);
    rpmRA = calculateRPM(counterRA);    
    // Serial.print("; RPM_R = "); Serial.println(rpmRA);
    resetCounters();
    localize();
  }
  if (rho<0.2) {
    // if ((atan2(deltaX, deltaY)<(theta_g+0.1))&&(atan2(deltaX, deltaY)>(theta_g-0.1))) {
    stop();
    delay(3000);
    reachedDestination =1;
    // while(1);
    
  // }
  }
}
void localize() { //update pose
  //rpm to m/s
  v1 = RPMtoMPS(rpmLA); //left motor
  v2 = RPMtoMPS(rpmRA); //right motor
  x = x + (v1 + v2) / 2 * cos(theta) * T;
  y = y + (v1 + v2) / 2 * sin(theta) * T;
  // theta = theta + (v2 - v1) / WHEEL_DISTANCE * T;
  // readIMU_noTimer();
  theta = normalizeAngle(-x_imu/360*2*3.14);
  
  Serial.print("X: "); Serial.print(x); Serial.print(", Y: "); Serial.print(y);
  Serial.print(", Theta: "); Serial.println(theta);
}

void transformCoordinate() {
  float deltaX = x_g - x;
  float deltaY = y_g - y;
  rho = sqrt(pow(deltaX, 2) + pow(deltaY, 2));
  phi = normalizeAngle(atan2(deltaY, deltaX) - theta_g);
  alpha = normalizeAngle(atan2(deltaY, deltaX) - theta);
}

void calculateMotion() {
  float v = gamma * cos(alpha) * rho ;
  w = lamda * alpha + gamma * cos(alpha) * sin(alpha) * (alpha + h * phi) / alpha;
  vr = v + WHEEL_DISTANCE * w / 2;
  vl = v - WHEEL_DISTANCE * w / 2;
  Serial.print("vl = ");Serial.print(vl);Serial.print("vr= ");Serial.println(vr);
  // wr = vr / WHEEL_RADIUS * 60 / (2 * M_PI);  // angular velocity in rpm
  // wl = vl / WHEEL_RADIUS * 60 / (2 * M_PI);
}


double calculateDistance(long counterL, long counterR){
  double pulseVelocity = 0.5*(counterL+counterR);
  double distance = pulseVelocity / (double)PULSE_PER_REV * WHEEL_PERIMETER;
  return distance; 
}
float previousError_position;

const float kp = 0.1;
const float ki = 0.0;
const float kd = 0.05;
float currentError_position, differenceError_position, sumError_position;
void calculatePIDError_position() {
  float deltaX = x_g - x;
  float deltaY = y_g - y;
  
  // Calculate distance to goal (rho) and heading angle (alpha)
  // rho = sqrt(deltaX * deltaX + deltaY * deltaY);
  // faster = 0.01 * rho+0.05*(rho-previousRho);
  // previousRho = rho;
  float theta_p = atan2(deltaY, deltaX);
  // Serial.print("Theta_p: "); Serial.println(theta_p);
  previousError_position = currentError_position;
  currentError_position = phi;
  
  differenceError_position = currentError_position - previousError_position;
  // Serial.print("differenceError: ");Serial.println(differenceError);
  sumError_position += currentError_position;

  // Calculate PID control output (error) for velocity difference between wheels
  phi = kp * currentError_position + ki * sumError_position + kd * differenceError_position;
  
  // // Adjust wheel velocities
  // vr = v+faster + error_position;
  // vl = v+faster - error_position;
}