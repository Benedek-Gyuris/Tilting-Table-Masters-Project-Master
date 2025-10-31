#include <Encoder.h>

// Motor 1 Pins
#define EN1 5       // Enable pin for Motor 1
#define ENB1 4      // Inverted Enable pin for Motor 1
#define PWM1_1 6    // PWM pin for Motor 1 control
#define PWM2_1 7    // PWM pin for Motor 1 direction

// Motor 2 Pins
#define EN2 10      // Enable pin for Motor 2
#define ENB2 9      // Inverted Enable pin for Motor 2
#define PWM1_2 8    // PWM pin for Motor 2 control
#define PWM2_2 12   // PWM pin for Motor 2 direction

// Encoder counts per revolution
const int COUNTS_PER_REV = 12;  // Based on your encoder specs
const int GEAR_RATIO = 25;      // Gear ratio
const int ENC_PRX = 500; // COUNTS_PER_REV * GEAR_RATIO; // Encoder counts per revolution Motor 1
const int ENC_PRZ = 500; // COUNTS_PER_REV * GEAR_RATIO; // Encoder counts per revolution Motor 2

// Encoder objects
Encoder myEncX(11, 3); // Encoder for Motor 1
Encoder myEncZ(13, 2); // Encoder for Motor 2

// PID constants and variables for Motor 1 (X)
float KpX = 80, KiX = 10, KdX = 1*0;
float errX = 0, err_lastX = 0, err_intX = 0, err_dotX = 0;
float ucommandX = 0;

// PID constants and variables for Motor 2 (Z)
float KpZ = 80, KiZ = 10, KdZ = 1*0;
float errZ = 0, err_lastZ = 0, err_intZ = 0, err_dotZ = 0;
float ucommandZ = 0;

// Timing variables
float curTime = 0, prevTime = 0, deltaTime = 0;

// Target positions
float targetX = 0, targetZ = 0;

// Target smoothing
float alpha = 0.1; // Smoothing factor for first-order filter
float theta_targetX = 0; // Smoothed reference angle for X
float theta_targetZ = 0; // Smoothed reference angle for Z

// Motor state variables
long curPositionX = 0, curPositionZ = 0;
float cur_radX = 0, cur_radZ = 0;

// Motor speed limits
const int MAX_PWM = 255;

void setup() {
  Serial.begin(9600);

  // Motor 1 setup
  pinMode(EN1, OUTPUT);
  pinMode(ENB1, OUTPUT);
  pinMode(PWM1_1, OUTPUT);
  pinMode(PWM2_1, OUTPUT);

  // Motor 2 setup
  pinMode(EN2, OUTPUT);
  pinMode(ENB2, OUTPUT);
  pinMode(PWM1_2, OUTPUT);
  pinMode(PWM2_2, OUTPUT);

  // Enable both motors
  digitalWrite(EN1, HIGH);
  digitalWrite(ENB1, LOW);
  digitalWrite(EN2, HIGH);
  digitalWrite(ENB2, LOW);

  Serial.println("Setup complete. Ready for control.");
}

void loop() {
  updateTargetFromSerial();

  // Read user hand position and map to desired angles
  float theta_desiredX = mapHandPositionToAngleX();
  float theta_desiredZ = mapHandPositionToAngleZ();
  
  // Smoothly update reference angles
  theta_targetX += alpha * (theta_desiredX - theta_targetX);
  theta_targetZ += alpha * (theta_desiredZ - theta_targetZ);

  // Time-based control for PID calculations
  curTime = millis();
  deltaTime = (curTime - prevTime) / 1000.0; // Convert deltaTime to seconds

  // Update positions from encoders
  curPositionX = myEncX.read();
  curPositionZ = myEncZ.read();

  // Convert encoder counts to radians
  cur_radX = (2 * PI * curPositionX) / ENC_PRX;
  cur_radZ = (2 * PI * curPositionZ) / ENC_PRZ;

  // Calculate errors for PID control
  errX = targetX - cur_radX;
  errZ = targetZ - cur_radZ;

  // PID terms for Motor 1
  err_intX += errX * deltaTime;
  err_dotX = (errX - err_lastX) / deltaTime;
  ucommandX = KpX * errX + KiX * err_intX + KdX * err_dotX;

  // PID terms for Motor 2
  err_intZ += errZ * deltaTime;
  err_dotZ = (errZ - err_lastZ) / deltaTime;
  ucommandZ = KpZ * errZ + KiZ * err_intZ + KdZ * err_dotZ;

  // Constrain PWM output to avoid overflow
  int PWMX = constrain(ucommandX, -MAX_PWM, MAX_PWM);
  int PWMZ = constrain(-ucommandZ, -MAX_PWM, MAX_PWM);
  if (abs(PWMX) < 10) {
    PWMX = 0; // Set PWM to 0 if within dead zone
  } else if (abs(PWMZ) < 10){
    PWMZ = 0; // Set PWM to 0 if within dead zone
  }
  // Apply PWM to motors
  setMotorSpeed(1, PWMX); // Motor 1
  setMotorSpeed(2, PWMZ); // Motor 2

  // Update last error and time
  err_lastX = errX;
  err_lastZ = errZ;
  prevTime = curTime;

  // Debugging output
  Serial.print("Motor 1 (X): ");
  Serial.print("Target: "); Serial.print(targetX);
  Serial.print(" Current: "); Serial.print(cur_radX);
  Serial.print(" PWM: "); Serial.print(PWMX);

  Serial.print("Motor 2 (Z): ");
  Serial.print("Target: "); Serial.print(targetZ);
  Serial.print(" Current: "); Serial.print(cur_radZ);
  Serial.print(" PWM: "); Serial.println(PWMZ);

  delay(10); // Control loop delay
}

// Function to control motor speed and direction
void setMotorSpeed(int motor, int pwm) {
  bool dir = pwm > 0; // Direction based on sign of PWM
  pwm = abs(pwm);     // Use absolute value for speed

  if (motor == 1) { // Motor 1
    analogWrite(EN1, pwm); // Set speed
    digitalWrite(PWM1_1, dir); // Set direction
    digitalWrite(PWM2_1, !dir);
  } else if (motor == 2) { // Motor 2
    analogWrite(EN2, pwm); // Set speed
    digitalWrite(PWM1_2, dir); // Set direction
    digitalWrite(PWM2_2, !dir);
  }
}

// Function to process serial input and update target positions
void updateTargetFromSerial() {
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n'); // Read input until newline

    // Check for targetX command (e.g., "X:1.57" for targetX in radians)
    if (input.startsWith("X:")) {
      targetX = input.substring(2).toFloat(); // Extract value and convert to float
      Serial.print("Updated targetX to: ");
      Serial.println(targetX);
    } 
    // Check for targetZ command (e.g., "Z:1.57" for targetZ in radians)
    else if (input.startsWith("Z:")) {
      targetZ = input.substring(2).toFloat(); // Extract value and convert to float
      Serial.print("Updated targetZ to: ");
      Serial.println(targetZ);
    } else {
      Serial.println("Invalid command. Use X:<value> or Z:<value>");
    }
  }
}