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
const int ENC_PRX = 1600; // COUNTS_PER_REV * GEAR_RATIO; // Encoder counts per revolution Motor 1
const int ENC_PRZ = 1600; // COUNTS_PER_REV * GEAR_RATIO; // Encoder counts per revolution Motor 2

// Encoder objects
Encoder myEncX(11, 3); // Encoder for Motor 1
Encoder myEncZ(13, 2); // Encoder for Motor 2

// PID constants and variables for Motor 1 (X)
float KpX = 275, KiX = 10*0, KdX = 6;
float errX = 0, err_lastX = 0, err_intX = 0, err_dotX = 0;
float ucommandX = 0;

// PID constants and variables for Motor 2 (Z)
float KpZ = 275, KiZ = 10*0, KdZ = 6;
float errZ = 0, err_lastZ = 0, err_intZ = 0, err_dotZ = 0;
float ucommandZ = 0;

// Timing variables
float curTime = 0, prevTime = 0, deltaTime = 0;

// Target positions
float targetX = 0, targetZ = 0;
float maxAngleX = 0, minAngleX = 0;
float maxAngleZ = 0, minAngleZ = 0;

// Target smoothing
float alpha = 0.1; // Smoothing factor for first-order filter
float smooth_targetX = 0; // Smoothed reference angle for X
float smooth_targetZ = 0; // Smoothed reference angle for Z

// Motor state variables
long curPositionX = 0, curPositionZ = 0;
float cur_radX = 0, cur_radZ = 0;

// Motor speed limits
const int MAX_PWMX = 210;
const int MAX_PWMZ = 190;

void setup() {
  Serial.begin(115200);

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
  delay(3000); // Find border in Python

  calibrateAngles();
  Serial.println("Setup complete. Ready for control.");

}

void loop() {
  updateTargetFromSerial();

  // Smoothly update reference angles
  smooth_targetX += alpha * (targetX - smooth_targetX);
  smooth_targetZ += alpha * (targetZ - smooth_targetZ);

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
  errX = smooth_targetX - cur_radX;
  errZ = smooth_targetZ - cur_radZ;

  // PID terms for Motor 1
  err_intX += errX * deltaTime;
  err_dotX = (errX - err_lastX) / deltaTime;
  ucommandX = KpX * errX + KiX * err_intX + KdX * err_dotX;
  //ucommandX = -ucommandX; // clear that we switched signs because of wiring. 

  // PID terms for Motor 2
  err_intZ += errZ * deltaTime;
  err_dotZ = (errZ - err_lastZ) / deltaTime;
  ucommandZ = KpZ * errZ + KiZ * err_intZ + KdZ * err_dotZ;
  ucommandZ = -ucommandZ;
  // Constrain PWM output to avoid overflow
  int PWMX = constrain(ucommandX, -MAX_PWMX, MAX_PWMX);
  int PWMZ = constrain(-ucommandZ, -MAX_PWMZ, MAX_PWMZ);
  if (abs(PWMX) < 10) {
    PWMX = 0; // Set PWM to 0 if within dead zone
  } else if (abs(PWMZ) < 10){
    PWMZ = 0; // Set PWM to 0 if within dead zone
  }

  // Apply dead zone logic
  if (abs(PWMX) < 30) {
    PWMX = 0; // Set PWM to 0 if within dead zone
  }
  if (abs(PWMZ) < 30) {
    PWMZ = 0; // Set PWM to 0 if within dead zone
  }

  // Apply PWM to motors
  setMotorSpeed(1, PWMX); // Motor 1
  setMotorSpeed(2, PWMZ); // Motor 2

  // Update last error and time
  err_lastX = errX;
  err_lastZ = errZ;
  prevTime = curTime;
  
  // // Debugging output
  // Serial.print("Motor 1 (X) ");
  // Serial.print("Target: "); Serial.print(targetX);
  // Serial.print(" Current: "); Serial.print(cur_radX);
  // Serial.print(" PWM: "); Serial.print(PWMX);
  

  // Serial.print("Motor 2 (Z) ");
  // Serial.print("Target: "); Serial.print(targetZ);
  // Serial.print(" Current: "); Serial.print(cur_radZ);
  // Serial.print(" PWM: "); Serial.println(PWMZ);
 
  
  delay(33); // Control loop delay
}

// Function to control motor speed and direction
void setMotorSpeed(int motor, int pwm) {
  bool dir = pwm > 0; // Direction based on sign of PWM
  pwm = abs(pwm);     // Use absolute value for speed

  if (motor == 1) { // Motor 1
    analogWrite(EN1, pwm); // Set speed
    digitalWrite(PWM1_1, !dir); // Set direction
    digitalWrite(PWM2_1, dir);
  } else if (motor == 2) { // Motor 2
    analogWrite(EN2, pwm); // Set speed
    digitalWrite(PWM1_2, !dir); // Set direction
    digitalWrite(PWM2_2, dir);
  }
}

// Helper function to constrain a value within limits
float constrainValue(float value, float minLimit, float maxLimit) {
  if (value < minLimit) {
    return minLimit+.2;
  } else if (value > maxLimit) {
    return maxLimit-.2; // keep them away from very limits
  }
  return value;
}

// Function to process serial input and update target positions
void updateTargetFromSerial() {
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n'); // Read input until newline

    // Example input format: "X:00.00Z00.00"
    if (input.startsWith("X:") && input.indexOf("Z") != -1) {
      int zIndex = input.indexOf("Z"); // Find the index of 'Z'

      // Extract X value
      String xValue = input.substring(2, zIndex); // Extract value after "X:" up to "Z"
      float parsedX = xValue.toFloat(); // Convert to float

      // Extract Z value
      String zValue = input.substring(zIndex + 1); // Extract value after "Z"
      float parsedZ = zValue.toFloat(); // Convert to float

      // Adjust targets relative to flat position
      targetX = constrainValue(parsedX + targetX, minAngleX, maxAngleX); // Add offset from flat
      targetZ = constrainValue(parsedZ + targetZ, minAngleZ, maxAngleZ); // Add offset from flat

     
      // // Print updated constrained values
      // Serial.print("Constrained targetX: ");
      // Serial.println(targetX);
      // Serial.print("Constrained targetZ: ");
      // Serial.println(targetZ);

    } else {
      Serial.println("Invalid command. Use format: X:00.00Z00.00");
    }
  }
}

// Function to calibrate the angle limits for the motors
void calibrateAngles() {
  // Move Motor 1 (X) to positive limit
  Serial.println("Calibrating Motor 1 (X)...");
  setMotorSpeed(1, MAX_PWMX); // Full speed forward
  delay(1000); // Run for 2 seconds
  long curPositionX = myEncX.read();
  maxAngleX = (2 * PI * curPositionX) / ENC_PRX; // Convert to radians
  setMotorSpeed(1, 0); // Stop motor
  delay(500); // Allow motor to settle

  // Move Motor 1 (X) to negative limit
  setMotorSpeed(1, -MAX_PWMX); // Full speed backward
  delay(1000); // Run for 2 seconds
  curPositionX = myEncX.read();
  minAngleX = (2 * PI * curPositionX) / ENC_PRX; // Convert to radians
  setMotorSpeed(1, 0); // Stop motor
  delay(500); // Allow motor to settle

  // Calculate flat reference for Motor 1
  targetX = (maxAngleX + minAngleX) / 2.0;

  // Move Motor 2 (Z) to positive limit
  Serial.println("Calibrating Motor 2 (Z)...");
  setMotorSpeed(2, MAX_PWMZ); // Full speed forward
  delay(1000); // Run for 2 seconds
  long curPositionZ = myEncZ.read();
  maxAngleZ = (2 * PI * curPositionZ) / ENC_PRZ; // Convert to radians
  setMotorSpeed(2, 0); // Stop motor
  delay(500); // Allow motor to settle

  // Move Motor 2 (Z) to negative limit
  setMotorSpeed(2, -MAX_PWMZ); // Full speed backward
  delay(1000); // Run for 2 seconds
  curPositionZ = myEncZ.read();
  minAngleZ = (2 * PI * curPositionZ) / ENC_PRZ; // Convert to radians
  setMotorSpeed(2, 0); // Stop motor
  delay(500); // Allow motor to settle

  // Calculate flat reference for Motor 2
  targetZ = (maxAngleZ + minAngleZ) / 2.0;

  // Output calibration results
  Serial.println("Calibration complete:");
  Serial.print("Motor 1 (X): Min = "); Serial.print(minAngleX);
  Serial.print(", Max = "); Serial.print(maxAngleX);
  Serial.print(", Flat Reference = "); Serial.println(targetX);

  Serial.print("Motor 2 (Z): Min = "); Serial.print(minAngleZ);
  Serial.print(", Max = "); Serial.print(maxAngleZ);
  Serial.print(", Flat Reference = "); Serial.println(targetZ);
}

