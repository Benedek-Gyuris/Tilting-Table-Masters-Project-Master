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
const int ENC_PRX = 1600; // Encoder counts per revolution for Motor 1
const int ENC_PRZ = 1600; // Encoder counts per revolution for Motor 2

// Encoder objects
Encoder myEncX(11, 3); // Encoder for Motor 1
Encoder myEncZ(13, 2); // Encoder for Motor 2

// Motor speed limits
const int MAX_PWM = 150;

// Variables for calibration
float minAngleX = 0, maxAngleX = 0, flatAngleX = 0;
float minAngleZ = 0, maxAngleZ = 0, flatAngleZ = 0;

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

  Serial.println("Starting angle calibration...");
  calibrateAngles(); // Perform calibration
}

void loop() {
  // Nothing to do in the loop for this standalone calibration
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

// Function to calibrate the angle limits for the motors
void calibrateAngles() {
  // Move Motor 1 (X) to positive limit
  Serial.println("Calibrating Motor 1 (X)...");
  setMotorSpeed(1, MAX_PWM+45); // Full speed forward
  delay(1000); // Run for 2 seconds
  long curPositionX = myEncX.read();
  maxAngleX = (2 * PI * curPositionX) / ENC_PRX; // Convert to radians
  setMotorSpeed(1, 0); // Stop motor
  delay(500); // Allow motor to settle

  // Move Motor 1 (X) to negative limit
  setMotorSpeed(1, -MAX_PWM-45); // Full speed backward
  delay(1000); // Run for 2 seconds
  curPositionX = myEncX.read();
  minAngleX = (2 * PI * curPositionX) / ENC_PRX; // Convert to radians
  setMotorSpeed(1, 0); // Stop motor
  delay(500); // Allow motor to settle

  // Calculate flat reference for Motor 1
  flatAngleX = (maxAngleX + minAngleX) / 2.0;

  // Move Motor 2 (Z) to positive limit
  Serial.println("Calibrating Motor 2 (Z)...");
  setMotorSpeed(2, MAX_PWM); // Full speed forward
  delay(1000); // Run for 2 seconds
  long curPositionZ = myEncZ.read();
  maxAngleZ = (2 * PI * curPositionZ) / ENC_PRZ; // Convert to radians
  setMotorSpeed(2, 0); // Stop motor
  delay(500); // Allow motor to settle

  // Move Motor 2 (Z) to negative limit
  setMotorSpeed(2, -MAX_PWM); // Full speed backward
  delay(1000); // Run for 2 seconds
  curPositionZ = myEncZ.read();
  minAngleZ = (2 * PI * curPositionZ) / ENC_PRZ; // Convert to radians
  setMotorSpeed(2, 0); // Stop motor
  delay(500); // Allow motor to settle

  // Calculate flat reference for Motor 2
  flatAngleZ = (maxAngleZ + minAngleZ) / 2.0;

  // Output calibration results
  Serial.println("Calibration complete:");
  Serial.print("Motor 1 (X): Min = "); Serial.print(minAngleX);
  Serial.print(", Max = "); Serial.print(maxAngleX);
  Serial.print(", Flat Reference = "); Serial.println(flatAngleX);

  Serial.print("Motor 2 (Z): Min = "); Serial.print(minAngleZ);
  Serial.print(", Max = "); Serial.print(maxAngleZ);
  Serial.print(", Flat Reference = "); Serial.println(flatAngleZ);
}
