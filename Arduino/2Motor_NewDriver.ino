#include <Encoder.h>

// Encoders for both motors
//Encoder myEncX(5, 4);  // Encoder for Motor 1 (M1)
//Encoder myEncZ(7, 6);  // Encoder for Motor 2 (M2)

// Variables to store previous encoder counts
long int prevCountX = 0;
long int prevCountZ = 0;


// Pin definitions for Channel A (Motor 1)
int directionPinA = 12;
int pwmPinA = 4;
int brakePinA = 9;

// Pin definitions for Channel B (Motor 2)
int directionPinB = 13;
int pwmPinB = 11;
int brakePinB = 8;

// Function to control a motor
void controlMotor(char motor, int pwm, bool direction) {
  int directionPin, pwmPin, brakePin;

  // Assign the correct pins based on the motor
  if (motor == 'A') {
    directionPin = directionPinA;
    pwmPin = pwmPinA;
    brakePin = brakePinA;
  } else if (motor == 'B') {
    directionPin = directionPinB;
    pwmPin = pwmPinB;
    brakePin = brakePinB;
  } else {
    Serial.println("Invalid motor identifier. Use 'A' or 'B'.");
    return;
  }

  // Set direction
  digitalWrite(directionPin, direction ? HIGH : LOW);

  // Release brake
  digitalWrite(brakePin, LOW);

  // Set PWM speed
  analogWrite(pwmPin, pwm);
}

void stopMotor(char motor) {
  int brakePin, pwmPin;

  // Assign the correct pins based on the motor
  if (motor == 'A') {
    brakePin = brakePinA;
    pwmPin = pwmPinA;
  } else if (motor == 'B') {
    brakePin = brakePinB;
    pwmPin = pwmPinB;
  } else {
    Serial.println("Invalid motor identifier. Use 'A' or 'B'.");
    return;
  }

  // Engage brake and stop PWM
  digitalWrite(brakePin, HIGH);
  analogWrite(pwmPin, 0);
}

void setup() {
  // Define pins for both motors
  pinMode(directionPinA, OUTPUT);
  pinMode(pwmPinA, OUTPUT);
  pinMode(brakePinA, OUTPUT);
  pinMode(directionPinB, OUTPUT);
  pinMode(pwmPinB, OUTPUT);
  pinMode(brakePinB, OUTPUT);

  // Reset encoders
  //myEncX.write(0);
  //myEncZ.write(0);
  
  // Initialize previous counts
  //prevCountX = myEncX.read();
  //prevCountZ = myEncZ.read();

  // Initialize Serial for debugging
  Serial.begin(9600);
  Serial.println("Motor Test Initialized.");
}

void loop() {
  // Read current encoder counts
  //long int currentCountX = myEncX.read();
  //long int currentCountZ = myEncZ.read();
/*
  // Check for changes in encoder count for Motor 1 (M1) 
  if (currentCountX != prevCountX) {
    Serial.print("Updated Encoder M1 (counts): ");
    Serial.println(currentCountX);
    prevCountX = currentCountX;  // Update previous count
  }

  // Check for changes in encoder count for Motor 2 (M2)
  if (currentCountZ != prevCountZ) {
    Serial.print("Updated Encoder M2 (counts): ");
    Serial.println(currentCountZ);
    prevCountZ = currentCountZ;  // Update previous count
  }
*/
  // Test Channel A (Motor 1)
  Serial.println("Testing Channel A (Motor 1)...");
  controlMotor('A', 200, true); // Move forward
  Serial.println("Channel A moving forward.");
  delay(2000);

  controlMotor('A', 200, false); // Move backward
  Serial.println("Channel A moving backward.");
  delay(2000);

  stopMotor('A');
  Serial.println("Channel A stopped.");
  delay(1000);

  // Test Channel B (Motor 2)
  Serial.println("Testing Channel B (Motor 2)...");
  controlMotor('B', 200, true); // Move forward
  Serial.println("Channel B moving forward.");
  delay(2000);

  controlMotor('B', 200, false); // Move backward
  Serial.println("Channel B moving backward.");
  delay(2000);

  stopMotor('B');
  Serial.println("Channel B stopped.");
  delay(1000);
}
