#include <util/atomic.h>  // For the ATOMIC_BLOCK macro
#include "DualTB9051FTGMotorShield.h"  // Motor shield library
#include "Encoder.h"

// Define the encoder pin (since ENCB is not used for direction)
#define ENCA 2  // Encoder channel A

// Motor Shield object
DualTB9051FTGMotorShield md;

// Volatile variable to track encoder position
volatile long posi = 0;  // Position in counts (increment/decrement per encoder pulse)
long pos = 0;

// Encoder counts per revolution (based on your encoder)
const int COUNTS_PER_REV = 12;  // Adjust based on your Hall effect encoder specs

// Gear ratio (if your motor has a gearbox)
const int GEAR_RATIO = 25;  // 25:1 gear ratio

// Variables for motor control
long targetCounts = 0;  // Target position in encoder counts
bool motorMoving = false;  // Flag to track motor movement
bool motorDirection = true;  // True for forward, False for reverse
bool targetSet = false;  // Flag to lock the target counts

void setup() {
  // Serial communication for debugging
  Serial.begin(9600);

  // Motor shield initialization
  md.init();
  md.enableDrivers();

  // Set up the encoder pin
  pinMode(ENCA, INPUT);
  
  // Attach interrupt on rising edge of ENCA
  attachInterrupt(digitalPinToInterrupt(ENCA), readEncoder, RISING);  

  md.setM1Speed(250);  // Move forward (clockwise)

  Serial.println("System initialized.");
  Serial.println("Enter angle in radians for motor control.");


}

void loop() {
  // Read the encoder position atomically to avoid race conditions
  pos = posi;

  // Calculate the angular position in radians
  float radians = ((float)pos / (125)) * 2.0 * PI;

  // Print the current encoder position and angular position
  Serial.print("Encoder position (counts): ");
  Serial.print(pos);
  Serial.print(" | Angular position (radians): ");
  Serial.println(radians);

  // Check for incoming commands from the Serial Monitor
  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n');  // Read the command

    // Check for angle command (e.g., "a:1.57" for 1.57 radians)
    if (command.startsWith("a:")) {
      float angle = command.substring(2).toFloat();  // Extract the angle (in radians)
      if (!motorMoving) {
        rotateMotorByRadians(angle);  // Rotate the motor by the specified angle only if it's not already moving
      }
    } else if (command == "S") {
      // Stop the motor
      md.setM1Speed(0);
      motorMoving = false;
      targetSet = false;  // Reset target when motor is stopped
      Serial.println("Motor stopped.");
    }

    // Acknowledge the command
    Serial.println("Command processed.");
  }
  // Check if the motor is moving and if the target has been reached
  if (motorMoving) {
    long currentPos = posi;  // Atomically read the current position

    // Check if the motor has reached the target position
    if ((motorDirection && currentPos >= targetCounts) || (!motorDirection && currentPos <= targetCounts)) {
      md.setM1Speed(0);  // Stop the motor
      motorMoving = false;
      targetSet = false;  // Reset the target lock after movement is complete
      Serial.println("Target position reached. Motor stopped.");
    }
  }

}


// Interrupt service routine to read the encoder
void readEncoder() {
  // Since we're assuming a single channel encoder, only increment or decrement based on motor direction
  if (motorDirection) {
    posi++;  // Increment position for forward rotation
  } else {
    posi--;  // Decrement position for reverse rotation
  }
}

// Function to rotate the motor by a specified angle in radians
void rotateMotorByRadians(float angle) {
  if (!targetSet) {  // Only set the target once when function is first called
    // Calculate the target encoder counts based on the angle (in radians)
    long countsToMove = (angle / (2.0 * PI)) * (125);
    
    Serial.print("Counts to move: ");
    Serial.println(countsToMove);
  
    // Set the target position
    targetCounts = posi + (motorDirection ? countsToMove : -countsToMove);
    targetSet = true;  // Lock the target so it doesn't change during motor movement

    Serial.println("Motor moving to target position...");
    Serial.println(targetCounts);

    // Move the motor in the correct direction
    if (countsToMove > 0) {
      motorDirection = true;  // Set direction to forward
      Serial.print("Starting to move... ");
      delay(3000);

      md.setM1Speed(250);  // Move forward (clockwise)
    } else {
      motorDirection = false;  // Set direction to reverse
      md.setM1Speed(-250);  // Move reverse (counterclockwise)
    }

    motorMoving = true;  // Set the motor moving flag
    Serial.println(motorMoving);
    delay(3000);


  }
  
  
}