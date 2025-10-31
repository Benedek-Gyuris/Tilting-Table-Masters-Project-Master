#include <Encoder.h>
#include "DualTB9051FTGMotorShield.h"  // Motor shield library

// Motor Shield object
DualTB9051FTGMotorShield md;

// Encoders for both motors
Encoder myEncX(13, 2);  // Encoder for Motor 1 (M1)
Encoder myEncZ(11, 3);  // Encoder for Motor 2 (M2)

// Variables to store previous encoder counts
long int prevCountX = 0;
long int prevCountZ = 0;

void setup() {
  // Serial communication for debugging
  Serial.begin(9600);
  
  // Initialize the motor driver
  md.init();
  Serial.println("Motor driver initialized.");

  // Reset encoders
  myEncX.write(0);
  myEncZ.write(0);

  
  // Initialize previous counts
  prevCountX = myEncX.read();
  prevCountZ = myEncZ.read();
}

void loop() {
  // Read current encoder counts
  long int currentCountX = myEncX.read();
  long int currentCountZ = myEncZ.read();

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
  // Test Motor 1 (M1)
  Serial.println("Testing Motor 1 (M1)...");
  md.setM1Speed(200);     // Move M1 forward
  delay(1000);            // Run for 1 second
  Serial.print("Encoder M1 (counts) after forward: "); Serial.println(myEncX.read());
  md.setM1Speed(0);       // Stop M1
  delay(1000);            // Run for 1 second

  md.setM1Speed(-200);    // Move M1 backward
  delay(1000);            // Run for 1 second
  Serial.print("Encoder M1 (counts) after backward: "); Serial.println(myEncX.read());

  md.setM1Speed(0);       // Stop M1
  Serial.println("Motor 1 (M1) test complete.");

  
  // Test Motor 2 (M2)
  Serial.println("Testing Motor 2 (M2)...");
  md.setM2Speed(200);     // Move M2 forward
  delay(1000);            // Run for 1 second
  Serial.print("Encoder M2 (counts) after forward: "); Serial.println(myEncZ.read());
  
  md.setM2Speed(-200);    // Move M2 backward
  delay(1000);            // Run for 1 second
  Serial.print("Encoder M2 (counts) after backward: "); Serial.println(myEncZ.read());

  md.setM2Speed(0);       // Stop M2
  Serial.println("Motor 2 (M2) test complete.");
  
  

  // Small delay to avoid excessive Serial output
  delay(100);
}
