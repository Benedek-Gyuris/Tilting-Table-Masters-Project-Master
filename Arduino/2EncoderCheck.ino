#include <Encoder.h>

// Encoder pin definitions
Encoder encoder2(13, 2);  // Encoder for Motor 1 
Encoder encoder1(11, 3);  // Encoder for Motor 2 COUNTS is 1500
 
// Variables to store encoder positions
long position1 = 0;
long position2 = 0;

void setup() {
  Serial.begin(9600);  // Set higher baud rate for smooth communication
  Serial.println("Encoder Live Plot Initialized");
}

void loop() {
  // Read current positions
  position1 = encoder1.read();
  position2 = encoder2.read();

  // Send the encoder positions as "position1,position2"
  Serial.print(position1);
  Serial.print(",");
  Serial.println(position2);

  // Small delay to limit data transmission rate
  delay(100);  // Adjust if needed for smoother performance
}
