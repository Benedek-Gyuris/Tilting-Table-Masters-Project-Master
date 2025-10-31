#include "DualTB9051FTGMotorShield.h"

DualTB9051FTGMotorShield md;

void setup() {
  Serial.begin(9600);
  md.init();
  md.enableDrivers();
}

void loop() {
  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n'); // Read the entire command

    if (command.startsWith("R")) {
      int speed = command.substring(1).toInt();  // Extract speed
      md.setM1Speed(speed);  // Move motor to the right with given speed
    } 
    else if (command.startsWith("L")) {
      int speed = command.substring(1).toInt();  // Extract speed
      md.setM1Speed(-speed); // Move motor to the left with given speed
    }
    else if (command == "S") {
      Serial.println("Stopping motor forever...");
      md.setM1Speed(0);  // Stop the motor
      md.disableDrivers(); // Disable the motor drivers
      while (1); // Stay here forever
    }
  }
}
