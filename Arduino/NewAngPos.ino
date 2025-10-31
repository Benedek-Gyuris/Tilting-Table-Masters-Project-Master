#include <Encoder.h>
#include "DualTB9051FTGMotorShield.h"  // Motor shield library

// Motor Shield object
DualTB9051FTGMotorShield md;

// Encoder counts per revolution (based on your encoder)
const int COUNTS_PER_REV = 12;  // Adjust based on your Hall effect encoder specs

// Gear ratio (if your motor has a gearbox)
const int GEAR_RATIO = 25;  // 25:1 gear ratio

// Setting encoder counts per revolution for the entire gearbox rotation
//const int ENC_PR = COUNTS_PER_REV*GEAR_RATIO; // NOT REALLY SURE WHY THIS DOESNT WORK
const int ENC_PR = 500;
float angle_radians = 0;

Encoder myEnc1(11, 3);

// PID Controller Variables
float Kp = 80;             // Proportional gain 
float Ki = 10*0;         // Integral gain
float Kd = .1*0;              // Derivative gain

float err = 0;
float err_last = 0;
float err_int = 0;
float err_dot = 0;

float curTime = 0;
float prevTime = 0;
float deltaTime = 0.0;
float ucommand = 0;

// Target position
float ref = 0 ;  // Target angle in radians

// Declare variables for controlling the motor
long int PWM = 0;
long int curPosition = 0;
long int prevPosition = 0;
float cur_rad = 0.0;
float prev_rad = 0;
float target = 0;
float velocity = 0.0;

// Calibration limit variables
long int minPosition = 0;    // Minimum position count (encoder)
long int maxPosition = 0;    // Maximum position count (encoder)
bool calibrated = false;     // Flag to check if calibration is complete
long int minRad = 0;
long int maxRad = 0;

void setup() {
  // Serial communication for debugging
  Serial.begin(9600);
  
  md.init();
  md.enableM1Driver();
  Serial.println("System initialized.");

   // **Calibrate motor limits**
  calibrateLimits();
  Serial.println("Calibration complete.");
}

// Calibration function to set the minimum and maximum positions
void calibrateLimits() {

  PWM = 200;
  // Move to maximum position
  md.setM1Speed(PWM);  // Move motor slowly toward maximum
  delay(2000);         // Adjust this delay based on how long it takes to reach limit
  maxPosition = myEnc1.read();
  Serial.print("PWM backward: "); Serial.println(PWM);

  md.setM1Speed(0);
  delay(1000);  // Small pause between calibration steps
  minRad = (2 * PI * ((minPosition)/2)) / ENC_PR;

 PWM = -200;
  // Move to minimum position
  md.setM1Speed(PWM);  // Move motor slowly toward minimum
  delay(2000);          // Adjust this delay based on how long it takes to reach limit
  minPosition = myEnc1.read();
  Serial.print("PWM forward: "); Serial.println(PWM);


  md.setM1Speed(0);
  maxRad = (2 * PI * ((maxPosition)/2)) / ENC_PR;
  calibrated = true;
  //Serial.print("Min Position: "); Serial.println(minPosition);
  //Serial.print("Max Position: "); Serial.println(maxPosition);
  curPosition = myEnc1.read();
  ref = (2 * PI * (minPosition+(abs(maxPosition-minPosition))/2)) / ENC_PR;
  target = ref;
  delay(1000);  // Small pause between calibration steps


}
void loop() {

  if (!calibrated) return;
  // Check limits before PID control
  
  // Time-based control for the PID calculations
  curTime = millis();
  deltaTime = (curTime - prevTime) / 1000.0;  // **Converting deltaTime to seconds**

  // Read encoder position
  curPosition = myEnc1.read();
  if (curPosition != prevPosition) {
    prevPosition = curPosition;
    //Serial.print("Current Pos : ");
    //Serial.println(curPosition);
  }

  // Calculate the current angle in radians
  cur_rad = (2 * PI * curPosition) / ENC_PR;


  // **Calculate the error based on the target**
  err = target - cur_rad;

  // Integral term (sum of errors over time)
  err_int += err * deltaTime;

  // Derivative term (rate of change of error)
  err_dot = (err - err_last) / deltaTime;

  // Full PID control command
  ucommand = Kp * err + Ki * err_int + Kd * err_dot;

  // **Clamp the PWM output to avoid overflow and set maximum speed**
  PWM = constrain(ucommand, -400, 400);  // Updated for safe limits of PWM signal

  // Send the PWM signal to control motor speed and direction
  md.setM1Speed(PWM);

  // Update the last error and previous time
  err_last = err;
  prevTime = curTime;

  // **Serial communication for input and control**
  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n');  // Read the command

    // Check for angle command (e.g., "a:1.57" for 1.57 radians)
    if (command.startsWith("a:")) {
      float angle = command.substring(2).toFloat();  // Extract the angle (in radians)
      ref = angle;
      target = ref + cur_rad;
      Serial.println("Target angle updated.");
    } else if (command == "S") {
      // Stop the motor
      md.setM1Speed(0);
      Serial.println("Motor stopped.");
    }
  }

  if (curPosition <= minPosition && target < cur_rad) {
    //Serial.println("| Reached minimum limit");
    md.setM1Speed(0);
    
  }
  if (curPosition >= maxPosition && target > cur_rad) {
    //Serial.println("Reached maximum limit");
    md.setM1Speed(0);
  
  }
  Serial.print("| Error: "); Serial.print(err);
  Serial.print("| Target Angle (rad): "); Serial.print(target);
  Serial.print("| Current Position (rad): "); Serial.print(cur_rad);
  Serial.print("| Max pos (counts): "); Serial.print(maxPosition);
  Serial.print("| Min pos (counts)): "); Serial.print(minPosition);
  Serial.print("| Current Position (counts): "); Serial.print(curPosition);

  Serial.print("| ucommand: "); Serial.println(ucommand);
  


  /*
  
  Serial.print("| Start pos (counts): "); Serial.print(minPosition + (maxPosition - minPosition)/2);
  Serial.print("| Current Position (counts): "); Serial.println(curPosition);
  /*Serial.print("Error: "); Serial.print(err);
    Serial.print("Target Angle (rad): "); Serial.print(target);
    Serial.print("Current Position (rad): "); Serial.print(cur_rad);
    
    
    Serial.print("ucommand: "); Serial.println(ucommand);*/
  /*Serial.print(err); // PLOTTING STUFF
    Serial.print(", ");
    Serial.print(target);
    Serial.print(", ");
    Serial.print(cur_rad);
    /*Serial.print(", ");
    Serial.print(maxPosition);
    Serial.print(", ");
    Serial.print(minPosition);
    Serial.print(", ");
    Serial.print((maxPosition - minPosition) / 2);
    Serial.print(", ");
    Serial.print(curPosition);
    Serial.print(", ");
    Serial.println(ucommand);*/
}
