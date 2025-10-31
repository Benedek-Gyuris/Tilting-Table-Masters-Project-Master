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
const int ENC_PRX = 500;
const int ENC_PRZ = 500;

Encoder myEncX(11,3);
Encoder myEncZ(13,2); // second  motor

// Encoder myEncZ(11,3); // second  motor

// PID Controller Variables for X direction
  float KpX = 80;             // Proportional gain 
  float KiX = 10*0;         // Integral gain
  float KdX = .1*0;              // Derivative gain

  float errX = 0;
  float err_lastX = 0;
  float err_intX = 0;
  float err_dotX = 0;

  float ucommandX = 0;

// PID Controller Variables for Z direction
  float KpZ = 80;             // Proportional gain 
  float KiZ = 10*0;         // Integral gain
  float KdZ = .1*0;              // Derivative gain

  float errZ = 0;
  float err_lastZ = 0;
  float err_intZ = 0;
  float err_dotZ = 0;

  float ucommandZ = 0;
float curTime = 0;
float prevTime = 0;
float deltaTime = 0.0;

// Target position
  float refX = 0 ;  // Target angle in radians X direction
  float refZ = 0 ;  // Target angle in radians X direction

// Declare variables for controlling the motor
  long int curPositionX = 0;
  long int prevPositionX = 0;
  float cur_radX = 0.0;
  float prev_radX = 0;
  float targetX = 0;
  float velocityX = 0.0;

// Declare variables for controlling the motor
  long int curPositionZ = 0;
  long int prevPositionZ = 0;
  float cur_radZ = 0.0;
  float prev_radZ = 0;
  float targetZ = 0;
  float velocityZ = 0.0;


long int PWM = 0; // number out of 400
long int PWMX = 0; // number out of 400
long int PWMZ = 0; // number out of 400

// Calibration limit variables
  long int minPositionX = 0;    // Minimum position count (encoder)
  long int maxPositionX = 0;    // Maximum position count (encoder)
  bool calibratedX = false;     // Flag to check if calibration is complete
  long int minRadX = 0;
  long int maxRadX = 0;

// Calibration limit variables
  long int minPositionZ = 0;    // Minimum position count (encoder)
  long int maxPositionZ = 0;    // Maximum position count (encoder)
  bool calibratedZ = false;     // Flag to check if calibration is complete
  long int minRadZ = 0;
  long int maxRadZ = 0;

void setup() {
  // Serial communication for debugging
  Serial.begin(9600);
  
  md.init();
  Serial.println("System initialized.");

  // Calibrate motor limits for X and Z directions
  Serial.println("Calibrating X and Z direction...");

  calibrateLimits();

  Serial.println("Calibration complete.");
}


/// Calibration function to set the minimum and maximum positions
void calibrateLimits() {

  PWM = 200;
  // Move to maximum position
  md.setM1Speed(PWM);  // Move motor slowly toward maximum
  md.setM2Speed(PWM);  // Move motor slowly toward maximum

  delay(2000);         // Adjust this delay based on how long it takes to reach limit
  maxPositionX = myEncX.read();
  maxPositionZ = myEncZ.read();

  Serial.print("PWM backward: "); Serial.println(PWM);

  md.setM1Speed(0);
  md.setM2Speed(0);

  delay(1000);  // Small pause between calibration steps
  minRadX = (2 * PI * ((minPositionX)/2)) / ENC_PRX;
  minRadZ = (2 * PI * ((minPositionZ)/2)) / ENC_PRZ;


  PWM = -200;
  // Move to minimum position
  md.setM1Speed(PWM);  // Move motor slowly toward minimum
  md.setM2Speed(PWM);  // Move motor slowly toward minimum
  delay(2000);          // Adjust this delay based on how long it takes to reach limit
  minPositionX = myEncX.read();
  minPositionZ = myEncZ.read();

  Serial.print("PWM forward: "); Serial.println(PWM);


  md.setM1Speed(0);
  maxRadX = (2 * PI * ((maxPositionX)/2)) / ENC_PRX;
  maxRadZ = (2 * PI * ((maxPositionZ)/2)) / ENC_PRZ;

  calibratedX = true;
  calibratedZ = true;
  //Serial.print("Min Position: "); Serial.println(minPosition);
  //Serial.print("Max Position: "); Serial.println(maxPosition);
  curPositionX = myEncX.read();
  curPositionZ = myEncZ.read();
  refX = (2 * PI * (minPositionX+(abs(maxPositionX-minPositionX))/2)) / ENC_PRX;
  refZ = (2 * PI * (minPositionZ+(abs(maxPositionZ-minPositionZ))/2)) / ENC_PRZ;

  targetX = refX;
  targetZ = refZ;

  delay(1000);  // Small pause between calibration steps


}


void loop() {
  
  if (!calibratedX || !calibratedZ) return;
  // Check limits before PID control
  
  // Time-based control for the PID calculations
  curTime = millis();
  deltaTime = (curTime - prevTime) / 1000.0;  // **Converting deltaTime to seconds**

  // Read encoder position X direction
  curPositionX = myEncX.read();
  if (curPositionX != prevPositionX) {
    prevPositionX = curPositionX;
    //Serial.print("Current Pos : ");
    //Serial.println(curPosition);
  }

  curPositionZ = myEncZ.read();
  if (curPositionZ != prevPositionZ) {
    prevPositionZ = curPositionZ;
    //Serial.print("Current Pos : ");
    //Serial.println(curPosition);
  }

  // Calculate the current angle in radians
  cur_radX = (2 * PI * curPositionX) / ENC_PRX;
  cur_radZ = (2 * PI * curPositionZ) / ENC_PRZ;

  // **Calculate the error based on the target**
  errX = targetX - cur_radX;
  errZ = targetZ - cur_radZ;

  // Integral term (sum of errors over time)
  err_intX += errX * deltaTime;
  err_intZ += errZ * deltaTime;

  // Derivative term (rate of change of error)
  err_dotX = (errX - err_lastX) / deltaTime;
  err_dotZ = (errZ - err_lastZ) / deltaTime;

  // Full PID control command
  ucommandX = KpX * errX + KiX * err_intX + KdX * err_dotX;
  ucommandZ = KpZ * errZ + KiZ * err_intZ + KdZ * err_dotZ;

  // **Clamp the PWM output to avoid overflow and set maximum speed**
  PWMX = constrain(ucommandX, -400, 400);  // Updated for safe limits of PWM signal
  PWMZ = constrain(ucommandZ, -400, 400);  // Updated for safe limits of PWM signal

  // Send the PWM signal to control motor speed and direction
  md.setM1Speed(PWMX);
  md.setM2Speed(PWMZ);


  // Update the last error and previous time
  err_lastX = errX;
  err_lastZ = errZ;

  prevTime = curTime;

  /*
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
  */

  if (curPositionX <= minPositionX && targetX < cur_radX) {
    //Serial.println("| Reached minimum limit");
    md.setM1Speed(0);
    
  }
  if (curPositionX >= maxPositionX && targetX > cur_radX) {
    //Serial.println("Reached maximum limit");
    md.setM1Speed(0);
  
  }

  if (curPositionZ <= minPositionZ && targetZ < cur_radZ) {
    //Serial.println("| Reached minimum limit");
    md.setM1Speed(0);
    
  }

  if (curPositionZ >= maxPositionZ && targetZ > cur_radZ) {
    //Serial.println("Reached maximum limit");
    md.setM1Speed(0);
  
  }

    /*

  Serial.print("| Current PositionX (counts): "); Serial.print(curPositionX);
  Serial.print("| Current PositionX (rad): "); Serial.print(cur_radX);
  Serial.print("| Target AngleX (rad): "); Serial.print(targetX);
  Serial.print("| ErrorX: "); Serial.print(errX);


  Serial.print("| ucommandX: "); Serial.println(ucommandX);
*/
  Serial.print("| Current PositionZ (counts): "); Serial.print(curPositionZ);
  Serial.print("| Current PositionZ (rad): "); Serial.print(cur_radZ);
  Serial.print("| Target AngleZ (rad): "); Serial.print(targetZ);
  Serial.print("| ErrorZ: "); Serial.print(errZ);

  Serial.print("| ucommandZ: "); Serial.println(ucommandZ);



  /*
  Serial.print("| Current Position (rad): "); Serial.print(cur_rad);
  Serial.print("| Max pos (counts): "); Serial.print(maxPosition);
  Serial.print("| Min pos (counts)): "); Serial.print(minPosition);
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
