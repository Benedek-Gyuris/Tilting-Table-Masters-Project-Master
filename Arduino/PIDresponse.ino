#include <Encoder.h>

// Motor Pins
#define EN1 5
#define ENB1 4
#define PWM1_1 6
#define PWM2_1 7
#define EN2 10
#define ENB2 9
#define PWM1_2 8
#define PWM2_2 12

// Encoder counts per revolution
const int ENC_PRX = 1600; 
const int ENC_PRZ = 1600;

// Encoder objects
Encoder myEncX(11, 3);
Encoder myEncZ(13, 2);

// PID Constants
float KpX = 275, KiX = 0, KdX = 6;
float KpZ = 275, KiZ = 0, KdZ = 10;

// State Variables
float targetX = 0.349066*2, targetZ = 0.349066;

float cur_radX = 0, cur_radZ = 0;
long curPositionX = 0, curPositionZ = 0;

// Motor speed limits
const int MAX_PWMX = 210;
const int MAX_PWMZ = 190;

// Flags
bool startUpdating = false;

void setup() {
  Serial.begin(115200);

  pinMode(EN1, OUTPUT);
  pinMode(ENB1, OUTPUT);
  pinMode(PWM1_1, OUTPUT);
  pinMode(PWM2_1, OUTPUT);
  pinMode(EN2, OUTPUT);
  pinMode(ENB2, OUTPUT);
  pinMode(PWM1_2, OUTPUT);
  pinMode(PWM2_2, OUTPUT);

  digitalWrite(EN1, HIGH);
  digitalWrite(ENB1, LOW);
  digitalWrite(EN2, HIGH);
  digitalWrite(ENB2, LOW);

  Serial.println("Ready. Waiting for 'R' to start.");
}

void loop() {
  // Check for serial command
  if (Serial.available() > 0) {
    char command = Serial.read();
    if (command == 'R') {
      startUpdating = true;
      Serial.println("Updating angles...");
    }
  }

  if (startUpdating) {
    updatePositions();
    applyPIDControl();
    sendData();
    delay(33);
  }
}

// Update encoder positions and calculate radians
void updatePositions() {
  curPositionX = myEncX.read();
  curPositionZ = myEncZ.read();

  cur_radX = (2 * PI * curPositionX) / ENC_PRX;
  cur_radZ = (2 * PI * curPositionZ) / ENC_PRZ;
}

// PID control and motor speed update
void applyPIDControl() {
  static float errX = 0, errZ = 0;
  static float err_lastX = 0, err_lastZ = 0;
  static float err_intX = 0, err_intZ = 0;

  float deltaTime = 0.033; // Approximate loop time in seconds

  // Calculate errors
  errX = targetX - cur_radX;
  errZ = targetZ - cur_radZ;

  // PID calculations for Motor 1 (X)
  err_intX += errX * deltaTime;
  float err_dotX = (errX - err_lastX) / deltaTime;
  int PWMX = constrain(KpX * errX + KiX * err_intX + KdX * err_dotX, -MAX_PWMX, MAX_PWMX);
  setMotorSpeed(1, PWMX);
  err_lastX = errX;

  // PID calculations for Motor 2 (Z)
  err_intZ += errZ * deltaTime;
  float err_dotZ = (errZ - err_lastZ) / deltaTime;
  int PWMZ = constrain(KpZ * errZ + KiZ * err_intZ + KdZ * err_dotZ, -MAX_PWMZ, MAX_PWMZ);
  setMotorSpeed(2, PWMZ);
  err_lastZ = errZ;
}

// Send data to MATLAB
void sendData() {
  Serial.print("X_Target:"); Serial.print(targetX);
  Serial.print(",X_Current:"); Serial.print(cur_radX);
  Serial.print(",Z_Target:"); Serial.print(targetZ);
  Serial.print(",Z_Current:"); Serial.println(cur_radZ);
}

// Set motor speed and direction
void setMotorSpeed(int motor, int pwm) {
  bool dir = pwm > 0;
  pwm = abs(pwm);

  if (motor == 1) {
    analogWrite(EN1, pwm);
    digitalWrite(PWM1_1, !dir);
    digitalWrite(PWM2_1, dir);
  } else if (motor == 2) {
    analogWrite(EN2, pwm);
    digitalWrite(PWM1_2, !dir);
    digitalWrite(PWM2_2, dir);
  }
}
