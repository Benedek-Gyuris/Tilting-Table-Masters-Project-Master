#include <Encoder.h>

// Motor 1 Pins
#define EN1 5       // Enable pin for Motor 1
#define ENB1 4      // Inverted Enable pin for Motor 1
#define PWM1_1 6    // PWM pin for Motor 1 direction
#define PWM2_1 7    // PWM pin for Motor 1 direction

// Encoder counts per revolution
const int COUNTS_PER_REV = 12;  // Based on your encoder specs
const int GEAR_RATIO = 25;      // Gear ratio
const int ENC_PRX = 1600;        // Encoder counts per revolution Motor 1

// Encoder object
Encoder myEncX(11, 3);          // Encoder for Motor 1

// Timing variables
unsigned long curTime = 0, prevTime = 0;
float deltaTime = 0;

// Motor state variables
long curPositionX = 0;
float cur_radX = 0, prev_radX = 0;

// Velocity variable
float velocityX = 0;            // Angular velocity (radians/second)

// Motor control state
bool motorRunning = false;
unsigned long motorStartTime = 0;

// Function to control motor speed and direction
void setMotorSpeed(int motor, int pwm) {
  bool dir = pwm > 0;  // Direction based on sign of PWM
  pwm = abs(pwm);      // Use absolute value for speed

  if (motor == 1) { // Motor 1
    analogWrite(EN1, pwm);      // Set speed
    digitalWrite(PWM1_1, dir); // Set direction
    digitalWrite(PWM2_1, !dir);
  }
}

void setup() {
  Serial.begin(115200);

  // Motor 1 setup
  pinMode(EN1, OUTPUT);
  pinMode(ENB1, OUTPUT);
  pinMode(PWM1_1, OUTPUT);
  pinMode(PWM2_1, OUTPUT);

  // Enable motor 1
  digitalWrite(EN1, HIGH);
  digitalWrite(ENB1, LOW);

  Serial.println("start velocity");      // Velocity

}

void loop() {
  // Check for serial input
   if (Serial.available() > 0) {
        char command = Serial.read(); // Read the received character

        // Display the received character on the Serial Monitor
        Serial.print("Received: ");
        Serial.println(command);

        // Perform action based on the received character
        if (command == 'R') {
        motorRunning = true;
        motorStartTime = millis();
        setMotorSpeed(1, -150); // Start motor at 50% PWM
        }
    }
  if (motorRunning) {
    // Check if motor should stop after 2 seconds
    if (millis() - motorStartTime > 4000) {
      setMotorSpeed(1, 0); // Turn off the motor
      motorRunning = false;
    }

    // Time-based control
    curTime = millis();
    deltaTime = (curTime - prevTime) / 1000.0; // Convert deltaTime to seconds

    // Update encoder position
    curPositionX = myEncX.read();

    // Convert encoder counts to radians
    cur_radX = (2 * PI * curPositionX) / ENC_PRX;

    // Calculate velocity
    velocityX = (cur_radX - prev_radX) / deltaTime;

    // Update previous position and time
    prev_radX = cur_radX;
    prevTime = curTime;

    // Output timestamp and velocity
    Serial.print(curTime / 1000.0); // Timestamp in seconds
    Serial.print(",");
    Serial.println(velocityX);      // Velocity
  }

  delay(50); // Optional delay for control loop stability
}
