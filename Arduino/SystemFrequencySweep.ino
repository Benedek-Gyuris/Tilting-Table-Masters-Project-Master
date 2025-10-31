// Motor 1 Pins
#define EN1 5       // Enable pin for Motor 1 (duty cycle)
#define ENB1 4      // Inverted Enable pin for Motor 1
#define PWM1_1 6    // Direction pin 1 for Motor 1
#define PWM2_1 7    // Direction pin for Motor 1

// Motor 2 Pins
#define EN2 10      // Enable pin for Motor 2 (duty cycle)
#define ENB2 9      // Inverted Enable pin for Motor 2
#define PWM1_2 8    // Direction pin 1 for Motor 2
#define PWM2_2 12   // Direction pin 2 for Motor 2

#include <Encoder.h>

// Encoder pin definitions
Encoder encoder2(13, 2);  // Encoder for Motor 1
Encoder encoder1(11, 3);  // Encoder for Motor 2

// Variables to store encoder positions
long position1 = 0;
long position2 = 0;

// Frequency Sweep Parameters
float R = 175.0;          // Amplitude for PWM (0-255)
float B = 2 * PI * 6;     // Maximum angular frequency (6 Hz in radians/sec)
float t = 0;              // Time in seconds
float dt = 0.01;          // Time step in seconds (10 ms)
float amplitude = 150.0;  // PWM amplitude

void setup() {
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
  digitalWrite(EN1, HIGH);  // Enable Motor 1
  digitalWrite(ENB1, LOW);  // Disable inverted enable for Motor 1
  digitalWrite(EN2, HIGH);  // Enable Motor 2
  digitalWrite(ENB2, LOW);  // Disable inverted enable for Motor 2

  Serial.begin(115200);     // Serial communication for data transfer
}

void loop() {
  // Calculate angular frequency based on time
  float w = B * abs(sin(0.1 * t)); // Angular frequency

  // Calculate sinusoidal position and velocity
  float x_pos = R * sin(w * t);        // Position trajectory
  float x_velocity = R * w * cos(w * t); // Velocity trajectory

  // Determine motor direction based on position sign
  bool direction = x_pos >= 0;

  // Apply PWM and direction to Motor 1
  if (direction) {
    digitalWrite(PWM1_1, HIGH);
    digitalWrite(PWM2_1, LOW);
  } else {
    digitalWrite(PWM1_1, LOW);
    digitalWrite(PWM2_1, HIGH);
  }
  analogWrite(EN1, (int)abs(x_pos)); // Adjust speed using PWM amplitude

  // Apply PWM and direction to Motor 2
  if (direction) {
    digitalWrite(PWM1_2, HIGH);
    digitalWrite(PWM2_2, LOW);
  } else {
    digitalWrite(PWM1_2, LOW);
    digitalWrite(PWM2_2, HIGH);
  }
  analogWrite(EN2, (int)abs(x_pos)); // Adjust speed using PWM amplitude

  // Read encoder positions
  position1 = encoder1.read();
  position2 = encoder2.read();
  Serial.print("Data: ");
  Serial.print(position1);
  Serial.print(",");
  Serial.print(position2);
  Serial.print(",");
  Serial.print(t, 3);
  Serial.print(",");
  Serial.println(w / (2 * PI), 3);

  // Increment time
  t += dt;

  // Small delay to match time step
  delay((int)(dt * 1000));
}
