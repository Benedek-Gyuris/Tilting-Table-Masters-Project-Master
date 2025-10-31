#include "DualTB9051FTGMotorShield.h"

DualTB9051FTGMotorShield md;

void stopIfFault()
{
  if (md.getM1Fault())
  {
    Serial.println("M1 fault");
    while (1);
  }
}

void checkForStop()
{
  if (Serial.available() > 0)
  {
    char c = Serial.read();
    if (c == 's')
    {
      Serial.println("Stopping motor forever...");
      md.setM1Speed(0);
      md.disableDrivers();
      while (1); // Stay here forever
    }
  }
}

void moveMotor(int startSpeed, int endSpeed)
{
  int increment = (startSpeed < endSpeed) ? 1 : -1;
  for (int i = startSpeed; i != endSpeed + increment; i += increment)
  {
    checkForStop();
    md.setM1Speed(i);
    stopIfFault();
    
    if (abs(i) % 200 == 100)
    {
      Serial.print("M1 current: ");
      Serial.println(md.getM1CurrentMilliamps());
    }
    delay(2);
  }
}

void setup()
{
  Serial.begin(9600);
  Serial.println("Dual TB9051FTG Motor Shield");
  md.init();
}

void loop()
{
  md.enableDrivers();
  delay(1); // wait for drivers to be enabled

  // Move from 0 to 400 and back to 0
  moveMotor(0, 400);
  moveMotor(400, 0);

  // Move from 0 to -400 and back to 0
  moveMotor(0, -400);
  moveMotor(-400, 0);

  md.disableDrivers();
  delay(500);
}
