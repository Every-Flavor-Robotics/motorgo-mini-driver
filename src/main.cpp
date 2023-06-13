#include <Arduino.h>

#include "motorgo_mini.h"

MotorGo::MotorGoMini* motorgo_mini;

void setup()
{
  Serial.begin(115200);
  // Instantiate motorgo mini board
  motorgo_mini = new MotorGo::MotorGoMini();

  // Setup Ch0 with FOCStudio enabled
  motorgo_mini->init_ch0(true, false);
}

void loop()
{
  // Run Ch0
  motorgo_mini->loop_ch0();

  //   Print shaft velocity
  Serial.print("Shaft velocity: ");
  Serial.println(motorgo_mini->get_ch0_velocity());
  delay(50);
}
