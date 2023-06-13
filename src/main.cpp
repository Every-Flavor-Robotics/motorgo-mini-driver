#include <Arduino.h>

#include "motorgo_mini.h"

MotorGo::MotorGoMini* motorgo_mini;

void setup()
{
  // Instantiate motorgo mini board
  motorgo_mini = new MotorGo::MotorGoMini();

  // Setup Ch0 with FOCStudio enabled
  motorgo_mini->init_ch0(false, true);
}

void loop()
{
  // Run Ch0
  motorgo_mini->loop_ch0();
}
