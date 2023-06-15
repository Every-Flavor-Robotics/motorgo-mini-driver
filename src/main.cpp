#include <Arduino.h>

#include "motorgo_mini.h"

#define LOOP_HZ 1000
#define LOOP_US 1000000 / LOOP_HZ

MotorGo::MotorGoMini* motorgo_mini;

void setup()
{
  Serial.begin(115200);
  // Instantiate motorgo mini board
  motorgo_mini = new MotorGo::MotorGoMini();

  // Setup Ch0 with FOCStudio enabled
  motorgo_mini->init_ch0(false, false);
  motorgo_mini->enable_ch0();
}

int i = 0;
// Constrain loop speed to 250 Hz
unsigned long last_loop_time = 0;
void loop()
{
  // Run Ch0
  motorgo_mini->loop_ch0();

  //   Print shaft velocity
  //   Serial.print("Shaft velocity: ");

  //   Spin forward
  //   Enable
  motorgo_mini->set_target_velocity_ch0(10.0);
  i++;

  // Delay necessary amount micros
  unsigned long now = micros();
  unsigned long loop_time = now - last_loop_time;

  if (loop_time < LOOP_US)
  {
    delayMicroseconds(LOOP_US - loop_time);

    // Serial.print("Loop time: ");
  }

  last_loop_time = now;
}
