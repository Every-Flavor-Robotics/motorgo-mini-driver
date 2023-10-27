#include <Arduino.h>

#include "motorgo_mini.h"

MotorGo::MotorGoMini* motorgo_mini;
MotorGo::MotorParameters motor_params_ch0;
MotorGo::MotorParameters motor_params_ch1;

// Function to print at a maximum frequency
void freq_println(String str, int freq)
{
  static unsigned long last_print_time = 0;
  unsigned long now = millis();

  if (now - last_print_time > 1000 / freq)
  {
    Serial.println(str);
    last_print_time = now;
  }
}

void setup()
{
  Serial.begin(115200);

  // Setup motor parameters
  motor_params_ch0.pole_pairs = 7;
  motor_params_ch0.power_supply_voltage = 5.0;
  motor_params_ch0.voltage_limit = 5.0;
  motor_params_ch0.current_limit = 300;
  motor_params_ch0.velocity_limit = 100.0;
  motor_params_ch0.calibration_voltage = 2.0;

  motor_params_ch1.pole_pairs = 7;
  motor_params_ch1.power_supply_voltage = 5.0;
  motor_params_ch1.voltage_limit = 5.0;
  motor_params_ch1.current_limit = 300;
  motor_params_ch1.velocity_limit = 100.0;
  motor_params_ch1.calibration_voltage = 2.0;

  // Instantiate motorgo mini board
  motorgo_mini = new MotorGo::MotorGoMini();

  // Setup Ch0 with FOCStudio enabled
  motorgo_mini->init_ch0(motor_params_ch0, false, false);
  motorgo_mini->init_ch1(motor_params_ch1, false, false);
}

void loop()
{
  // Run Ch0
  motorgo_mini->loop_ch0();
  motorgo_mini->loop_ch1();

  String str = "Velocity - Ch0: " + String(motorgo_mini->get_ch0_velocity()) +
               " Ch1: " + String(motorgo_mini->get_ch1_velocity());

  freq_println(str, 10);
}
