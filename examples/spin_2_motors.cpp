#include <Arduino.h>

#include "motorgo_mini.h"

MotorGo::MotorGoMini* motorgo_mini;
MotorGo::MotorParameters motor_params_ch0;
MotorGo::MotorParameters motor_params_ch1;

MotorGo::PIDParameters velocity_pid_params_ch0;
MotorGo::PIDParameters velocity_pid_params_ch1;

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
  bool calibrate = false;
  bool enable_foc_studio = false;
  motorgo_mini->init_ch0(motor_params_ch0, calibrate, enable_foc_studio);
  motorgo_mini->init_ch1(motor_params_ch1, calibrate, enable_foc_studio);

  // Set velocity controller parameters
  // Setup PID parameters
  velocity_pid_params_ch0.p = 4.0;
  velocity_pid_params_ch0.i = 0.5;
  velocity_pid_params_ch0.d = 0.0;
  velocity_pid_params_ch0.output_ramp = 10000.0;
  velocity_pid_params_ch0.lpf_time_constant = 0.1;

  velocity_pid_params_ch1.p = 4.0;
  velocity_pid_params_ch1.i = 0.5;
  velocity_pid_params_ch1.d = 0.0;
  velocity_pid_params_ch1.output_ramp = 10000.0;
  velocity_pid_params_ch1.lpf_time_constant = 0.1;

  motorgo_mini->set_velocity_controller_ch0(velocity_pid_params_ch0);
  motorgo_mini->set_velocity_controller_ch1(velocity_pid_params_ch1);

  //   Set closed-loop velocity mode
  motorgo_mini->set_control_mode_ch0(MotorGo::ControlMode::Velocity);
  motorgo_mini->set_control_mode_ch1(MotorGo::ControlMode::Velocity);

  motorgo_mini->enable_ch0();
  motorgo_mini->enable_ch1();
}

void loop()
{
  // Run Ch0
  motorgo_mini->loop_ch0();
  motorgo_mini->loop_ch1();

  motorgo_mini->set_target_velocity_ch0(10.0);
  motorgo_mini->set_target_velocity_ch1(10.0);

  String str = "Velocity - Ch0: " + String(motorgo_mini->get_ch0_velocity()) +
               " Ch1: " + String(motorgo_mini->get_ch1_velocity());

  freq_println(str, 10);
}
