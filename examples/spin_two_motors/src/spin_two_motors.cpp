#include <Arduino.h>

#include "motorgo_mini.h"

MotorGo::MotorGoMini motorgo_mini;
MotorGo::MotorChannel& motor_left = motorgo_mini.ch0;
MotorGo::MotorChannel& motor_right = motorgo_mini.ch1;

MotorGo::MotorParameters motor_params_left;
MotorGo::MotorParameters motor_params_right;

MotorGo::PIDParameters velocity_pid_params_left;
MotorGo::PIDParameters velocity_pid_params_right;

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

  delay(3000);

  // Setup motor parameters
  motor_params_left.pole_pairs = 7;
  motor_params_left.power_supply_voltage = 5.0;
  motor_params_left.voltage_limit = 5.0;
  motor_params_left.current_limit = 300;
  motor_params_left.velocity_limit = 100.0;
  motor_params_left.calibration_voltage = 2.0;
  motor_params_left.reversed = false;

  motor_params_right.pole_pairs = 7;
  motor_params_right.power_supply_voltage = 5.0;
  motor_params_right.voltage_limit = 5.0;
  motor_params_right.current_limit = 300;
  motor_params_right.velocity_limit = 100.0;
  motor_params_right.calibration_voltage = 2.0;
  motor_params_right.reversed = true;

  // Setup Ch0
  bool calibrate = false;
  motor_left.init(motor_params_left, calibrate);
  motor_right.init(motor_params_right, calibrate);

  // Set velocity controller parameters
  // Setup PID parameters
  velocity_pid_params_left.p = 1.6;
  velocity_pid_params_left.i = 0.01;
  velocity_pid_params_left.d = 0.0;
  velocity_pid_params_left.output_ramp = 10000.0;
  velocity_pid_params_left.lpf_time_constant = 0.11;

  velocity_pid_params_right.p = 1.6;
  velocity_pid_params_right.i = 0.01;
  velocity_pid_params_right.d = 0.0;
  velocity_pid_params_right.output_ramp = 10000.0;
  velocity_pid_params_right.lpf_time_constant = 0.11;

  motor_left.set_velocity_controller(velocity_pid_params_left);
  motor_right.set_velocity_controller(velocity_pid_params_right);

  //   Set closed-loop velocity mode
  motor_left.set_control_mode(MotorGo::ControlMode::Velocity);
  motor_right.set_control_mode(MotorGo::ControlMode::Velocity);

  //   Enable motors
  motor_left.enable();
  motor_right.enable();
}

void loop()
{
  motor_left.loop();
  motor_right.loop();

  motor_left.set_target_velocity(10.0);
  motor_right.set_target_velocity(10.0);

  String str = "Velocity - Ch0: " + String(motor_left.get_velocity()) +
               " Ch1: " + String(motor_right.get_velocity());

  freq_println(str, 10);
}
