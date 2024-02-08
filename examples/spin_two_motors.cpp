#include <Arduino.h>

#include "motorgo_mini.h"

MotorGo::MotorGoMini motorgo_mini;
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

  delay(3000);

  // Setup motor parameters
  motor_params_ch0.pole_pairs = 7;
  motor_params_ch0.power_supply_voltage = 5.0;
  motor_params_ch0.voltage_limit = 5.0;
  motor_params_ch0.current_limit = 300;
  motor_params_ch0.velocity_limit = 100.0;
  motor_params_ch0.calibration_voltage = 2.0;
  motor_params_ch0.reversed = false;

  motor_params_ch1.pole_pairs = 7;
  motor_params_ch1.power_supply_voltage = 5.0;
  motor_params_ch1.voltage_limit = 5.0;
  motor_params_ch1.current_limit = 300;
  motor_params_ch1.velocity_limit = 100.0;
  motor_params_ch1.calibration_voltage = 2.0;
  motor_params_ch1.reversed = true;

  // Setup Ch0
  bool calibrate = false;
  motorgo_mini.ch0.init(motor_params_ch0, calibrate, "ch0");
  motorgo_mini.ch1.init(motor_params_ch1, calibrate, "ch1");

  // Set velocity controller parameters
  // Setup PID parameters
  velocity_pid_params_ch0.p = 1.6;
  velocity_pid_params_ch0.i = 0.01;
  velocity_pid_params_ch0.d = 0.0;
  velocity_pid_params_ch0.output_ramp = 10000.0;
  velocity_pid_params_ch0.lpf_time_constant = 0.11;

  velocity_pid_params_ch1.p = 1.6;
  velocity_pid_params_ch1.i = 0.01;
  velocity_pid_params_ch1.d = 0.0;
  velocity_pid_params_ch1.output_ramp = 10000.0;
  velocity_pid_params_ch1.lpf_time_constant = 0.11;

  motorgo_mini.ch0.set_velocity_controller(velocity_pid_params_ch0);
  motorgo_mini.ch1.set_velocity_controller(velocity_pid_params_ch1);

  //   Set closed-loop velocity mode
  motorgo_mini.ch0.set_control_mode(MotorGo::ControlMode::Velocity);
  motorgo_mini.ch1.set_control_mode(MotorGo::ControlMode::Velocity);

  //   Enable motors
  motorgo_mini.ch0.enable();
  motorgo_mini.ch1.enable();
}

void loop()
{
  motorgo_mini.ch0.loop();
  motorgo_mini.ch1.loop();

  motorgo_mini.ch0.set_target_velocity(10.0);
  motorgo_mini.ch1.set_target_velocity(10.0);

  String str = "Velocity - Ch0: " + String(motorgo_mini.ch0.get_velocity()) +
               " Ch1: " + String(motorgo_mini.ch1.get_velocity());

  freq_println(str, 10);
}
