#include <Arduino.h>

#include "motorgo_mini.h"

#define LOOP_HZ 1000
#define LOOP_US 1000000 / LOOP_HZ

MotorGo::MotorGoMini* motorgo_mini;
MotorGo::MotorParameters motor_params_ch0;
MotorGo::PIDParameters velocity_pid_params_ch0;

void setup()
{
  Serial.begin(115200);

  // Setup motor parameters
  motor_params_ch0.pole_pairs = 11;
  motor_params_ch0.power_supply_voltage = 9.0;
  motor_params_ch0.voltage_limit = 9.0;
  motor_params_ch0.current_limit = 320;
  motor_params_ch0.velocity_limit = 100.0;
  motor_params_ch0.calibration_voltage = 2.0;

  // Setup PID parameters
  velocity_pid_params_ch0.p = 4.0;
  velocity_pid_params_ch0.i = 0.5;
  velocity_pid_params_ch0.d = 0.0;
  velocity_pid_params_ch0.output_ramp = 10000.0;
  velocity_pid_params_ch0.lpf_time_constant = 0.1;

  // Instantiate motorgo mini board
  motorgo_mini = new MotorGo::MotorGoMini();

  // Setup Ch0 with FOCStudio enabled
  motorgo_mini->init_ch0(motor_params_ch0, false, false);
  // Set velocity controller parameters
  motorgo_mini->set_velocity_controller_ch0(velocity_pid_params_ch0);
  // Set closed-loop velocity control mode
  motorgo_mini->set_control_mode_ch0(MotorGo::ControlMode::Velocity);

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
