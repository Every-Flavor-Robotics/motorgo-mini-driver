#include <Arduino.h>
#include <ESPmDNS.h>

#include "configurable.h"
#include "motorgo_mini.h"
#include "pid_manager.h"
#include "web_server.h"

// UPDATE THESE VALUES
String WIFI_SSID = "YOUR_SSID";
String WIFI_PASSWORD = "YOUR_PASSWORD";

MotorGo::MotorGoMini motorgo_mini;
MotorGo::MotorChannel& motor_ch0 = motorgo_mini.ch0;
MotorGo::MotorChannel& motor_ch1 = motorgo_mini.ch1;

MotorGo::ChannelConfiguration config_ch0;
MotorGo::ChannelConfiguration config_ch1;

MotorGo::PIDParameters current_pid_params_ch0;
MotorGo::PIDParameters current_pid_params_ch1;

MotorGo::PIDParameters velocity_pid_params_ch0;
MotorGo::PIDParameters velocity_pid_params_ch1;

MotorGo::PIDParameters position_pid_params_ch0;
MotorGo::PIDParameters position_pid_params_ch1;

// declare PID manager object
MotorGo::PIDManager pid_manager;

bool motors_enabled = false;
ESPWifiConfig::Configurable<bool> enable_motors(motors_enabled, "/enable",
                                                "Enable motors");

void enable_motors_callback(bool value)
{
  if (value)
  {
    Serial.println("Enabling motors");
    motor_ch0.enable();
    motor_ch1.enable();
  }
  else
  {
    Serial.println("Disabling motors");
    motor_ch0.disable();
    motor_ch1.disable();
  }
}

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
  config_ch0.motor_config = MotorGo::MotorGoGreen;
  config_ch0.power_supply_voltage = 5.0;

  config_ch1.motor_config = MotorGo::MotorGoGreen;
  config_ch1.power_supply_voltage = 5.0;

  pid_manager.add_controller(
      "/ch0/torque", current_pid_params_ch0,
      []() { motor_ch0.set_torque_controller(current_pid_params_ch0); });

  pid_manager.add_controller(
      "/ch1/torque", current_pid_params_ch1,
      []() { motor_ch1.set_torque_controller(current_pid_params_ch1); });

  pid_manager.add_controller(
      "/ch0/velocity", velocity_pid_params_ch0,
      []() { motor_ch0.set_velocity_controller(velocity_pid_params_ch0); });

  pid_manager.add_controller(
      "/ch1/velocity", velocity_pid_params_ch1,
      []() { motor_ch1.set_velocity_controller(velocity_pid_params_ch1); });

  pid_manager.add_controller(
      "/ch0/position", position_pid_params_ch0,
      []() { motor_ch0.set_position_controller(position_pid_params_ch0); });

  pid_manager.add_controller(
      "/ch1/position", position_pid_params_ch1,
      []() { motor_ch1.set_position_controller(position_pid_params_ch1); });

  enable_motors.set_post_callback(enable_motors_callback);

  // Setup Ch0 with FOCStudio enabled
  bool calibrate = false;
  motor_ch0.init(config_ch0, calibrate);
  motor_ch1.init(config_ch1, calibrate);

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

  motor_ch0.set_velocity_controller(velocity_pid_params_ch0);
  motor_ch1.set_velocity_controller(velocity_pid_params_ch1);

  //   Set closed-loop velocity mode
  motor_ch0.set_control_mode(MotorGo::ControlMode::Velocity);
  motor_ch1.set_control_mode(MotorGo::ControlMode::Velocity);

  pid_manager.init(WIFI_SSID, WIFI_PASSWORD);

  //   motor_ch0.enable();
  //   motor_ch1.enable();
}

void loop()
{
  // Run Ch0
  motor_ch0.loop();
  motor_ch1.loop();

  motor_ch0.set_target_velocity(10.0);
  motor_ch1.set_target_velocity(10.0);

  //   String str = "Velocity - Ch0: " +
  //   String(motorgo_mini->get_ch0_velocity()) +
  //                " Ch1: " + String(motorgo_mini->get_ch1_velocity());

  //   freq_println(str, 10);
}
