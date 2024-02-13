#include <Arduino.h>
#include <ESPmDNS.h>

#include "configurable.h"
#include "motorgo_mini.h"
#include "readable.h"
#include "web_server.h"

MotorGo::MotorGoMini motorgo_mini;
MotorGo::MotorChannel& motor_ch0 = motorgo_mini.ch0;
MotorGo::MotorChannel& motor_ch1 = motorgo_mini.ch1;

MotorGo::MotorParameters motor_params_ch0;
MotorGo::MotorParameters motor_params_ch1;

MotorGo::PIDParameters current_pid_params_ch0;
MotorGo::PIDParameters current_pid_params_ch1;

MotorGo::PIDParameters velocity_pid_params_ch0;
MotorGo::PIDParameters velocity_pid_params_ch1;

MotorGo::PIDParameters position_pid_params_ch0;
MotorGo::PIDParameters position_pid_params_ch1;

ESPWifiConfig::Configurable<float> current_p_ch0(
    current_pid_params_ch0.p, "/ch0/current/p",
    "P Gain for Channel 0 current controller");

ESPWifiConfig::Configurable<float> current_i_ch0(
    current_pid_params_ch0.i, "/ch0/current/i",
    "I Gain for Channel 0 current controller");

ESPWifiConfig::Configurable<float> current_d_ch0(
    current_pid_params_ch0.d, "/ch0/current/d",
    "D Gain for Channel 0 current controller");

ESPWifiConfig::Configurable<float> current_lpf_ch0(
    current_pid_params_ch0.lpf_time_constant, "/ch0/current/lpf",
    "Low pass filter time constant for Channel 0 current controller");

ESPWifiConfig::Configurable<float> current_p_ch1(
    current_pid_params_ch1.p, "/ch1/current/p",
    "P Gain for Channel 1 current controller");

ESPWifiConfig::Configurable<float> current_i_ch1(
    current_pid_params_ch1.i, "/ch1/current/i",
    "I Gain for Channel 1 current controller");

ESPWifiConfig::Configurable<float> current_d_ch1(
    current_pid_params_ch1.d, "/ch1/current/d",
    "D Gain for Channel 1 current controller");

ESPWifiConfig::Configurable<float> current_lpf_ch1(
    current_pid_params_ch1.lpf_time_constant, "/ch1/current/lpf",
    "Low pass filter time constant for Channel 1 current controller");

ESPWifiConfig::Configurable<float> velocity_p_ch0(
    velocity_pid_params_ch0.p, "/ch0/velocity/p",
    "P Gain for Channel 0 velocity controller");

ESPWifiConfig::Configurable<float> velocity_i_ch0(
    velocity_pid_params_ch0.i, "/ch0/velocity/i",
    "I Gain for Channel 0 velocity controller");

ESPWifiConfig::Configurable<float> velocity_d_ch0(
    velocity_pid_params_ch0.d, "/ch0/velocity/d",
    "D Gain for Channel 0 velocity controller");

ESPWifiConfig::Configurable<float> velocity_lpf_ch0(
    velocity_pid_params_ch0.lpf_time_constant, "/ch0/velocity/lpf",
    "Low pass filter time constant for Channel 0 velocity controller");

ESPWifiConfig::Configurable<float> velocity_p_ch1(
    velocity_pid_params_ch1.p, "/ch1/velocity/p",
    "P Gain for Channel 1 velocity controller");

ESPWifiConfig::Configurable<float> velocity_i_ch1(
    velocity_pid_params_ch1.i, "/ch1/velocity/i",
    "I Gain for Channel 1 velocity controller");

ESPWifiConfig::Configurable<float> velocity_d_ch1(
    velocity_pid_params_ch1.d, "/ch1/velocity/d",
    "D Gain for Channel 1 velocity controller");

ESPWifiConfig::Configurable<float> velocity_lpf_ch1(
    velocity_pid_params_ch1.lpf_time_constant, "/ch1/velocity/lpf",
    "Low pass filter time constant for Channel 1 velocity controller");

ESPWifiConfig::Configurable<float> position_p_ch0(
    position_pid_params_ch0.p, "/ch0/position/p",
    "P Gain for Channel 0 position controller");

ESPWifiConfig::Configurable<float> position_i_ch0(
    position_pid_params_ch0.i, "/ch0/position/i",
    "I Gain for Channel 0 position controller");

ESPWifiConfig::Configurable<float> position_d_ch0(
    position_pid_params_ch0.d, "/ch0/position/d",
    "D Gain for Channel 0 position controller");

ESPWifiConfig::Configurable<float> position_lpf_ch0(
    position_pid_params_ch0.lpf_time_constant, "/ch0/position/lpf",
    "Low pass filter time constant for Channel 0 position controller");

ESPWifiConfig::Configurable<float> position_p_ch1(
    position_pid_params_ch1.p, "/ch1/position/p",
    "P Gain for Channel 1 position controller");

ESPWifiConfig::Configurable<float> position_i_ch1(
    position_pid_params_ch1.i, "/ch1/position/i",
    "I Gain for Channel 1 position controller");

ESPWifiConfig::Configurable<float> position_d_ch1(
    position_pid_params_ch1.d, "/ch1/position/d",
    "D Gain for Channel 1 position controller");

ESPWifiConfig::Configurable<float> position_lpf_ch1(
    position_pid_params_ch1.lpf_time_constant, "/ch1/position/lpf",
    "Low pass filter time constant for Channel 1 position controller");

bool save_pid_params_ch0()
{
  motor_ch0.save_position_controller();
  motor_ch0.save_velocity_controller();
  motor_ch0.save_torque_controller();

  return true;
}

bool save_pid_params_ch1()
{
  motor_ch1.save_position_controller();
  motor_ch1.save_velocity_controller();
  motor_ch1.save_torque_controller();

  return true;
}

bool load_pid_params_ch0()
{
  motor_ch0.load_position_controller();
  motor_ch0.load_velocity_controller();
  motor_ch0.load_torque_controller();

  return true;
}

bool load_pid_params_ch1()
{
  motor_ch1.load_position_controller();
  motor_ch1.load_velocity_controller();
  motor_ch1.load_torque_controller();

  return true;
}

// ESPWifiConfig::Readable<bool> call_save_pid_params_ch0(
//     save_pid_params_ch0, "/ch0/save_pid_params",
//     "Save PID parameters for Channel 0");

// ESPWifiConfig::Readable<bool> call_save_pid_params_ch1(
//     save_pid_params_ch1, "/ch1/save_pid_params",
//     "Save PID parameters for Channel 1");

// ESPWifiConfig::Readable<bool> call_load_pid_params_ch0(
//     load_pid_params_ch0, "/ch0/load_pid_params",
//     "Save PID parameters for Channel 0");

// ESPWifiConfig::Readable<bool> call_load_pid_params_ch1(
//     load_pid_params_ch1, "/ch1/load_pid_params",
//     "Save PID parameters for Channel 1");

void get_pid_params_ch0()
{
  position_pid_params_ch0 = motor_ch0.get_position_controller();
  velocity_pid_params_ch0 = motor_ch0.get_velocity_controller();
  current_pid_params_ch0 = motor_ch0.get_torque_controller();
}

void get_pid_params_ch1()
{
  position_pid_params_ch1 = motor_ch1.get_position_controller();
  velocity_pid_params_ch1 = motor_ch1.get_velocity_controller();
  current_pid_params_ch1 = motor_ch1.get_torque_controller();
}

void position_pid_update_ch0(float value)
{
  motor_ch0.disable();
  motor_ch0.set_position_controller(position_pid_params_ch0);

  //   Reset position, velocity, and torque controllers;
  motor_ch0.reset_position_controller();
  motor_ch0.reset_velocity_controller();
  motor_ch0.reset_torque_controller();
}

void position_pid_update_ch1(float value)
{
  motor_ch1.disable();
  motor_ch1.set_position_controller(position_pid_params_ch1);

  //   Reset position, velocity, and torque controllers;
  motor_ch1.reset_position_controller();
  motor_ch1.reset_velocity_controller();
  motor_ch1.reset_torque_controller();
}

void velocity_pid_update_ch0(float value)
{
  motor_ch0.disable();
  motor_ch0.set_velocity_controller(velocity_pid_params_ch0);

  //   Reset velocity and torque controllers;
  motor_ch0.reset_velocity_controller();
  motor_ch0.reset_torque_controller();
}

void velocity_pid_update_ch1(float value)
{
  motor_ch1.disable();
  motor_ch1.set_velocity_controller(velocity_pid_params_ch1);

  //   Reset velocity and torque controllers;
  motor_ch1.reset_velocity_controller();
  motor_ch1.reset_torque_controller();
}

void current_pid_update_ch0(float value)
{
  motor_ch0.disable();
  motor_ch0.set_torque_controller(current_pid_params_ch0);

  //   Reset torque controller;
  motor_ch0.reset_torque_controller();
}

void current_pid_update_ch1(float value)
{
  motor_ch1.disable();
  motor_ch1.set_torque_controller(current_pid_params_ch1);

  //   Reset torque controller;
  motor_ch1.reset_torque_controller();
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

  //   Configure PID update callbacks
  current_p_ch0.set_get_callback(get_pid_params_ch0);
  current_i_ch0.set_get_callback(get_pid_params_ch0);
  current_d_ch0.set_get_callback(get_pid_params_ch0);
  current_lpf_ch0.set_get_callback(get_pid_params_ch0);

  current_p_ch1.set_get_callback(get_pid_params_ch1);
  current_i_ch1.set_get_callback(get_pid_params_ch1);
  current_d_ch1.set_get_callback(get_pid_params_ch1);
  current_lpf_ch1.set_get_callback(get_pid_params_ch1);

  velocity_p_ch0.set_get_callback(get_pid_params_ch0);
  velocity_i_ch0.set_get_callback(get_pid_params_ch0);
  velocity_d_ch0.set_get_callback(get_pid_params_ch0);
  velocity_lpf_ch0.set_get_callback(get_pid_params_ch0);

  velocity_p_ch1.set_get_callback(get_pid_params_ch1);
  velocity_i_ch1.set_get_callback(get_pid_params_ch1);
  velocity_d_ch1.set_get_callback(get_pid_params_ch1);
  velocity_lpf_ch1.set_get_callback(get_pid_params_ch1);

  position_p_ch0.set_get_callback(get_pid_params_ch0);
  position_i_ch0.set_get_callback(get_pid_params_ch0);
  position_d_ch0.set_get_callback(get_pid_params_ch0);
  position_lpf_ch0.set_get_callback(get_pid_params_ch0);

  position_p_ch1.set_get_callback(get_pid_params_ch1);
  position_i_ch1.set_get_callback(get_pid_params_ch1);
  position_d_ch1.set_get_callback(get_pid_params_ch1);

  current_p_ch0.set_post_callback(current_pid_update_ch0);
  current_i_ch0.set_post_callback(current_pid_update_ch0);
  current_d_ch0.set_post_callback(current_pid_update_ch0);
  current_lpf_ch0.set_post_callback(current_pid_update_ch0);

  current_p_ch1.set_post_callback(current_pid_update_ch1);
  current_i_ch1.set_post_callback(current_pid_update_ch1);
  current_d_ch1.set_post_callback(current_pid_update_ch1);
  current_lpf_ch1.set_post_callback(current_pid_update_ch1);

  velocity_p_ch0.set_post_callback(velocity_pid_update_ch0);
  velocity_i_ch0.set_post_callback(velocity_pid_update_ch0);
  velocity_d_ch0.set_post_callback(velocity_pid_update_ch0);
  velocity_lpf_ch0.set_post_callback(velocity_pid_update_ch0);

  velocity_p_ch1.set_post_callback(velocity_pid_update_ch1);
  velocity_i_ch1.set_post_callback(velocity_pid_update_ch1);
  velocity_d_ch1.set_post_callback(velocity_pid_update_ch1);
  velocity_lpf_ch1.set_post_callback(velocity_pid_update_ch1);

  position_p_ch0.set_post_callback(position_pid_update_ch0);
  position_i_ch0.set_post_callback(position_pid_update_ch0);
  position_d_ch0.set_post_callback(position_pid_update_ch0);
  position_lpf_ch0.set_post_callback(position_pid_update_ch0);

  ESPWifiConfig::WebServer::getInstance().start();

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

  // Setup Ch0 with FOCStudio enabled
  bool calibrate = false;
  motor_ch0.init(motor_params_ch0, calibrate);
  motor_ch1.init(motor_params_ch1, calibrate);

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

  //   Print url: http://{IP_ADDRESS}:PORT
  Serial.print("Please connect to http://");
  Serial.print(WiFi.localIP());
  Serial.print(":");
  Serial.println(8080);

  //   motorgo_mini->enable_ch0();
  //   motorgo_mini->enable_ch1();
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
