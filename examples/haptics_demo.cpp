#include <Arduino.h>

#include "configurable.h"
#include "motorgo_mini.h"
#include "readable.h"
#include "web_server.h"

MotorGo::MotorGoMini* motorgo_mini;
MotorGo::MotorParameters motor_params_ch0;
MotorGo::MotorParameters motor_params_ch1;

// instantiate pid motorgo pid params
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

bool motors_enabled = false;
ESPWifiConfig::Configurable<bool> enable_motors(motors_enabled, "/enable",
                                                "Enable motors");

void enable_motors_callback(bool value)
{
  if (value)
  {
    motorgo_mini->enable_ch0();
    motorgo_mini->enable_ch1();
  }
  else
  {
    motorgo_mini->disable_ch0();
    motorgo_mini->disable_ch1();
  }
}

bool save_pid_params_ch0()
{
  motorgo_mini->save_position_controller_ch0();
  motorgo_mini->save_velocity_controller_ch0();
  motorgo_mini->save_torque_controller_ch0();

  return true;
}

bool save_pid_params_ch1()
{
  motorgo_mini->save_position_controller_ch1();
  motorgo_mini->save_velocity_controller_ch1();
  motorgo_mini->save_torque_controller_ch1();

  return true;
}

bool load_pid_params_ch0()
{
  motorgo_mini->load_position_controller_ch0();
  motorgo_mini->load_velocity_controller_ch0();
  motorgo_mini->load_torque_controller_ch0();

  return true;
}

bool load_pid_params_ch1()
{
  motorgo_mini->load_position_controller_ch1();
  motorgo_mini->load_velocity_controller_ch1();
  motorgo_mini->load_torque_controller_ch1();

  return true;
}

void get_pid_params_ch0()
{
  position_pid_params_ch0 = motorgo_mini->get_position_controller_ch0();
  velocity_pid_params_ch0 = motorgo_mini->get_velocity_controller_ch0();
  current_pid_params_ch0 = motorgo_mini->get_torque_controller_ch0();
}

void get_pid_params_ch1()
{
  position_pid_params_ch1 = motorgo_mini->get_position_controller_ch1();
  velocity_pid_params_ch1 = motorgo_mini->get_velocity_controller_ch1();
  current_pid_params_ch1 = motorgo_mini->get_torque_controller_ch1();
}

void position_pid_update_ch0(float value)
{
  motorgo_mini->disable_ch0();
  motorgo_mini->set_position_controller_ch0(position_pid_params_ch0);

  //   Reset position, velocity, and torque controllers;
  motorgo_mini->reset_position_controller_ch0();
  motorgo_mini->reset_velocity_controller_ch0();
  motorgo_mini->reset_torque_controller_ch0();

  motorgo_mini->enable_ch0();
}

void position_pid_update_ch1(float value)
{
  motorgo_mini->disable_ch1();
  motorgo_mini->set_position_controller_ch1(position_pid_params_ch1);

  //   Reset position, velocity, and torque controllers;
  motorgo_mini->reset_position_controller_ch1();
  motorgo_mini->reset_velocity_controller_ch1();
  motorgo_mini->reset_torque_controller_ch1();

  motorgo_mini->enable_ch1();
}

void velocity_pid_update_ch0(float value)
{
  motorgo_mini->disable_ch0();
  motorgo_mini->set_velocity_controller_ch0(velocity_pid_params_ch0);

  //   Reset velocity and torque controllers;
  motorgo_mini->reset_velocity_controller_ch0();
  motorgo_mini->reset_torque_controller_ch0();

  motorgo_mini->enable_ch0();
}

void velocity_pid_update_ch1(float value)
{
  motorgo_mini->disable_ch1();
  motorgo_mini->set_velocity_controller_ch1(velocity_pid_params_ch1);

  //   Reset velocity and torque controllers;
  motorgo_mini->reset_velocity_controller_ch1();
  motorgo_mini->reset_torque_controller_ch1();

  motorgo_mini->enable_ch1();
}

void current_pid_update_ch0(float value)
{
  motorgo_mini->disable_ch0();
  motorgo_mini->set_torque_controller_ch0(current_pid_params_ch0);

  //   Reset torque controller;
  motorgo_mini->reset_torque_controller_ch0();

  motorgo_mini->enable_ch0();
}

void current_pid_update_ch1(float value)
{
  motorgo_mini->disable_ch1();
  motorgo_mini->set_torque_controller_ch1(current_pid_params_ch1);

  //   Reset torque controller;
  motorgo_mini->reset_torque_controller_ch1();

  motorgo_mini->enable_ch1();
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

  enable_motors.set_post_callback(enable_motors_callback);

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

  // Instantiate motorgo mini board
  motorgo_mini = new MotorGo::MotorGoMini();

  // Setup Ch0 with FOCStudio enabled
  bool calibrate = false;
  bool enable_foc_studio = false;
  motorgo_mini->init_ch0(motor_params_ch0, calibrate, enable_foc_studio);
  motorgo_mini->init_ch1(motor_params_ch1, calibrate, enable_foc_studio);

  // Set velocity controller parameters
  // Setup PID parameters - velocity

  float vel_p = 0.5;
  float vel_i = 0.0;
  float vel_d = 0.0;

  velocity_pid_params_ch0.p = vel_p;
  velocity_pid_params_ch0.i = vel_i;
  velocity_pid_params_ch0.d = vel_d;
  velocity_pid_params_ch0.output_ramp = 10000.0;
  velocity_pid_params_ch0.lpf_time_constant = 0.11;

  velocity_pid_params_ch1.p = vel_p;
  velocity_pid_params_ch1.i = vel_i;
  velocity_pid_params_ch1.d = vel_d;
  velocity_pid_params_ch1.output_ramp = 10000.0;
  velocity_pid_params_ch1.lpf_time_constant = 0.11;

  // Setup PID parameters - position
  // set up p controller only for position control.
  float pos_p = 5.0;
  float pos_i = 0.5;
  float pos_d = 0.0;

  position_pid_params_ch0.p = pos_p;
  position_pid_params_ch0.i = pos_i;
  position_pid_params_ch0.d = pos_d;
  position_pid_params_ch0.output_ramp = 10000.0;
  position_pid_params_ch0.lpf_time_constant = 0.11;

  position_pid_params_ch1.p = pos_p;
  position_pid_params_ch1.i = pos_i;
  position_pid_params_ch1.d = pos_d;
  position_pid_params_ch1.output_ramp = 10000.0;
  position_pid_params_ch1.lpf_time_constant = 0.11;

  // Instantiate controllers
  motorgo_mini->set_velocity_controller_ch0(velocity_pid_params_ch0);
  motorgo_mini->set_velocity_controller_ch1(velocity_pid_params_ch1);

  motorgo_mini->set_position_controller_ch0(position_pid_params_ch0);
  motorgo_mini->set_position_controller_ch1(position_pid_params_ch1);

  //   Set closed-loop position mode
  motorgo_mini->set_control_mode_ch0(MotorGo::ControlMode::Position);
  motorgo_mini->set_control_mode_ch1(MotorGo::ControlMode::Position);

  //   Print url: http://{IP_ADDRESS}:PORT
  Serial.print("Please connect to http://");
  Serial.print(WiFi.localIP());
  Serial.print(":");
  Serial.println(8080);

  // enable controllers and prepare for the loop
  //   motorgo_mini->enable_ch0();
  //   motorgo_mini->enable_ch1();
}

void loop()
{
  // Run Ch0
  motorgo_mini->loop_ch0();
  motorgo_mini->loop_ch1();

  // measure positions
  float ch0_pos = motorgo_mini->get_ch0_position();
  float ch1_pos = motorgo_mini->get_ch1_position();

  // set target positions between each motor
  motorgo_mini->set_target_position_ch0(ch1_pos);
  motorgo_mini->set_target_position_ch1(ch0_pos);

  String x = "ch0 pos: " + String(ch0_pos);
  String y = "ch1 pos: " + String(ch1_pos);

  freq_println(x + " | " + y, 10);
}