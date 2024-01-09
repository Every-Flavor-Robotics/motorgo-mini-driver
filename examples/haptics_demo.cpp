#include <Arduino.h>

#include "configurable.h"
#include "motorgo_mini.h"
#include "pid_manager.h"

// UPDATE THESE VALUES
String WIFI_SSID = "YOUR_SSID";
String WIFI_PASSWORD = "YOUR_PASSWORD";

MotorGo::MotorGoMini* motorgo_mini;
MotorGo::MotorParameters motor_params_ch0;
MotorGo::MotorParameters motor_params_ch1;

// declare PID manager object
MotorGo::PIDManager pid_manager;

// Instantiate pid motorgo pid params
MotorGo::PIDParameters velocity_pid_params_ch0;
MotorGo::PIDParameters velocity_pid_params_ch1;
MotorGo::PIDParameters position_pid_params_ch0;
MotorGo::PIDParameters position_pid_params_ch1;

bool motors_enabled = false;
ESPWifiConfig::Configurable<bool> enable_motors(motors_enabled, "/enable",
                                                "Enable motors");

void enable_motors_callback(bool value)
{
  if (value)
  {
    Serial.println("Enabling motors");
    motorgo_mini->enable_ch0();
    motorgo_mini->enable_ch1();
  }
  else
  {
    Serial.println("Disabling motors");
    motorgo_mini->disable_ch0();
    motorgo_mini->disable_ch1();
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
  bool calibrate = true;
  bool enable_foc_studio = false;
  motorgo_mini->init_ch0(motor_params_ch0, calibrate, enable_foc_studio);
  motorgo_mini->init_ch1(motor_params_ch1, calibrate, enable_foc_studio);

  // Instantiate controllers
  motorgo_mini->set_velocity_controller_ch0(velocity_pid_params_ch0);
  motorgo_mini->set_velocity_controller_ch1(velocity_pid_params_ch1);

  motorgo_mini->set_position_controller_ch0(position_pid_params_ch0);
  motorgo_mini->set_position_controller_ch1(position_pid_params_ch1);

  pid_manager.add_controller(
      "/ch0/velocity", velocity_pid_params_ch0,
      []()
      {
        motorgo_mini->set_velocity_controller_ch0(velocity_pid_params_ch0);
        motorgo_mini->reset_velocity_controller_ch0();
      });

  pid_manager.add_controller(
      "/ch1/velocity", velocity_pid_params_ch1,
      []()
      {
        motorgo_mini->set_velocity_controller_ch1(velocity_pid_params_ch1);
        motorgo_mini->reset_velocity_controller_ch1();
      });

  pid_manager.add_controller(
      "/ch0/position", position_pid_params_ch0,
      []()
      {
        motorgo_mini->set_position_controller_ch0(position_pid_params_ch0);
        motorgo_mini->reset_position_controller_ch0();
      });

  pid_manager.add_controller(
      "/ch1/position", position_pid_params_ch1,
      []()
      {
        motorgo_mini->set_position_controller_ch1(position_pid_params_ch1);
        motorgo_mini->reset_position_controller_ch1();
      });

  enable_motors.set_post_callback(enable_motors_callback);

  // initialize the PID manager
  pid_manager.init(WIFI_SSID, WIFI_PASSWORD);

  //   Set closed-loop position mode
  motorgo_mini->set_control_mode_ch0(MotorGo::ControlMode::Position);
  motorgo_mini->set_control_mode_ch1(MotorGo::ControlMode::Position);

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