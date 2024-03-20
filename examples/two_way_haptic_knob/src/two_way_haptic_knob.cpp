#include <Arduino.h>
#include <motorgo_mini.h>

#include "configurable.h"
#include "pid_manager.h"

// UPDATE THESE VALUES
String WIFI_SSID = "YOUR_SSID";
String WIFI_PASSWORD = "YOUR_PASSWORD";

MotorGo::MotorGoMini motorgo_mini;
MotorGo::MotorChannel& motor_ch0 = motorgo_mini.ch0;
MotorGo::MotorChannel& motor_ch1 = motorgo_mini.ch1;

MotorGo::ChannelConfiguration config_ch0;
MotorGo::ChannelConfiguration config_ch1;

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

  // Setup Ch0
  bool calibrate = true;
  motor_ch0.init(config_ch0, calibrate);
  motor_ch1.init(config_ch1, calibrate);

  // Instantiate controllers
  motor_ch0.set_velocity_controller(velocity_pid_params_ch0);
  motor_ch1.set_velocity_controller(velocity_pid_params_ch1);

  motor_ch0.set_position_controller(position_pid_params_ch0);
  motor_ch1.set_position_controller(position_pid_params_ch1);

  pid_manager.add_controller(
      "/ch0/velocity", velocity_pid_params_ch0,
      []()
      {
        motor_ch0.set_velocity_controller(velocity_pid_params_ch0);
        motor_ch0.reset_velocity_controller();
      });

  pid_manager.add_controller(
      "/ch1/velocity", velocity_pid_params_ch1,
      []()
      {
        motor_ch1.set_velocity_controller(velocity_pid_params_ch1);
        motor_ch1.reset_velocity_controller();
      });

  pid_manager.add_controller(
      "/ch0/position", position_pid_params_ch0,
      []()
      {
        motor_ch0.set_position_controller(position_pid_params_ch0);
        motor_ch0.reset_position_controller();
      });

  pid_manager.add_controller(
      "/ch1/position", position_pid_params_ch1,
      []()
      {
        motor_ch1.set_position_controller(position_pid_params_ch1);
        motor_ch1.reset_position_controller();
      });

  enable_motors.set_post_callback(enable_motors_callback);

  // initialize the PID manager
  pid_manager.init(WIFI_SSID, WIFI_PASSWORD);

  //   Set closed-loop position mode
  motor_ch0.set_control_mode(MotorGo::ControlMode::Position);
  motor_ch1.set_control_mode(MotorGo::ControlMode::Position);

  // enable controllers and prepare for the loop
  //   motor_ch0.enable();
  //   motor_ch1.enable();
}

void loop()
{
  // Run Ch0
  motor_ch0.loop();
  motor_ch1.loop();

  // measure positions
  float ch0_pos = motor_ch0.get_position();
  float ch1_pos = motor_ch1.get_position();

  // set target positions between each motor
  motor_ch0.set_target_position(ch1_pos);
  motor_ch1.set_target_position(ch0_pos);

  String x = "ch0 pos: " + String(ch0_pos);
  String y = "ch1 pos: " + String(ch1_pos);

  freq_println(x + " | " + y, 10);
}