// Header file for the MotorGo Mini driver class.

#ifndef MOTORGO_MINI_H
#define MOTORGO_MINI_H

#include <SPI.h>
#include <SimpleFOC.h>
#include <Wire.h>

#include "encoders/calibrated/CalibratedSensor.h"
#include "encoders/mt6701/MagneticSensorMT6701SSI.h"

namespace MotorGo
{

// Control config struct
enum ControlMode
{
  None,
  Voltage,
  Velocity,
  Torque,
  Position,
  VelocityOpenLoop,
  PositionOpenLoop
};

struct PIDParameters
{
  float p;
  float i;
  float d;
  float output_ramp = 10000.0f;
  float lpf_time_constant = 0.1f;
  float limit = 10000.0f;
};

struct MotorParameters
{
  int pole_pairs;
  float power_supply_voltage;
  float voltage_limit;
  float current_limit = 1000.0f;
  float velocity_limit = 1000.0f;
  float calibration_voltage;
};

// TODO: These are global because the SimpleFOC commander API doesn't
// support lambdas to save state in for the callback Need to decide if this
// is the best solution, or if there is something better we can do
extern Commander command;
extern BLDCMotor motor_ch0;
extern BLDCMotor motor_ch1;
extern SPIClass hspi;

class MotorGoMini
{
 public:
  MotorGoMini();

  // Init motors and encoders, calibration is automatically loaded and FOCStudio
  // is disabled
  void init_ch0(MotorParameters params);
  void init_ch1(MotorParameters params);

  // Init motors and encoders, optionally calibrating and/or enabling FOCStudio
  void init_ch0(MotorParameters params, bool should_calibrate,
                bool enable_foc_studio);
  void init_ch1(MotorParameters params, bool should_calibrate,
                bool enable_foc_studio);

  // Run control loop, should be called at fixed frequency
  void loop_ch0();
  void loop_ch1();

  void set_torque_controller_ch0(PIDParameters params);
  void set_torque_controller_ch1(PIDParameters params);

  void set_velocity_controller_ch0(PIDParameters params);
  void set_velocity_controller_ch1(PIDParameters params);

  void set_position_controller_ch0(PIDParameters params);
  void set_position_controller_ch1(PIDParameters params);

  // Enable and disable motors
  void enable_ch0();
  void enable_ch1();
  void disable_ch0();
  void disable_ch1();

  // Set control mode
  void set_control_mode_ch0(ControlMode mode);
  void set_control_mode_ch1(ControlMode mode);

  //  Set target velocity
  void set_target_velocity_ch0(float target);
  void set_target_velocity_ch1(float target);

  // Set target torque
  void set_target_torque_ch0(float target);
  void set_target_torque_ch1(float target);

  // Set target position
  void set_target_position_ch0(float target);
  void set_target_position_ch1(float target);

  // Zero Position
  void zero_position_ch0();
  void zero_position_ch1();

  // Get states
  float get_ch0_position();
  float get_ch0_velocity();
  float get_ch0_torque();
  float get_ch0_voltage();

  float get_ch1_position();
  float get_ch1_velocity();
  float get_ch1_torque();
  float get_ch1_voltage();

 private:
  // Encoder I2C bus
  const int enc_sda = 35;
  const int enc_scl = 36;

  // ch0 Motor and Encoder pins
  const int k_ch0_enc_cs = 37;
  const int k_ch0_gpio_uh = 18;
  const int k_ch0_gpio_ul = 15;
  const int k_ch0_gpio_vh = 17;
  const int k_ch0_gpio_vl = 5;
  const int k_ch0_gpio_wh = 16;
  const int k_ch0_gpio_wl = 6;
  const int k_ch0_current_u = 7;
  const int k_ch0_current_w = 4;

  // ch1 Motor and Encoder pins
  const int k_ch1_enc_cs = 48;
  const int k_ch1_gpio_uh = 9;
  const int k_ch1_gpio_ul = 13;
  const int k_ch1_gpio_vh = 10;
  const int k_ch1_gpio_vl = 21;
  const int k_ch1_gpio_wh = 11;
  const int k_ch1_gpio_wl = 14;
  const int k_ch1_current_u = 47;
  const int k_ch1_current_w = 12;

  // Additional motor and encoder parameters
  MotorParameters motor_params_ch0;
  MotorParameters motor_params_ch1;

  // Store whether the parameters have been set
  // If not, the motor will not run be disabled when a command is received
  bool pid_torque_ch0_enabled = false;
  bool pid_torque_ch1_enabled = false;

  bool pid_velocity_ch0_enabled = false;
  bool pid_velocity_ch1_enabled = false;

  bool pid_position_ch0_enabled = false;
  bool pid_position_ch1_enabled = false;

  // Current targets
  // Either velocity or torque will be used depending on the control mode
  // Store both to avoid erroneous behaviors due to switching units
  // when switching between control modes
  // Set to None by default to require user to set a control mode
  ControlMode control_mode_ch0 = None;
  ControlMode control_mode_ch1 = None;
  // Rad/s
  float target_velocity_ch0 = 0.0f;
  float target_velocity_ch1 = 0.0f;
  // N*m
  float target_torque_ch0 = 0.0f;
  float target_torque_ch1 = 0.0f;
  // Rad
  float target_position_ch0 = 0.0f;
  float target_position_ch1 = 0.0f;
  // V
  float target_voltage_ch0 = 0.0f;
  float target_voltage_ch1 = 0.0f;

  // Calibration parameters
  // If should_calibrate is set to true, the motor will be calibrated on startup
  // Else, the calibration will be loaded from EEPROM. If no calibration is
  // found, the motor will be calibrated anyway and the calibration will be
  // saved to EEPROM
  bool should_calibrate_ch0;
  bool should_calibrate_ch1;

  // If enable_foc_studio is set to true, data will be written to serial
  // to communicate with FOCStudio
  // This significantly slows the control loop, enable only when necessary
  // TODO: Enable "offline" tuning
  bool enable_foc_studio_ch0;
  bool enable_foc_studio_ch1;

  // Encoder, motor, and driver instances
  // MT6701Sensor encoder_ch0;
  MagneticSensorMT6701SSI encoder_ch0;
  BLDCDriver6PWM driver_ch0;
  CalibratedSensor sensor_calibrated_ch0;

  MagneticSensorMT6701SSI encoder_ch1;
  CalibratedSensor sensor_calibrated_ch1;
  BLDCDriver6PWM driver_ch1;

  // Helper functions
  // init_helper configures the motor and encoder
  void init_helper(MotorParameters params, bool should_calibrate,
                   bool enable_foc_studio, BLDCMotor& motor,
                   BLDCDriver6PWM& driver, CalibratedSensor& sensor_calibrated,
                   MagneticSensorMT6701SSI& encoder, const char* name);

  // set_control_mode_helper sets parameters for each control_mode option
  // Sets values for torque controller type and motion controller type
  void set_control_mode_helper(BLDCMotor& motor, ControlMode control_mode);

  // Set correct targets
  void set_target_helper_ch0();
  void set_target_helper_ch1();

  void set_torque_controller_helper(BLDCMotor& motor, PIDParameters params);
  void set_velocity_controller_helper(BLDCMotor& motor, PIDParameters params);
  void set_position_controller_helper(BLDCMotor& motor, PIDParameters params);
};
}  // namespace MotorGo

#endif  // MOTORGO_MINI_H
