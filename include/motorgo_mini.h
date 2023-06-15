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
  Voltage,
  Velocity,
  Torque,
  Position,
  VelocityOpenLoop,
  PositionOpenLoop
};

// TODO: These are global because the SimpleFOC commander API doesn't support
// lambdas to save state in for the callback Need to decide if this is the best
// solution, or if there is something better we can do
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
  void init_ch0();
  void init_ch1();

  // Init motors and encoders, optionally calibrating and/or enabling FOCStudio
  void init_ch0(bool should_calibrate, bool enable_foc_studio);
  void init_ch1(bool should_calibrate, bool enable_foc_studio);

  void loop_ch0();
  void loop_ch1();

  void set_target_ch0(float target_ch0, float target_ch1);
  void set_target_ch1(float target_ch0, float target_ch1);

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

  // Get states
  float get_ch0_position();
  float get_ch0_velocity();
  float get_ch0_torque();
  float get_ch0_voltage();

  float get_ch1_position();
  float get_ch1_velocity();
  float get_ch1_torque();
  float get_ch1_voltage();

  // void doTargetch0(char* cmd);
  // void doTargetch1(char* cmd);

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

  // ch1 Motor and Encoder pins
  const int k_ch1_enc_cs = 48;
  const int k_ch1_gpio_uh = 9;
  const int k_ch1_gpio_ul = 13;
  const int k_ch1_gpio_vh = 10;
  const int k_ch1_gpio_vl = 21;
  const int k_ch1_gpio_wh = 11;
  const int k_ch1_gpio_wl = 14;

  // Motor and Encoder parameters
  const float k_voltage_power_supply = 9.0;
  const float k_voltage_limit = 9.0;
  const float k_current_limit = 10.0;
  const float k_velocity_limit = 100.0;
  const float k_voltage_calibration = 2.0;

  bool should_calibrate;
  bool enable_foc_studio;

  // Encoder, motor, and driver instances
  // MT6701Sensor encoder_ch0;
  MagneticSensorMT6701SSI encoder_ch0;
  BLDCDriver6PWM driver_ch0;
  CalibratedSensor sensor_calibrated_ch0;

  MagneticSensorMT6701SSI encoder_ch1;
  CalibratedSensor sensor_calibrated_ch1;
  BLDCDriver6PWM driver_ch1;

  void init_helper(BLDCMotor& motor, BLDCDriver6PWM& driver,
                   CalibratedSensor& sensor_calibrated,
                   MagneticSensorMT6701SSI& encoder, const char* name);

  // set_control_mode_helper sets parameters for each control_mode option
  // Sets values for torque controller type and motion controller type
  void set_control_mode_helper(BLDCMotor& motor, ControlMode control_mode);
};

}  // namespace MotorGo

#endif  // MOTORGO_MINI_H
