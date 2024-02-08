// Header file for the Motor Channel class.

#ifndef MOTOR_CHANNEL_H
#define MOTOR_CHANNEL_H

#include <SPI.h>
#include <SimpleFOC.h>
#include <Wire.h>

#include "encoders/calibrated/CalibratedSensor.h"
#include "encoders/mt6701/MagneticSensorMT6701SSI.h"
#include "motorgo_common.h"

namespace MotorGo
{

class MotorChannel
{
 public:
  MotorChannel(BLDCChannelParameters params);
  MotorChannel(const MotorChannel&) = delete;  // Delete copy constructor
  MotorChannel& operator=(const MotorChannel&) =
      delete;  // Delete copy assignment operator

  // Init motors and encoders, calibration is automatically loaded
  void init(MotorParameters params, const char* name);

  // Init motors and encoders, optionally calibrating
  void init(MotorParameters params, bool should_calibrate, const char* name);

  // Run control loop, call as fast as possible
  void loop();

  // Retrieve built-in controllers
  PIDParameters get_torque_controller();
  PIDParameters get_velocity_controller();
  PIDParameters get_position_controller();

  // Set the parameters for the built-in controllers
  void set_torque_controller(PIDParameters params);
  void set_velocity_controller(PIDParameters params);
  void set_position_controller(PIDParameters params);

  // Reset the state of the built-in controllers
  // This will clear the integral term and reset the output to 0
  void reset_torque_controller();
  void reset_velocity_controller();
  void reset_position_controller();

  //   Enable and disable motors
  //   Disabling will immediately set the voltage command to 0
  void enable();
  void disable();

  //   Set control mode - changes which controllers are used
  void set_control_mode(ControlMode mode);

  //   Set the targets for each control mode
  void set_target_velocity(float target);
  void set_target_torque(float target);
  void set_target_position(float target);
  void set_target_voltage(float target);

  //   Save and load controller parameters to EEPROM
  void save_torque_controller();
  void save_velocity_controller();
  void save_position_controller();

  void load_torque_controller();
  void load_velocity_controller();
  void load_position_controller();

  // Set the current encoder reading as the zero position
  void zero_position();

  // Get current motor state
  float get_position();
  float get_velocity();
  float get_torque();
  float get_voltage();

 private:
  // Encoder, motor, and driver instances
  BLDCMotor motor;
  BLDCDriver6PWM driver;
  MagneticSensorMT6701SSI encoder;
  //   Calibrated sensor stores the calibration parameters
  CalibratedSensor sensor_calibrated;

  // Additional motor and encoder parameters
  MotorParameters motor_params;
  Direction motor_direction;

  // Current targets
  // target_velocity, target_position, target_voltage, or target_torque will
  // be used depending on the control mode. We store all of them separately
  // to avoid erroneous motion due to switching units when switching between
  // control modes
  // Set to None by default to require user to set a control mode
  ControlMode control_mode = None;

  // Rad/s
  float target_velocity = 0.0f;
  // N*m
  float target_torque = 0.0f;
  // Rad
  float target_position = 0.0f;
  // V
  float target_voltage = 0.0f;

  // Calibration parameters
  // If should_calibrate is set to true, the motor will be calibrated on startup
  // Else, the calibration will be loaded from EEPROM. If no calibration is
  // found, the motor will be calibrated anyway and the calibration will be
  // saved to EEPROM
  bool should_calibrate;

  // Store whether the parameters have been set
  // If not, the motor will not run be disabled when a command is received
  bool pid_torque_enabled = false;
  bool pid_velocity_enabled = false;
  bool pid_position_enabled = false;

  //   Helper functions for loading and saving PID parameters
  void save_controller_helper(const char* key, const PIDController& controller,
                              const LowPassFilter& lpf);

  void load_controller_helper(const char* key, PIDController& controller,
                              LowPassFilter& lpf);
};

}  // namespace MotorGo

#endif  // MOTOR_CHANNEL_H