// Header file for the Motor Channel class.

#ifndef MOTOR_CHANNEL_H
#define MOTOR_CHANNEL_H

#include <SPI.h>
#include <SimpleFOC.h>
#include <Wire.h>

#include "encoders/calibrated/CalibratedSensor.h"
#include "encoders/mt6701/MagneticSensorMT6701SSI.h"

namespace MotorGo
{

// Define NOT_SET as 255
#define NOT_SET 255

struct BLDCChannelParameters
{
  uint8_t uh;
  uint8_t ul;
  uint8_t vh;
  uint8_t vl;
  uint8_t wh;
  uint8_t wl;
  uint8_t current_u;
  uint8_t current_v;
  uint8_t current_w;
  uint8_t enc_cs;
};

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

typedef union
{
  struct __attribute__((packed))
  {
    float p;
    float i;
    float d;
    float output_ramp;
    float lpf_time_constant;
    float limit;
  };

  uint8_t raw[sizeof(PIDParameters)];
} packed_pid_parameters_t;

struct MotorParameters
{
  int pole_pairs;
  float power_supply_voltage;
  float voltage_limit;
  float current_limit = 1000.0f;
  float velocity_limit = 1000.0f;
  float calibration_voltage;
  bool reversed = false;
};

class MotorChannel
{
 public:
  MotorChannel(BLDCChannelParameters params);

  // Init motors and encoders, calibration is automatically loaded
  void init(MotorParameters params);

  // Init motors and encoders, optionally calibrating
  void init(MotorParameters params, bool should_calibrate);

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