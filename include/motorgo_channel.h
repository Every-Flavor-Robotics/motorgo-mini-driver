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

/**
 * @class MotorChannel
 * @brief Provides control and management for each of the MotorGo brushless
 *        DC (BLDC) motor channels.
 *
 * This class encapsulates the functionality for controlling and managing
 * BLDC motors using the MotorGo hardware. It provides functions for setting
 * up motors and encoders, running control loops, and setting control modes.
 */
class MotorChannel
{
 public:
  /**
   * @brief Constructor for the MotorChannel class.
   * @param params BLDCChannelParameters structure containing the pin
   * configuration for the motor channel.
   * @param name The name of the motor channel, used for saving calibration
   * parameters to EEPROM.
   */
  MotorChannel(BLDCChannelParameters params, const char* name);
  MotorChannel(const MotorChannel&) = delete;  // Delete copy constructor
  MotorChannel& operator=(const MotorChannel&) =
      delete;  // Delete copy assignment operator

  /**
   * @name Motor Initialization
   * @brief Functions for initializing motors and encoders with various
   * settings.
   * @{
   */

  /**
  * @brief Initializes motor and encoder with default settings. Calibration
           is automatically loaded.
  * @param channel_config ChannelConfiguration structure containing the
  * configuration for the motor channel.
  */
  void init(ChannelConfiguration channel_config);

  /**
   * @brief Initializes motor and encoder with the option of calibration.
   *        If should_calibrate is false, calibration procedure will not run.
   *        Calibration will be loaded from flash if available, else the
   *        channel will only run open loop control modes
   * @param channel_config ChannelConfiguration structure containing the
   * configuration for the motor channel.
   * @param should_calibrate If true, performs calibration on startup.
   */
  void init(ChannelConfiguration channel_config, bool should_calibrate);

  /** @} */  // end of motor_initialization group

  /**
   * @brief Runs the control loop for the motor channel and updates encoder
   *        data. This should be run as fast as possible, without delays.
   */
  void loop();

  /**
   * @name PID Controller Management
   * @brief Functions for managing PID controllers, including getting and
   * setting parameters.
   * @{
   */

  /**
   * @brief Retrieves the PID parameters for the torque controller
   * motor.
   * @return PIDParameters structure containing the current settings of the
   * torque controller.
   */
  PIDParameters get_torque_controller();

  /**
   * @brief Retrieves the PID parameters for the velocity controller.
   * @return PIDParameters structure containing the current settings of the
   * velocity controller.
   */
  PIDParameters get_velocity_controller();

  /**
   * @brief Retrieves the PID parameters for the position controller.
   * @return PIDParameters structure containing the current settings of the
   * position controller.
   */
  PIDParameters get_position_controller();

  /**
   * @brief Sets the PID parameters for the torque controller.
   * @param params The PIDParameters structure to configure the torque
   * controller.
   */
  void set_torque_controller(PIDParameters params);

  /**
   * @brief Sets the PID parameters for the velocity controller.
   * @param params The PIDParameters structure to configure the velocity
   * controller.
   */
  void set_velocity_controller(PIDParameters params);

  /**
   * @brief Sets the PID parameters for the position controller.
   * @param params The PIDParameters structure to configure the position
   * controller.
   */
  void set_position_controller(PIDParameters params);

  /**
   * @brief Get the user-specified torque limit.
   *
   * @return float
   */
  float get_torque_limit();

  /**
   * @brief Get the user-specified velocity limit.
   *
   * @return float
   */
  float get_velocity_limit();

  /**
   * @brief Get the user-specified lower position limit.
   *
   * @return float
   */
  float get_position_limit_low();

  /**
   * @brief Get the user-specified upper position limit.
   *
   * @return float
   */
  float get_position_limit_high();

  /**
   * @brief Get the user-specified voltage limit.
   *
   * @return float
   */
  float get_voltage_limit();

  /**
   * @brief Sets the maximum torque that can be commanded to the torque
   * controller.
   * @param limit The maximum torque in N*m that can be commanded to the motor.
   * @note This limit is only enforced if the motor is in torque control mode.
   */
  void set_torque_limit(float limit);

  /**
   * @brief Sets the maximum velocity that can be commanded to the velocity
   * controller.
   * @param limit The maximum velocity in rad/s that can be commanded to the
   * motor.
   * @note This limit is only enforced if the motor is in velocity control mode.
   */
  void set_velocity_limit(float limit);

  /**
   * @brief Sets the position limits for the position controller.
   * @param low The lower position limit in radians.
   * @param high The upper position limit in radians.
   * @note This limit is only enforced if the motor is in position control mode.
   */
  void set_position_limit(float low, float high);

  /**
   * @brief Sets the maximum voltage that can be commanded to the motor.
   * @param limit The maximum voltage in V that can be commanded to the motor.
   * @note This limit is only enforced if the motor is in voltage control mode.
   */
  void set_voltage_limit(float limit);

  /**
   * @brief Resets the internal state of the torque controller. This clears
   *        the integral term and sets the output to zero.
   */
  void reset_torque_controller();

  /**
   * @brief Resets the internal state of the velocity controller. This clears
   *        the integral term and sets the output to zero.
   */
  void reset_velocity_controller();

  /**
   * @brief Resets the internal state of the position controller. This clears
   *        the integral term and sets the output to zero.
   */
  void reset_position_controller();

  /**
   * @brief Saves the current torque controller settings to memory.
   */
  void save_torque_controller();

  /**
   * @brief Saves the current velocity controller settings to memory.
   */
  void save_velocity_controller();

  /**
   * @brief Saves the current position controller settings to memory.
   */
  void save_position_controller();

  /**
   * @brief Loads the torque controller settings from memory.
   */
  void load_torque_controller();

  /**
   * @brief Loads the velocity controller settings from memory.
   */
  void load_velocity_controller();

  /**
   * @brief Loads the position controller settings from memory.
   */
  void load_position_controller();

  /** @} */  // end of pid_controller_management group

  /**
   * @name Motor Command
   * @brief Functions for basic motor command operations like enable, disable,
   * and setting control modes.
   * @{
   */

  /**
   * @brief Enables the motor, it will run the currently set command.
   */
  void enable();

  /**
   * @brief Disables the motor, it will stop running and ignore commands.
   */
  void disable();

  /**
   * @brief Sets the control mode for the motor.
   * @param mode The desired ControlMode (e.g., Velocity, Position) for the
   * motor.
   */
  void set_control_mode(ControlMode mode);

  /**
   * @brief Sets the target torque for the motor.
   * @param target The desired target torque in N*m for the motor.
   */
  void set_target_torque(float target);

  /**
   * @brief Sets the target velocity for the motor.
   * @param target The desired target velocity in rad/s for the motor.
   */
  void set_target_velocity(float target);

  /**
   * @brief Sets the target position for the motor.
   * @param target The desired target position in rad for the motor.
   */
  void set_target_position(float target);

  /**
   * @brief Sets the target voltage for the motor.
   * @param target The desired target voltage in V for the motor.
   */
  void set_target_voltage(float target);

  /** @} */  // end of motor_command group

  /**
   * @name State Retrieval
   * @brief Functions for retrieving the current state of the motors.
   * @{
   */

  /**
   * @brief Sets the current position of the motor to zero.
   *        This function resets the motor's position value to zero.
   */
  void zero_position();

  /**
   * @brief Retrieves the current torque of the motor.
   * @return The motor's torque in N*m.
   */
  float get_torque();

  /**
   * @brief Retrieves the current velocity of the motor.
   * @return The motor's velocity in rad/s.
   */
  float get_velocity();

  /**
   * @brief Retrieves the current position of the motor.
   * @return The motor's position in radians.
   */
  float get_position();

  /**
   * @brief Retrieves the current voltage of the motor.
   * @return The motor's voltage in V.
   */
  float get_voltage();

  /** @} */  // end of state_retrieval group

 private:
  //    Motor name
  //   Used to store calibration parameters in EEPROM
  const char* name;

  // Encoder, motor, and driver instances
  BLDCMotor motor;
  BLDCDriver6PWM driver;
  MagneticSensorMT6701SSI encoder;
  //   Calibrated sensor stores the calibration parameters
  CalibratedSensor sensor_calibrated;

  // Additional motor and encoder parameters
  ChannelConfiguration channel_config;
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
  bool velocity_limit_enabled = false;
  float velocity_limit = 0.0f;

  // N*m
  float target_torque = 0.0f;
  bool torque_limit_enabled = false;
  float torque_limit = 0.0f;

  // Rad
  float target_position = 0.0f;
  bool position_limit_enabled = false;
  float position_limit_low = 0.0f;
  float position_limit_high = 0.0f;

  // V
  float target_voltage = 0.0f;
  bool voltage_limit_enabled = false;
  float voltage_limit = 10000.0f;

  // MotorGo Limits
  // MotorGo Mini driver voltage limit
  // TODO: Some board definitions include the voltage limit, others don't
  // Set a default for now thats safe for the released hardware
#ifndef DRIVER_VOLTAGE_LIMIT
  const float DRIVER_VOLTAGE_LIMIT = 11.0f;
#endif

  // MotorGo Mini driver current limit
#ifndef DRIVER_CURRENT_LIMIT
  const float DRIVER_CURRENT_LIMIT = 1.8f;
#endif

  // Calibration parameters
  // If should_calibrate is set to true, the motor will be calibrated on startup
  // Else, the calibration will be loaded from EEPROM. If no calibration is
  // found, the motor will be calibrated anyway and the calibration will be
  // saved to EEPROM
  bool should_calibrate;
  bool calibration_loaded = false;

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