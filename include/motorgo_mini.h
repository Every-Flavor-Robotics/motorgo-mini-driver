/**
 * @file MotorGoMini.h
 * @brief Header file for the MotorGo Mini driver class.
 *
 * This file provides the declarations for the MotorGo Mini driver. It includes
 * necessary libraries for SPI communication, SimpleFOC for motor control, and
 * Wire for I2C communication. Additionally, it includes headers for calibrated
 * sensors and MT6701 magnetic sensors specific to the MotorGo Mini hardware.
 */

#ifndef MOTORGO_MINI_H
#define MOTORGO_MINI_H

#include <SPI.h>        ///< Include SPI library for serial communication.
#include <SimpleFOC.h>  ///< Include SimpleFOC library for motor control.
#include <Wire.h>       ///< Include Wire library for I2C communication.

// Including specific encoder and sensor classes for the MotorGo Mini
#include "encoders/calibrated/CalibratedSensor.h"  ///< Include header for calibrated sensors.
#include "encoders/mt6701/MagneticSensorMT6701SSI.h"  ///< Include header for MT6701 magnetic sensor.

namespace MotorGo
{

/**
 * @enum ControlMode
 * @brief Enumerates the control modes for motor operation.
 *
 * This enumeration defines various modes of operation for the motor, such as
 * voltage control, velocity control, torque control, position control, and
 * their respective open-loop variants.
 */
enum ControlMode
{
  None,              ///< No control mode specified.
  Voltage,           ///< Voltage control mode.
  Velocity,          ///< Velocity control mode.
  Torque,            ///< Torque control mode.
  Position,          ///< Position control mode.
  VelocityOpenLoop,  ///< Open-loop velocity control.
  PositionOpenLoop   ///< Open-loop position control.
};

/**
 * @struct PIDParameters
 * @brief Structure holding PID controller parameters.
 *
 * This structure encapsulates the proportional (P), integral (I), and
 * derivative (D) parameters used in PID control, along with additional
 * parameters such as output ramp rate, low pass filter time constant, and limit
 * for the output.
 */
struct PIDParameters
{
  float p;                         ///< Proportional gain.
  float i;                         ///< Integral gain.
  float d;                         ///< Derivative gain.
  float output_ramp = 10000.0f;    ///< Ramp rate for output.
  float lpf_time_constant = 0.1f;  ///< Low pass filter time constant.
  float limit = 10000.0f;          ///< Limit for the PID output.
};

/**
 * @union packed_pid_parameters_t
 * @brief Union representing PID parameters in packed and raw byte formats.
 *
 * This union allows PIDParameters to be accessed either as a struct with
 * individual fields or as a raw byte array, facilitating easy storage or
 * transmission.
 */
typedef union
{
  struct __attribute__((packed))
  {
    float p;                  ///< Proportional gain.
    float i;                  ///< Integral gain.
    float d;                  ///< Derivative gain.
    float output_ramp;        ///< Ramp rate for output.
    float lpf_time_constant;  ///< Low pass filter time constant.
    float limit;              ///< Limit for the PID output.
  };

  uint8_t raw[sizeof(
      PIDParameters)];  ///< Raw byte array representation of PIDParameters.
} packed_pid_parameters_t;

/**
 * @struct MotorParameters
 * @brief Structure to store parameters for motor configuration.
 *
 * Encapsulates various parameters required for setting up a motor, such as the
 * number of pole pairs, power supply voltage, limits on voltage and current,
 * and calibration voltage.
 */
struct MotorParameters
{
  int pole_pairs;                  ///< Number of pole pairs in the motor.
  float power_supply_voltage;      ///< Voltage of the power supply.
  float voltage_limit;             ///< Maximum allowable voltage.
  float current_limit = 1000.0f;   ///< Maximum allowable current.
  float velocity_limit = 1000.0f;  ///< Maximum allowable velocity.
  float calibration_voltage;       ///< Voltage used for calibration.
};

// Global variables documentation
/**
 * @var command
 * @brief Global Commander instance for motor control commands.
 *
 * This instance of Commander is used to interpret and execute motor control
 * commands. Note: It is a global variable due to limitations in the SimpleFOC
 * commander API.
 */
extern Commander command;

/**
 * @var motor_ch0
 * @brief Global instance of BLDCMotor for channel 0.
 *
 * Represents the BLDC motor connected to channel 0.
 */
extern BLDCMotor motor_ch0;

/**
 * @var motor_ch1
 * @brief Global instance of BLDCMotor for channel 1.
 *
 * Represents the BLDC motor connected to channel 1.
 */
extern BLDCMotor motor_ch1;

/**
 * @var hspi
 * @brief Global instance of SPIClass for SPI communication.
 *
 * Used for SPI communication with peripherals.
 */
extern SPIClass hspi;

/**
 * @class MotorGoMini
 * @brief Provides control and management for MotorGo Mini motors.
 *
 * This class encapsulates the functionality for controlling and managing
 * BLDC motors using the MotorGo Mini hardware. It offers methods for motor
 * initialization, control loop execution, PID controller management, motor
 * enabling/disabling, setting control modes and targets, and saving/loading
 * controller parameters.
 */
class MotorGoMini
{
 public:
  MotorGoMini();

  /**
   * @defgroup motor_initialization Motor Initialization
   * @brief Functions for initializing motors and encoders with various
   * settings.
   * @{
   */

  /**
   * @brief Initializes motor and encoder on channel 0 with default settings.
   *        Calibration is automatically loaded, and FOCStudio is disabled.
   * @param params MotorParameters structure containing motor setup parameters.
   */
  void init_ch0(MotorParameters params);

  /**
   * @brief Initializes motor and encoder on channel 1 with default settings.
   *        Calibration is automatically loaded, and FOCStudio is disabled.
   * @param params MotorParameters structure containing motor setup parameters.
   */
  void init_ch1(MotorParameters params);

  /**
   * @brief Initializes motor and encoder on channel 0 with additional options.
   *        Allows for optional calibration and enabling of FOCStudio.
   * @param params MotorParameters structure containing motor setup parameters.
   * @param should_calibrate If true, performs calibration on startup.
   * @param enable_foc_studio If true, enables FOCStudio communication.
   */
  void init_ch0(MotorParameters params, bool should_calibrate,
                bool enable_foc_studio);

  /**
   * @brief Initializes motor and encoder on channel 1 with additional options.
   *        Allows for optional calibration and enabling of FOCStudio.
   * @param params MotorParameters structure containing motor setup parameters.
   * @param should_calibrate If true, performs calibration on startup.
   * @param enable_foc_studio If true, enables FOCStudio communication.
   */
  void init_ch1(MotorParameters params, bool should_calibrate,
                bool enable_foc_studio);

  /** @} */  // end of motor_initialization group

  /**
   * @defgroup control_loop Control Loop
   * @brief Functions to run the control loop for each motor.
   * @{
   */

  /**
   * @brief Runs the control loop for the motor on channel 0.
   */
  void loop_ch0();

  /**
   * @brief Runs the control loop for the motor on channel 1.
   */
  void loop_ch1();

  /** @} */  // end of control_loop group

  /**
   * @defgroup pid_controller_management PID Controller Management
   * @brief Functions for managing PID controllers, including getting and
   * setting parameters.
   * @{
   */

  /**
   * @brief Retrieves the PID parameters for the torque controller of channel 0
   * motor.
   * @return PIDParameters structure containing the current settings of the
   * torque controller for channel 0.
   */
  PIDParameters get_torque_controller_ch0();

  /**
   * @brief Retrieves the PID parameters for the torque controller of channel 1
   * motor.
   * @return PIDParameters structure containing the current settings of the
   * torque controller for channel 1.
   */
  PIDParameters get_torque_controller_ch1();

  /**
   * @brief Retrieves the PID parameters for the velocity controller of channel
   * 0 motor.
   * @return PIDParameters structure containing the current settings of the
   * velocity controller for channel 0.
   */
  PIDParameters get_velocity_controller_ch0();

  /**
   * @brief Retrieves the PID parameters for the velocity controller of channel
   * 1 motor.
   * @return PIDParameters structure containing the current settings of the
   * velocity controller for channel 1.
   */
  PIDParameters get_velocity_controller_ch1();

  /**
   * @brief Retrieves the PID parameters for the position controller of channel
   * 0 motor.
   * @return PIDParameters structure containing the current settings of the
   * position controller for channel 0.
   */
  PIDParameters get_position_controller_ch0();

  /**
   * @brief Retrieves the PID parameters for the position controller of channel
   * 1 motor.
   * @return PIDParameters structure containing the current settings of the
   * position controller for channel 1.
   */
  PIDParameters get_position_controller_ch1();

  /**
   * @brief Sets the PID parameters for the torque controller of channel 0
   * motor.
   * @param params The PIDParameters structure to configure the torque
   * controller for channel 0.
   */
  void set_torque_controller_ch0(PIDParameters params);

  /**
   * @brief Sets the PID parameters for the torque controller of channel 1
   * motor.
   * @param params The PIDParameters structure to configure the torque
   * controller for channel 1.
   */
  void set_torque_controller_ch1(PIDParameters params);

  /**
   * @brief Sets the PID parameters for the velocity controller of channel 0
   * motor.
   * @param params The PIDParameters structure to configure the velocity
   * controller for channel 0.
   */
  void set_velocity_controller_ch0(PIDParameters params);

  /**
   * @brief Sets the PID parameters for the velocity controller of channel 1
   * motor.
   * @param params The PIDParameters structure to configure the velocity
   * controller for channel 1.
   */
  void set_velocity_controller_ch1(PIDParameters params);

  /**
   * @brief Sets the PID parameters for the position controller of channel 0
   * motor.
   * @param params The PIDParameters structure to configure the position
   * controller for channel 0.
   */
  void set_position_controller_ch0(PIDParameters params);

  /**
   * @brief Sets the PID parameters for the position controller of channel 1
   * motor.
   * @param params The PIDParameters structure to configure the position
   * controller for channel 1.
   */
  void set_position_controller_ch1(PIDParameters params);

  /**
   * @brief Resets the torque controller parameters to default for channel 0
   * motor.
   */
  void reset_torque_controller_ch0();

  /**
   * @brief Resets the torque controller parameters to default for channel 1
   * motor.
   */
  void reset_torque_controller_ch1();

  /**
   * @brief Resets the velocity controller parameters to default for channel 0
   * motor.
   */
  void reset_velocity_controller_ch0();

  /**
   * @brief Resets the velocity controller parameters to default for channel 1
   * motor.
   */
  void reset_velocity_controller_ch1();

  /**
   * @brief Resets the position controller parameters to default for channel 0
   * motor.
   */
  void reset_position_controller_ch0();

  /**
   * @brief Resets the position controller parameters to default for channel 1
   * motor.
   */
  void reset_position_controller_ch1();

  /**
   * @brief Saves the current torque controller settings for channel 0 to
   * memory.
   */
  void save_torque_controller_ch0();

  /**
   * @brief Saves the current torque controller settings for channel 1 to
   * memory.
   */
  void save_torque_controller_ch1();

  /**
   * @brief Loads the torque controller settings for channel 0 from memory.
   */
  void load_torque_controller_ch0();

  /**
   * @brief Loads the torque controller settings for channel 1 from memory.
   */
  void load_torque_controller_ch1();

  /**
   * @brief Saves the current velocity controller settings for channel 0 to
   * memory.
   */
  void save_velocity_controller_ch0();

  /**
   * @brief Saves the current velocity controller settings for channel 1 to
   * memory.
   */
  void save_velocity_controller_ch1();

  /**
   * @brief Loads the velocity controller settings for channel 0 from memory.
   */
  void load_velocity_controller_ch0();

  /**
   * @brief Loads the velocity controller settings for channel 1 from memory.
   */
  void load_velocity_controller_ch1();

  /**
   * @brief Saves the current position controller settings for channel 0 to
   * memory.
   */
  void save_position_controller_ch0();

  /**
   * @brief Saves the current position controller settings for channel 1 to
   * memory.
   */
  void save_position_controller_ch1();

  /**
   * @brief Loads the position controller settings for channel 0 from memory.
   */
  void load_position_controller_ch0();

  /**
   * @brief Loads the position controller settings for channel 1 from memory.
   */
  void load_position_controller_ch1();

  /** @} */  // end of pid_controller_management group

  /**
   * @defgroup motor_command Motor Command
   * @brief Functions for basic motor command operations like enable, disable,
   * and setting control modes.
   * @{
   */

  /**
   * @brief Enables the motor on channel 0, allowing it to operate.
   */
  void enable_ch0();

  /**
   * @brief Enables the motor on channel 1, allowing it to operate.
   */
  void enable_ch1();

  /**
   * @brief Disables the motor on channel 0, preventing it from operating.
   */
  void disable_ch0();

  /**
   * @brief Disables the motor on channel 1, preventing it from operating.
   */
  void disable_ch1();

  /**
   * @brief Sets the control mode for the motor on channel 0.
   * @param mode The desired ControlMode (e.g., Velocity, Position) for the
   * motor on channel 0.
   */
  void set_control_mode_ch0(ControlMode mode);

  /**
   * @brief Sets the control mode for the motor on channel 1.
   * @param mode The desired ControlMode (e.g., Velocity, Position) for the
   * motor on channel 1.
   */
  void set_control_mode_ch1(ControlMode mode);

  /**
   * @brief Sets the target torque for the motor on channel 0.
   * @param target The desired target torque in N*m for the motor on channel 0.
   */
  void set_target_torque_ch0(float target);

  /**
   * @brief Sets the target torque for the motor on channel 1.
   * @param target The desired target torque in N*m for the motor on channel 1.
   */
  void set_target_torque_ch1(float target);

  /**
   * @brief Sets the target velocity for the motor on channel 0.
   * @param target The desired target velocity in rad/s for the motor on channel
   * 0.
   */
  void set_target_velocity_ch0(float target);

  /**
   * @brief Sets the target velocity for the motor on channel 1.
   * @param target The desired target velocity in rad/s for the motor on
   * channel 1.
   */
  void set_target_velocity_ch1(float target);

  /**
   * @brief Sets the target position for the motor on channel 0.
   * @param target The desired target position in radians for the motor on
   * channel 0.
   */
  void set_target_position_ch0(float target);

  /**
   * @brief Sets the target position for the motor on channel 1.
   * @param target The desired target position in radians for the motor on
   * channel 1.
   */
  void set_target_position_ch1(float target);

  /**
   * @brief Sets the target voltage for the motor on channel 0.
   * @param target The desired target voltage in volts for the motor on channel
   * 0.
   */
  void set_target_voltage_ch0(float target);

  /**
   * @brief Sets the target voltage for the motor on channel 1.
   * @param target The desired target voltage in volts for the motor on
   * channel 1.
   */
  void set_target_voltage_ch1(float target);

  /** @} */  // end of motor_control group

  /**
   * @defgroup state_retrieval State Retrieval
   * @brief Functions for retrieving the current state of the motors.
   * @{
   */

  /**
   * @brief Sets the current position of the motor on channel 0 to zero.
   *        This function resets the motor's position value to zero.
   */
  void zero_position_ch0();

  /**
   * @brief Sets the current position of the motor on channel 1 to zero.
   *        This function resets the motor's position value to zero.
   */
  void zero_position_ch1();

  /**
   * @brief Retrieves the current position of the motor on channel 0.
   * @return The motor's position in radians.
   */
  float get_ch0_position();

  /**
   * @brief Retrieves the current velocity of the motor on channel 0.
   * @return The motor's velocity in rad/s.
   */
  float get_ch0_velocity();

  /**
   * @brief Retrieves the current torque of the motor on channel 0.
   * @return The motor's torque in N*m.
   */
  float get_ch0_torque();

  /**
   * @brief Retrieves the current voltage applied to the motor on channel 0.
   * @return The motor's voltage in volts.
   */
  float get_ch0_voltage();

  /**
   * @brief Retrieves the current position of the motor on channel 1.
   * @return The motor's position in radians.
   */
  float get_ch1_position();

  /**
   * @brief Retrieves the current velocity of the motor on channel 1.
   * @return The motor's velocity in rad/s.
   */
  float get_ch1_velocity();

  /**
   * @brief Retrieves the current torque of the motor on channel 1.
   * @return The motor's torque in N*m.
   */
  float get_ch1_torque();

  /**
   * @brief Retrieves the current voltage applied to the motor on channel 1.
   * @return The motor's voltage in volts.
   */
  float get_ch1_voltage();

  /** @} */  // end of state_retrieval group

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
  const int k_ch1_current_u = 8;
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

  void save_controller_helper(const char* key,
                              const packed_pid_parameters_t& packed_params);
  packed_pid_parameters_t load_controller_helper(const char* key);
  void set_torque_controller_helper(BLDCMotor& motor, PIDParameters params);
  void set_velocity_controller_helper(BLDCMotor& motor, PIDParameters params);
  void set_position_controller_helper(BLDCMotor& motor, PIDParameters params);
};
}  // namespace MotorGo

#endif  // MOTORGO_MINI_H
