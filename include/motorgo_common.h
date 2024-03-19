// A file that stores common definitions and types used by the MotorGo library.

#ifndef MOTORGO_TYPES_H
#define MOTORGO_TYPES_H

#include <Arduino.h>
#include <SPI.h>

#include "SimpleFOC.h"

namespace MotorGo
{

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
  None,
  Voltage,
  Velocity,
  Torque,
  Position,
  VelocityOpenLoop,
  PositionOpenLoop
};

/**
 * @struct MotorConfiguration
 * @brief Structure that stores the motor configuration.
 *
 * These parameters describe the motor's electrical and mechanical properties,
 * such as the number of pole pairs, the motor's velocity constant (kv), phase
 * resistance, and phase inductance. The limits characterize the safe
 * operating range of the motor.
 */
struct MotorConfiguration
{
  int pole_pairs = -1;
  float kv = NOT_SET;
  float phase_resistance = NOT_SET;
  float phase_inductance = NOT_SET;
  float voltage_limit = 1000.0f;
  float current_limit = 1000.0f;
  float velocity_limit = 1000.0f;
  float calibration_voltage = NOT_SET;

  // Default constructor
  MotorConfiguration()
      : pole_pairs(-1),
        kv(NOT_SET),
        phase_resistance(NOT_SET),
        phase_inductance(NOT_SET),
        voltage_limit(1000.0f),
        current_limit(1000.0f),
        velocity_limit(1000.0f),
        calibration_voltage(NOT_SET)
  {
  }

  // Constructor with parameters
  MotorConfiguration(int pole_pairs, float kv, float phase_resistance,
                     float phase_inductance, float voltage_limit,
                     float current_limit, float velocity_limit,
                     float calibration_voltage)
      : pole_pairs(pole_pairs),
        kv(kv),
        phase_resistance(phase_resistance),
        phase_inductance(phase_inductance),
        voltage_limit(voltage_limit),
        current_limit(current_limit),
        velocity_limit(velocity_limit),
        calibration_voltage(calibration_voltage)
  {
  }
};

/**
 * @struct ChannelConfiguration
 * @brief Structure that stores the configuration for setting up a motor
 * channel. This includes the motor configuration, the power supply voltage,
 * and whether the motor should be reversed. If reversed is false, the motor
 * will rotate counter-clockwise with a positive command. If reversed is true,
 * the motor will rotate clockwise with a positive command.
 */
struct ChannelConfiguration
{
  MotorConfiguration motor_config;
  float power_supply_voltage;
  float reversed;
};

/**
 * @struct PIDParameters
 * @brief Structure holding PID controller parameters.
 *
 * This structure encapsulates the proportional (P), integral (I), and
 * derivative (D) parameters used in PID control, along with additional
 * parameters such as output ramp rate, low pass filter time constant, and
 * limit for the output.
 */
struct PIDParameters
{
  float p;
  float i;
  float d;
  float output_ramp = 10000.0f;
  float lpf_time_constant = 0.1f;
  float limit = 10000.0f;
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
    float p;
    float i;
    float d;
    float output_ramp;
    float lpf_time_constant;
    float limit;
  };

  uint8_t raw[sizeof(PIDParameters)];
} packed_pid_parameters_t;

/**
 * @var hspi
 * @brief Global instance of SPIClass for SPI communication.
 *
 * Used for SPI communication with peripherals.
 */
extern SPIClass hspi;

/**
 * @brief Initializes the SPI interface for communication with the two
 *        encoders on the MotorGo Mini.
 */
void init_encoder_spi();

}  // namespace MotorGo

#endif  // MOTORGO_TYPES_H