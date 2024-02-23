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
 * @struct MotorParameters
 * @brief Structure to store parameters for motor configuration.
 *
 * Encapsulates various parameters required for setting up a motor, such as the
 * number of pole pairs, power supply voltage, limits on voltage and current,
 * and calibration voltage.
 */struct MotorParameters
{
  int pole_pairs;
  float kv = NOT_SET;
  float phase_resistance = NOT_SET;
  float phase_inductance = NOT_SET;

  float power_supply_voltage;
  float voltage_limit = 1000.0f;
  float current_limit = 1000.0f;
  float velocity_limit = 1000.0f;
  float calibration_voltage;
  bool reversed = false;
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