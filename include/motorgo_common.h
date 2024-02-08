// A file that stores common definitions and types used by the MotorGo library.

#ifndef MOTORGO_TYPES_H
#define MOTORGO_TYPES_H

#include <Arduino.h>
#include <SPI.h>

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

// Control Mode
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

// Motor Parameters
struct MotorParameters
{
  int pole_pairs;
  float power_supply_voltage;
  float voltage_limit = 1000.0f;
  float current_limit = 1000.0f;
  float velocity_limit = 1000.0f;
  float calibration_voltage;
  bool reversed = false;
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

extern SPIClass hspi;
void init_encoder_spi();

}  // namespace MotorGo

#endif  // MOTORGO_TYPES_H