// Header fill to store configurations for the official MotorGo motors

#ifndef MOTORGO_MOTORS_H
#define MOTORGO_MOTORS_H

#include "motorgo_common.h"

namespace MotorGo
{

/**
 * @brief Configuration for the official MotorGo Green motor. This is a
 * PTZ 2804 motor.
 *
 */
const MotorConfiguration MotorGoGreen(7,         // pole_pairs
                                      320.0f,    // kv
                                      2.3f,      // phase_resistance
                                      NOT_SET,   // phase_inductance
                                      12.0f,     // voltage_limit
                                      2.5f,      // current_limit
                                      10000.0f,  // velocity_limit
                                      1.0f       // calibration_voltage
);
}  // namespace MotorGo
#endif  // MOTORGO_MOTORS_H