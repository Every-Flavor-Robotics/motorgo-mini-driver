#include "motorgo_mini.h"

#include "Preferences.h"

MotorGo::MotorGoMini::MotorGoMini() : ch0(ch0_params), ch1(ch1_params)
{
  // Initialize the SPI bus for the encoders
  MotorGo::init_encoder_spi();
}
