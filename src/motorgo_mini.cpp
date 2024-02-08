#include "motorgo_mini.h"

#include "Preferences.h"

MotorGo::MotorGoMini::MotorGoMini()
    : ch0(ch0_params, "ch0"), ch1(ch1_params, "ch1")
{
  // Initialize the SPI bus for the encoders
  MotorGo::init_encoder_spi();
}
