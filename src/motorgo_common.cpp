#include "motorgo_common.h"

// Encoder I2C bus, define and initialize here to avoid multiple definitions
SPIClass MotorGo::hspi = SPIClass(HSPI);
bool hspi_initialized = false;

void MotorGo::init_encoder_spi()
{
  //   Guard to prevent multiple initializations, which could cause a crash
  if (!hspi_initialized)
  {
    hspi_initialized = true;
    MotorGo::hspi.begin(ENC_SCL, ENC_SDA, ENC_MOSI);
  }
}