// Header file for the MotorGo Mini driver class.

#ifndef MOTORGO_MINI_H
#define MOTORGO_MINI_H

#include "motorgo_channel.h"
#include "motorgo_common.h"

namespace MotorGo
{

class MotorGoMini
{
 private:
  BLDCChannelParameters ch0_params = {
      .uh = CH0_GPIO_UH,
      .ul = CH0_GPIO_UL,
      .vh = CH0_GPIO_VH,
      .vl = CH0_GPIO_VL,
      .wh = CH0_GPIO_WH,
      .wl = CH0_GPIO_WL,
      .current_u = CH0_CURRENT_U,
      .current_v = GPIO_NOT_SET,
      .current_w = CH0_CURRENT_W,
      .enc_cs = CH0_ENC_CS,
  };

  BLDCChannelParameters ch1_params = {
      .uh = CH1_GPIO_UH,
      .ul = CH1_GPIO_UL,
      .vh = CH1_GPIO_VH,
      .vl = CH1_GPIO_VL,
      .wh = CH1_GPIO_WH,
      .wl = CH1_GPIO_WL,
      .current_u = CH1_CURRENT_U,
      .current_v = GPIO_NOT_SET,
      .current_w = CH1_CURRENT_W,
      .enc_cs = CH1_ENC_CS,
  };

 public:
  MotorGoMini();

  // Define the two motor channels on the MotorGo Mini
  MotorChannel ch0;
  MotorChannel ch1;
};
}  // namespace MotorGo

#endif  // MOTORGO_MINI_H
