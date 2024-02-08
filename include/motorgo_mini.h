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
      .uh = CH0_UH,
      .ul = CH0_UL,
      .vh = CH0_VH,
      .vl = CH0_VL,
      .wh = CH0_WH,
      .wl = CH0_WL,
      .current_u = CH0_CURRENT_U,
      .current_v = CH0_CURRENT_V,
      .current_w = CH0_CURRENT_W,
      .enc_cs = CH0_ENC_CS,
  };

  BLDCChannelParameters ch1_params = {
      .uh = CH1_UH,
      .ul = CH1_UL,
      .vh = CH1_VH,
      .vl = CH1_VL,
      .wh = CH1_WH,
      .wl = CH1_WL,
      .current_u = CH1_CURRENT_U,
      .current_v = CH1_CURRENT_V,
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
