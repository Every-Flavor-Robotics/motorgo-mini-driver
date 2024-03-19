#include <Arduino.h>

#include "motorgo_mini.h"

MotorGo::MotorGoMini motorgo_mini;
MotorGo::MotorChannel& motor_ch0 = motorgo_mini.ch0;
MotorGo::MotorChannel& motor_ch1 = motorgo_mini.ch1;

MotorGo::ChannelConfiguration config_ch0;
MotorGo::ChannelConfiguration config_ch1;

// Function to print at a maximum frequency
void freq_println(String str, int freq)
{
  static unsigned long last_print_time = 0;
  unsigned long now = millis();

  if (now - last_print_time > 1000 / freq)
  {
    Serial.println(str);
    last_print_time = now;
  }
}

void setup()
{
  Serial.begin(115200);

  // Setup motor parameters
  config_ch0.motor_config = MotorGo::MotorGoGreen;
  config_ch0.power_supply_voltage = 5.0;

  config_ch1.motor_config = MotorGo::MotorGoGreen;
  config_ch1.power_supply_voltage = 5.0;

  // Setup Ch0
  bool calibrate = true;
  motor_ch0.init(config_ch0, calibrate);
  motor_ch1.init(config_ch1, calibrate);
}

void loop()
{
  // Run Ch0
  motor_ch0.loop();
  motor_ch1.loop();

  String str = "Velocity - Ch0: " + String(motor_ch0.get_velocity()) +
               " Ch1: " + String(motor_ch1.get_velocity());

  freq_println(str, 10);
}
