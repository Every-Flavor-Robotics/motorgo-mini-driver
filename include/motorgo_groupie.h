#ifndef MOTOR_GO_GROUPIE_H
#define MOTOR_GO_GROUPIE_H

#include <WiFi.h>
#include <esp_now.h>

#include <chrono>
#include <map>
#include <vector>

#include "esp_now_comms.h"
#include "motorgo_comms_types.h"

class MotorGoGroupie
{
 public:
  MotorGoGroupie();
  //   ~MotorGoGroupie();

  // Init takes in a vector of device names to register
  void init(std::string device_name);
  void loop();
  void register_device(const uint8_t* mac, const uint8_t* data, int len);

 private:
  std::string device_name;

  GroupieState state = GroupieState::Uninitialized;

  void discovery_receive_cb(const uint8_t* mac, const uint8_t* data, int len);

  static void run_receive_cb(const uint8_t* mac, const uint8_t* data, int len);

  static void data_send_cb(const uint8_t* mac, esp_now_send_status_t status);

  void enter_discovery_mode();
  void enter_run_mode();

  void send_ack(const uint8_t* mac);
};

#endif  // MOTOR_GO_GROUPIE_H
