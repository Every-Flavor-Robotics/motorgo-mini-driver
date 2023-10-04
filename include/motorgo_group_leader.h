#ifndef MOTOR_GO_GROUP_LEADER_H
#define MOTOR_GO_GROUP_LEADER_H

#include <Arduino_JSON.h>
#include <WiFi.h>
#include <esp_now.h>

#include <chrono>
#include <map>
#include <vector>

#include "motorgo_comms_types.h"

class MotorGoGroupLeader
{
 public:
  MotorGoGroupLeader();
  ~MotorGoGroupLeader();

  // Init takes in a vector of device names to register
  void init(std::vector<String> device_names);
  void loop();
  void register_device(const uint8_t* mac, const uint8_t* data, int len);

 private:
  LEADER_STATE state = UNINITIALIZED;

  // device_names stores the names of the devices to register
  // This only stores the names for this instance of the leader
  std::vector<String> device_names;
  static std::map<String, DeviceInfo> registered_devices;
  static std::map<String, String> mac_to_name;

  static void discovery_receive_cb(const uint8_t* mac, const uint8_t* data,
                                   int len);
  static void data_send_cb(const uint8_t* mac, esp_now_send_status_t status);

  void enter_discovery_mode();

  void send_ack(const uint8_t* mac);
};

#endif  // MOTOR_GO_GROUP_LEADER_H
