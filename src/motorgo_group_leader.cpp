#include "motorgo_group_leader.h"

const uint8_t broadcast_address[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

MotorGoGroupLeader::MotorGoGroupLeader()
{
  // Constructor
}

MotorGoGroupLeader::~MotorGoGroupLeader()
{
  // Destructor
}

void MotorGoGroupLeader::init(std::vector<String> device_names)
{
  // Initialize ESP-NOW
  ESPNowComms::init_esp_now();

  // Add all device_names to the map
  for (String device_name : device_names)
  {
    devices[device_name] = ESPNowComms::mac_to_string(broadcast_address);
  }
  enter_discovery_mode();
}

void MotorGoGroupLeader::loop()
{
  if (state == LeaderState::Discovery)
  {
    // Wait until all devices are registered
    // Check this by checking if all registered devices are connected in map
    bool all_registered = true;
    for (auto& device : devices)
    {
      if (!ESPNowComms::get_last_send_success(device.second))
      {
        all_registered = false;
        break;
      }
    }

    if (all_registered)
    {
      // All devices are registered, enter run mode
      enter_run_mode();
    }
  }

  if (state == LeaderState::Run)
  {
    // If latest message is > 1 second ago, send heartbeat
    for (auto& device : devices)
    {
      if (ESPNowComms::get_time_since_last_send(device.second) > 1000)
      {
        send_heartbeat(device.second);
      }
    }
  }
}

void MotorGoGroupLeader::discovery_receive_cb(const uint8_t* mac,
                                              const uint8_t* data, int len)
{
  Serial.println("Received discovery message");
  BeaconPayload* beacon = (BeaconPayload*)data;

  String sender_mac_str = ESPNowComms::mac_to_string(mac);

  //   Check if device is in device_names
  if (devices.find(beacon->device_name) != devices.end())
  {
    Serial.print("Received beacon from ");
    Serial.println(beacon->device_name);

    // Register the new follower
    if (ESPNowComms::register_device(mac))
    {
      // If registration is successful, save the mac address
      devices[beacon->device_name] = sender_mac_str;
    }
  }
}

void MotorGoGroupLeader::run_send_cb(const uint8_t* mac,
                                     esp_now_send_status_t status)
{
}

void MotorGoGroupLeader::enter_discovery_mode()
{
  state = LeaderState::Discovery;

  Serial.println("Entering discovery mode");

  // Set up callbacks correctly for send/receive
  ESPNowComms::set_data_receive_callback(
      [this](const uint8_t* mac, const uint8_t* data, int len)
      { this->discovery_receive_cb(mac, data, len); });
}

void MotorGoGroupLeader::enter_run_mode()
{
  state = LeaderState::Run;

  // Set up callbacks correctly for send/receive
  //   esp_now_register_recv_cb(run_receive_cb);
  esp_now_register_send_cb(run_send_cb);
}

void MotorGoGroupLeader::send_heartbeat(const String mac)
{
  Serial.println("Sending heartbeat");

  // Send heartbeat to device
  HeartbeatMessage heartbeat;

  ESPNowComms::message_t message;
  encode_message(heartbeat, message.data, &message.len);
  Serial.println(message.len);
  Serial.println(message.data[0]);

  ESPNowComms::send_data(mac, message);
}