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

void MotorGoGroupLeader::send_message(const String device_name,
                                      const MessageBase& message)
{
  // Send message to device
  ESPNowComms::message_t esp_now_message;
  encode_message(message, esp_now_message.data, &esp_now_message.len);

  ESPNowComms::send_data(devices[device_name], esp_now_message);
}

void MotorGoGroupLeader::discovery_receive_cb(const uint8_t* mac,
                                              const uint8_t* data, int len)
{
  Serial.println("Received discovery message");

  std::unique_ptr<MessageBase> decoded_msg = decode_message(data, len);

  //   Check if nullptr or if message is not a beacon
  if (decoded_msg == nullptr || decoded_msg->type() != 0x02)
  {
    return;
  }

  //   Cast to BeaconMessage
  BeaconMessage* beacon = static_cast<BeaconMessage*>(decoded_msg.get());

  //   Check if device is in device_names
  if (devices.find(beacon->device_name) != devices.end())
  {
    String sender_mac_str = ESPNowComms::mac_to_string(mac);
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

  ESPNowComms::set_data_send_callback(
      [this](const uint8_t* mac, esp_now_send_status_t status)
      { this->run_send_cb(mac, status); });
}

void MotorGoGroupLeader::send_heartbeat(const String mac)
{
  Serial.println("Sending heartbeat");

  // Send heartbeat to device
  HeartbeatMessage heartbeat;

  ESPNowComms::message_t message;
  encode_message(heartbeat, message.data, &message.len);

  ESPNowComms::send_data(mac, message);
}