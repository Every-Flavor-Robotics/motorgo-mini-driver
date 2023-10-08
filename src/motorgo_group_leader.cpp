#include "motorgo_group_leader.h"

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
      state = LeaderState::Run;
    }
  }

  if (state == LeaderState::Run)
  {
    Serial.println("Running");
    delay(500);
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
    ESPNowComms::register_device(mac);

    devices[beacon->device_name] = sender_mac_str;

    // Send acknowledgment
    AckPayload ack;
    strncpy(ack.message, "Registered", sizeof(ack.message));
    ESPNowComms::message_t message;
    message.data = (uint8_t*)&ack;
    message.len = sizeof(ack);
    ESPNowComms::send_data(mac, message);
  }
}

void MotorGoGroupLeader::data_send_cb(const uint8_t* mac,
                                      esp_now_send_status_t status)
{
  //   if (status == ESP_NOW_SEND_SUCCESS)
  //   {
  //     // Convert the MAC address bytes to a human-readable string
  //     String mac_str = "";
  //     for (int i = 0; i < 6; ++i)
  //     {
  //       mac_str += String(mac[i], HEX);
  //       if (i < 5)
  //       {
  //         mac_str += ":";
  //       }
  //     }

  //     // Look up the device name by its MAC address
  //     if (mac_to_name.find(mac_str) != mac_to_name.end())
  //     {
  //       String device_name = mac_to_name[mac_str];

  //       //   // Check if the device is registered
  //       //   if (registered_devices.find(device_name) !=
  //       registered_devices.end())
  //       //   {
  //       //     // Update the last successful send time for the device
  //       //     registered_devices[device_name].last_send_time = millis();
  //       //   }
  //     }
  //   }
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

// void MotorGoGroupLeader::enter_run_mode()
// {
//   state = RUN;

//   // Set up callbacks correctly for send/receive
//   esp_now_register_recv_cb(run_receive_cb);
//   esp_now_register_send_cb(run_send_cb);
// }
