#include "motorgo_group_leader.h"

std::map<String, DeviceInfo> MotorGoGroupLeader::registered_devices;
std::map<String, String> MotorGoGroupLeader::mac_to_name;

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
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();

  this->device_names = device_names;

  // Initialize ESP-NOW
  if (esp_now_init() != ESP_OK)
  {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  enter_discovery_mode();
}

void MotorGoGroupLeader::loop()
{
  if (state == DISCOVERY)
  {
    // Wait until all devices are registered
    // Check this by checking if all registered devices are connected in map
    bool all_registered = true;
    for (String device_name : device_names)
    {
      // Check if in map and registered
      if (registered_devices.find(device_name) == registered_devices.end() ||
          !registered_devices[device_name].registered)
      {
        all_registered = false;
        break;
      }
    }

    if (all_registered)
    {
      // All devices are registered, enter run mode
      state = RUN;
    }
  }

  if (state == RUN)
  {
    Serial.println("Running");
    delay(500);
  }
}

void MotorGoGroupLeader::register_device(const uint8_t* mac,
                                         const uint8_t* data, int len)
{
  // Device registration logic
}

void MotorGoGroupLeader::discovery_receive_cb(const uint8_t* mac,
                                              const uint8_t* data, int len)
{
  Serial.println("Data received");
  BeaconPayload* beacon = (BeaconPayload*)data;

  char macStr[18];
  snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x", mac[0],
           mac[1], mac[2], mac[3], mac[4], mac[5]);

  Serial.println(macStr);

  String sender_mac_str = macStr;

  // Check if device is already registered
  if (registered_devices.find(sender_mac_str) == registered_devices.end())
  {
    // Register the new follower
    DeviceInfo device_info;
    device_info.registered = true;
    device_info.last_send_time = millis();

    registered_devices[beacon->device_name] = device_info;
    registered_devices[beacon->device_name].mac[0] = mac[0];
    registered_devices[beacon->device_name].mac[1] = mac[1];
    registered_devices[beacon->device_name].mac[2] = mac[2];
    registered_devices[beacon->device_name].mac[3] = mac[3];
    registered_devices[beacon->device_name].mac[4] = mac[4];
    registered_devices[beacon->device_name].mac[5] = mac[5];
    mac_to_name[sender_mac_str] = beacon->device_name;

    Serial.printf("New device registered: %s\n",
                  (const char*)beacon->device_name);

    // Add as peer
    esp_now_peer_info_t peer_info;
    memcpy(peer_info.peer_addr, mac, 6);
    peer_info.encrypt = false;
    peer_info.channel = 0;  // use the current channel
    peer_info.ifidx = WIFI_IF_STA;

    if (esp_now_add_peer(&peer_info) == ESP_OK)
    {
      // Send acknowledgment
      AckPayload ack;
      strncpy(ack.message, "Registered", sizeof(ack.message));
      Serial.println("Attemping to send");
      esp_err_t result = esp_now_send(mac, (uint8_t*)&ack, sizeof(ack));

      if (result == ESP_OK)
      {
        Serial.println("Send command issued");
      }
      else
      {
        Serial.printf("Send command failed with error code: 0x%X\n", result);
      }
    }
    else
    {
      Serial.println("Failed to add peer");
    }
  }
}

void MotorGoGroupLeader::data_send_cb(const uint8_t* mac,
                                      esp_now_send_status_t status)
{
  if (status == ESP_NOW_SEND_SUCCESS)
  {
    // Convert the MAC address bytes to a human-readable string
    String mac_str = "";
    for (int i = 0; i < 6; ++i)
    {
      mac_str += String(mac[i], HEX);
      if (i < 5)
      {
        mac_str += ":";
      }
    }

    // Look up the device name by its MAC address
    if (mac_to_name.find(mac_str) != mac_to_name.end())
    {
      String device_name = mac_to_name[mac_str];

      // Check if the device is registered
      if (registered_devices.find(device_name) != registered_devices.end())
      {
        // Update the last successful send time for the device
        registered_devices[device_name].last_send_time = millis();
      }
    }
  }
}

void MotorGoGroupLeader::send_ack(const uint8_t* mac)
{
  // Send acknowledgment
}

void MotorGoGroupLeader::enter_discovery_mode()
{
  state = DISCOVERY;

  // Set up callbacks correctly for send/receive
  esp_now_register_recv_cb(discovery_receive_cb);
  esp_now_register_send_cb(data_send_cb);
}

// void MotorGoGroupLeader::enter_run_mode()
// {
//   state = RUN;

//   // Set up callbacks correctly for send/receive
//   esp_now_register_recv_cb(run_receive_cb);
//   esp_now_register_send_cb(run_send_cb);
// }
