#include <Arduino_JSON.h>
#include <WiFi.h>
#include <esp_now.h>
#include <motorgo_group_leader.h>

#include <map>

// To store MAC address and other info about the follower
std::map<String, bool> registered_devices;

// Callback when data is received
void data_receive_cb(const uint8_t *sender_mac, const uint8_t *data, int len)
{
  Serial.println("Data received");
  BeaconPayload *beacon = (BeaconPayload *)data;

  char macStr[18];
  snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
           sender_mac[0], sender_mac[1], sender_mac[2], sender_mac[3],
           sender_mac[4], sender_mac[5]);

  Serial.println(macStr);

  String sender_mac_str = macStr;

  // Create JSON object to parse received data
  JSONVar json_obj;
  json_obj["device_name"] = beacon->device_name;

  // Check if device is already registered
  if (registered_devices.find(sender_mac_str) == registered_devices.end())
  {
    // Register the new follower
    registered_devices[sender_mac_str] = true;
    Serial.printf("New device registered: %s\n",
                  (const char *)json_obj["device_name"]);

    // Add as peer
    esp_now_peer_info_t peer_info;
    memcpy(peer_info.peer_addr, sender_mac, 6);
    peer_info.encrypt = false;
    peer_info.channel = 0;  // use the current channel
    peer_info.ifidx = WIFI_IF_STA;

    if (esp_now_add_peer(&peer_info) == ESP_OK)
    {
      // Send acknowledgment
      AckPayload ack;
      strncpy(ack.message, "Registered", sizeof(ack.message));
      Serial.println("Attemping to send");
      esp_err_t result = esp_now_send(sender_mac, (uint8_t *)&ack, sizeof(ack));

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

MotorGoGroupLeader leader;

void setup()
{
  Serial.begin(115200);

  delay(4000);

  // Create array of strings of device names
  std::vector<String> device_names;
  device_names.push_back("Follower_1");

  leader.init(device_names);
}

void loop() { leader.loop(); }