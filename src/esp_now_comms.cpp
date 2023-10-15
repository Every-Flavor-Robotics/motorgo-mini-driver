#include "esp_now_comms.h"

bool ESPNowComms::init_esp_now()
{
  pinMode(ESPNowComms::LED_PIN, OUTPUT);

  if (esp_now_initialized)
  {
    return true;
  }

  WiFi.mode(WIFI_STA);
  WiFi.disconnect();

  bool success = esp_now_init() == ESP_OK;

  // Register callbacks if init successful
  if (success)
  {
    esp_now_register_send_cb(ESPNowComms::data_send_callback);
    esp_now_register_recv_cb(ESPNowComms::data_receive_callback);
  }

  return success;
}

bool ESPNowComms::send_data(const uint8_t* mac, const message_t& message)
{
  // Send data to device
  esp_err_t result = esp_now_send(mac, message.data, message.len);
  return result == ESP_OK;
}

bool ESPNowComms::send_data(const String mac, const message_t& message)
{
  // Look up mac address in registered devices
  if (ESPNowComms::registered_devices.find(mac) ==
      ESPNowComms::registered_devices.end())
  {
    Serial.println("Device not registered");
    return false;
  }
  // Send data to device
  esp_err_t result =
      esp_now_send(registered_devices[mac].mac, message.data, message.len);
  return result == ESP_OK;
}

void ESPNowComms::set_data_receive_callback(data_receive_callback_t callback)
{
  ESPNowComms::user_data_receive_callback = callback;
}

void ESPNowComms::set_data_send_callback(data_send_callback_t callback)
{
  ESPNowComms::user_data_send_callback = callback;
}

String ESPNowComms::mac_to_string(const uint8_t* mac)
{
  char macStr[18];
  snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x", mac[0],
           mac[1], mac[2], mac[3], mac[4], mac[5]);
  return String(macStr);
}

void ESPNowComms::mac_to_cstring(const uint8_t* mac, char* mac_str)
{
  snprintf(mac_str, 18, "%02x:%02x:%02x:%02x:%02x:%02x", mac[0], mac[1], mac[2],
           mac[3], mac[4], mac[5]);
}

bool ESPNowComms::register_device(const uint8_t* mac)
{
  String mac_str = ESPNowComms::mac_to_string(mac);

  // Device registration logic
  if (ESPNowComms::registered_devices.find(mac_str) ==
      ESPNowComms::registered_devices.end())
  {
    esp_now_peer_info_t peer_info;
    memcpy(peer_info.peer_addr, mac, 6);
    peer_info.encrypt = false;
    peer_info.channel = 0;
    peer_info.ifidx = WIFI_IF_STA;

    if (esp_now_add_peer(&peer_info) != ESP_OK)
    {
      Serial.println("Failed to add peer");
      return false;
    }

    // Register device
    ESPNowComms::registered_devices[mac_str].last_send_time = 0;
    // Copy mac address into registered_devices
    memcpy(ESPNowComms::registered_devices[mac_str].mac, mac, 6);

    bool send_success = ESPNowComms::send_ack(mac);
  }
  else
  {
    // Device already registered
    Serial.println("Device already registered");
    return false;
  }
  return true;
}

bool ESPNowComms::send_ack(const uint8_t* mac)
{
  // Send ack to device
  esp_err_t result = esp_now_send(mac, (uint8_t*)"ack", 4);
  return result == ESP_OK;
}

// Send and Receive Callbacks
void ESPNowComms::data_send_callback(const uint8_t* mac,
                                     esp_now_send_status_t status)
{
  // Blink LED
  digitalWrite(ESPNowComms::LED_PIN, HIGH);
  // Delay microsecond
  delayMicroseconds(50);
  // Turn off LED
  digitalWrite(ESPNowComms::LED_PIN, LOW);

  String mac_str = ESPNowComms::mac_to_string(mac);
  // Check if send status is successful
  if (status == ESP_NOW_SEND_SUCCESS)
  {
    ESPNowComms::registered_devices[mac_str].last_send_success = true;
    ESPNowComms::registered_devices[mac_str].last_send_time = millis();
  }
  else
  {
    ESPNowComms::registered_devices[mac_str].last_send_success = false;
  }

  // Call user defined callback if it exists
  if (ESPNowComms::user_data_send_callback != NULL)
  {
    ESPNowComms::user_data_send_callback(mac, status);
  }
}

void ESPNowComms::data_receive_callback(const uint8_t* mac, const uint8_t* data,
                                        int len)
{
  // Blink LED
  digitalWrite(ESPNowComms::LED_PIN, HIGH);
  // Delay microsecond
  delayMicroseconds(50);
  // Turn off LED
  digitalWrite(ESPNowComms::LED_PIN, LOW);

  //   Serial.println("ESP Now data received");

  // Call user defined callback if it exists
  if (ESPNowComms::user_data_receive_callback != NULL)
  {
    ESPNowComms::user_data_receive_callback(mac, data, len);
  }
}

time_t ESPNowComms::get_time_since_last_send(String mac)
{
  return millis() - ESPNowComms::registered_devices[mac].last_send_time;
}

time_t ESPNowComms::get_time_since_last_send(const uint8_t* mac)
{
  String mac_str = ESPNowComms::mac_to_string(mac);
  return ESPNowComms::get_time_since_last_send(mac_str);
}

bool ESPNowComms::get_last_send_success(String mac)
{
  return ESPNowComms::registered_devices[mac].last_send_success;
}

bool ESPNowComms::get_last_send_success(const uint8_t* mac)
{
  String mac_str = ESPNowComms::mac_to_string(mac);
  return ESPNowComms::get_last_send_success(mac_str);
}