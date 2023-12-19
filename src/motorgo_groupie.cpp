#include "motorgo_groupie.h"

const uint8_t broadcast_address[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

MotorGoGroupie::MotorGoGroupie()
{
  // Constructor
}

void MotorGoGroupie::init(std::string device_name)
{
  this->device_name = device_name;

  ESPNowComms::init_esp_now();

  MotorGoGroupie::enter_discovery_mode();

  ESPNowComms::register_device(broadcast_address);
}

void MotorGoGroupie::loop()
{
  if (state == GroupieState::Discovery)
  {
    BeaconPayload beacon_payload;
    strncpy(beacon_payload.device_name, device_name.c_str(),
            sizeof(beacon_payload.device_name));

    ESPNowComms::message_t message;
    message.data = (uint8_t*)&beacon_payload;
    message.len = sizeof(beacon_payload);

    Serial.println("Sending beacon");

    ESPNowComms::send_data(broadcast_address, message);

    delay(100);
  }

  if (state == GroupieState::Run)
  {
    Serial.println("Running");
    delay(500);
  }
}

void MotorGoGroupie::register_device(const uint8_t* mac, const uint8_t* data,
                                     int len)
{
  // Device registration logic
}

void MotorGoGroupie::discovery_receive_cb(const uint8_t* mac,
                                          const uint8_t* data, int len)
{
  Serial.println(len);
  std::unique_ptr<MessageBase> decoded_msg = decode_message(data, len);

  //   Check if nullptr
  if (decoded_msg == nullptr)
  {
    return;
  }
  if (decoded_msg->type() == 0x01)
  {
    // Heartbeat message
    Serial.println("Received heartbeat");
    Serial.println("Entering run mode");
    enter_run_mode();
  }
}

void MotorGoGroupie::run_receive_cb(const uint8_t* mac, const uint8_t* data,
                                    int len)
{
  std::unique_ptr<MessageBase> decoded_msg = decode_message(data, len);

  if (decoded_msg->type() == 0x01)
  {
    // Heartbeat message
    Serial.println("Received heartbeat in run");
  }
}

void MotorGoGroupie::data_send_cb(const uint8_t* mac,
                                  esp_now_send_status_t status)
{
}

void MotorGoGroupie::enter_discovery_mode()
{
  state = GroupieState::Discovery;

  // Set up callbacks correctly for send/receive
  ESPNowComms::set_data_receive_callback(
      [this](const uint8_t* mac, const uint8_t* data, int len)
      { this->discovery_receive_cb(mac, data, len); });

  ESPNowComms::set_data_send_callback(
      [](const uint8_t* mac_addr, esp_now_send_status_t status)
      {
        Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Send success"
                                                      : "Send failure");
      });
}

void MotorGoGroupie::enter_run_mode()
{
  state = GroupieState::Run;

  // Set up callbacks correctly for send/receive
  ESPNowComms::set_data_receive_callback(
      [this](const uint8_t* mac, const uint8_t* data, int len)
      { this->run_receive_cb(mac, data, len); });

  // TODO: This code does not handle the failure mode where the peer is not
  // added successfully
  //   esp_now_peer_info_t peer_info;
  //   memcpy(peer_info.peer_addr, broadcast_address, 6);
  //   peer_info.encrypt = false;

  //   if (esp_now_add_peer(&peer_info) == ESP_OK)
  //   {
  //   }
  //   else
  //   {
  //     Serial.println("Failed to add peer");
  //     delay(500);
  //   }

  // Set up callbacks correctly for send/receive
  //   esp_now_register_recv_cb(run_receive_cb);
  //   esp_now_register_send_cb(run_send_cb);
}
