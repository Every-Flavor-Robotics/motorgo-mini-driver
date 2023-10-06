#include "motorgo_groupie.h"

MotorGoGroupie::MotorGoGroupie()
{
  // Constructor
}

MotorGoGroupie::~MotorGoGroupie()
{
  // Destructor
}

void MotorGoGroupie::init(std::string device_name)
{
  this->device_name = device_name;

  WiFi.mode(WIFI_STA);
  WiFi.disconnect();

  // Initialize ESP-NOW
  if (esp_now_init() != ESP_OK)
  {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  enter_discovery_mode();
}

void MotorGoGroupie::loop()
{
  if (state == DISCOVERY)
  {
    BeaconPayload beacon_payload;
    strncpy(beacon_payload.device_name, device_name.c_str(),
            sizeof(beacon_payload.device_name));

    esp_now_send(broadcast_address, (uint8_t*)&beacon_payload,
                 sizeof(beacon_payload));
  }

  if (state == RUN)
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
  AckPayload* ack = (AckPayload*)data;
  if (strncmp(ack->message, "Registered", sizeof(ack->message)) == 0)
  {
    enter_run_mode();
  }
}

void MotorGoGroupie::data_send_cb(const uint8_t* mac,
                                  esp_now_send_status_t status)
{
}

void MotorGoGroupie::send_ack(const uint8_t* mac)
{
  // Send acknowledgment
}

void MotorGoGroupie::enter_discovery_mode()
{
  state = DISCOVERY;

  // Set up callbacks correctly for send/receive
  esp_now_register_recv_cb(discovery_receive_cb);
  esp_now_register_send_cb(data_send_cb);
}

void MotorGoGroupie::enter_run_mode()
{
  state = RUN;

  // TODO: This code does not handle the failure mode where the peer is not
  // added successfully
  esp_now_peer_info_t peer_info;
  memcpy(peer_info.peer_addr, broadcast_address, 6);
  peer_info.encrypt = false;

  if (esp_now_add_peer(&peer_info) == ESP_OK)
  {
  }
  else
  {
    Serial.println("Failed to add peer");
    delay(500);
  }

  // Set up callbacks correctly for send/receive
  esp_now_register_recv_cb(run_receive_cb);
  //   esp_now_register_send_cb(run_send_cb);
}
