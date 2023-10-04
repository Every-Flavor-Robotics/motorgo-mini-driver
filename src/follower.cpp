#include <WiFi.h>
#include <esp_now.h>

bool peer_added = false;
bool ack_received = false;

struct BeaconPayload
{
  char device_name[20];
};

struct AckPayload
{
  char message[20];
};

// Callback when data is received
void data_receive_cb(const uint8_t *sender_mac, const uint8_t *data, int len)
{
  AckPayload *ack = (AckPayload *)data;
  if (strncmp(ack->message, "Registered", sizeof(ack->message)) == 0)
  {
    ack_received = true;
    Serial.println("Acknowledgment received");
  }
}

void setup()
{
  Serial.begin(115200);

  WiFi.mode(WIFI_STA);
  WiFi.disconnect();

  if (esp_now_init() != ESP_OK)
  {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  esp_now_register_recv_cb(data_receive_cb);

  esp_now_register_send_cb(
      [](const uint8_t *mac_addr, esp_now_send_status_t status)
      {
        Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Send success"
                                                      : "Send failure");
      });
}

void loop()
{
  uint8_t broadcast_address[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

  if (!peer_added)
  {
    esp_now_peer_info_t peer_info;
    memcpy(peer_info.peer_addr, broadcast_address, 6);
    peer_info.encrypt = false;

    if (esp_now_add_peer(&peer_info) == ESP_OK)
    {
      peer_added = true;
    }
    else
    {
      Serial.println("Failed to add peer");
      delay(500);
      return;
    }
  }

  if (!ack_received)
  {
    BeaconPayload beacon_payload;
    strncpy(beacon_payload.device_name, "Follower_1",
            sizeof(beacon_payload.device_name));

    delay(1000);
    esp_now_send(broadcast_address, (uint8_t *)&beacon_payload,
                 sizeof(beacon_payload));
  }

  delay(1000);
}
