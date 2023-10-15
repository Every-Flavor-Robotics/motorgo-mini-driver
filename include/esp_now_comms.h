#ifndef ESP_NOW_COMMS
#define ESP_NOW_COMMS

#include <WiFi.h>
#include <esp_now.h>

#include <functional>
#include <map>

#include "motorgo_comms_types.h"

namespace ESPNowComms
{

struct device_info_t
{
  time_t last_send_time;
  bool last_send_success;
  uint8_t mac[6];
};

struct message_t
{
  uint8_t* data;
  int len;
};

static std::map<String, device_info_t> registered_devices;
static bool esp_now_initialized = false;

// Function pointers for user-defined callbacks
// TODO: Might be worth storing a vector of callbacks so that
// Multiple services can register callbacks for the same event
using data_receive_callback_t =
    std::function<void(const uint8_t*, const uint8_t*, int)>;
using data_send_callback_t =
    std::function<void(const uint8_t*, esp_now_send_status_t)>;

static data_receive_callback_t user_data_receive_callback = NULL;
static data_send_callback_t user_data_send_callback = NULL;

void set_data_receive_callback(data_receive_callback_t callback);
void set_data_send_callback(data_send_callback_t callback);

bool init_esp_now();

String mac_to_string(const uint8_t* mac);
void mac_to_cstring(const uint8_t* mac, char* mac_str);

bool register_device(const uint8_t* mac);
bool send_ack(const uint8_t* mac);

bool send_data(const uint8_t* mac, const message_t& message);
bool send_data(const String mac, const message_t& message);

// Send and receive callbacks
void data_send_callback(const uint8_t* mac, esp_now_send_status_t status);
void data_receive_callback(const uint8_t* mac, const uint8_t* data, int len);

time_t get_time_since_last_send(const uint8_t* mac);
time_t get_time_since_last_send(String mac);

bool get_last_send_success(const uint8_t* mac);
bool get_last_send_success(String mac);

const static int LED_PIN = 38;

}  // namespace ESPNowComms

#endif  // ESP_NOW_COMMS