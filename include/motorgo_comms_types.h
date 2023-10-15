#ifndef MOTOR_GO_COMM_TYPES_H
#define MOTOR_GO_COMM_TYPES_H

#include <memory>

#include "Arduino.h"

// typedef for time as long
typedef long time_t;

// Broadcast address
// static uint8_t broadcast_address[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

struct DeviceInfo
{
  bool registered;
  time_t last_send_time;
  uint8_t mac[6];
};

class MessageBase
{
 public:
  virtual uint8_t type() const = 0;
  virtual void encode(uint8_t* data) const = 0;
  virtual void decode(const uint8_t* data, int len) = 0;
  virtual int length() const = 0;
  virtual ~MessageBase() {}
};

class HeartbeatMessage : public MessageBase
{
 public:
  uint8_t type() const override { return 0x01; }
  HeartbeatMessage() {}
  HeartbeatMessage(const uint8_t* data, int len) { decode(data, len); }

  void encode(uint8_t* data) const override;

  void decode(const uint8_t* data, int len) override;

  int length() const override;
};

template <typename T, typename... Args>
std::unique_ptr<T> make_unique(Args&&... args);

std::unique_ptr<MessageBase> decode_message(const uint8_t* data, int len);

void encode_message(const MessageBase& msg, uint8_t*& output_data, int* len);

struct BeaconPayload
{
  char device_name[20];
};

struct AckPayload
{
  char message[20];
};

enum class LeaderState
{
  Discovery,
  Run,
  WaitForReconnect,
  Uninitialized
};

enum class GroupieState
{
  Discovery,
  Run,
  Uninitialized
};

#endif  // MOTOR_GO_COMM_TYPES_H
