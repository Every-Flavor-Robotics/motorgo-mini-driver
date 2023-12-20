#include "motorgo_comms_types.h"

void HeartbeatMessage::encode(uint8_t* data) const
{
  // Heartbeat is empty, so nothing to encode
}
void HeartbeatMessage::decode(const uint8_t* data, int len)
{
  // Heartbeat is empty, so nothing to decode
}

int HeartbeatMessage::length() const { return 0; }

template <typename T, typename... Args>
std::unique_ptr<T> make_unique(Args&&... args)
{
  return std::unique_ptr<T>(new T(std::forward<Args>(args)...));
}

BeaconMessage::BeaconMessage() { memset(device_name, 0, sizeof(device_name)); }

int BeaconMessage::length() const { return sizeof(device_name); }

void BeaconMessage::encode(uint8_t* data) const
{
  if (data != nullptr)
  {
    memcpy(data, device_name, sizeof(device_name));
  }
}

void BeaconMessage::decode(const uint8_t* data, int len)
{
  memcpy(device_name, data, sizeof(device_name));
}

void BeaconMessage::set_device_name(std::string name)
{
  // Confirm that the name is not too long
  //   Else, truncate it with a warning
  if (name.length() >= sizeof(device_name))
  {
    Serial.println("Device name too long, truncating");
    name = name.substr(0, sizeof(device_name) - 1);
  }
  // Copy the device name into the beacon message
  memcpy(device_name, name.c_str(), name.length());
}

int CommandMessage::length() const { return sizeof(command) + sizeof(enabled); }

void CommandMessage::encode(uint8_t* data) const
{
  memcpy(data, &command, sizeof(command));
  memcpy(data + sizeof(command), &enabled, sizeof(enabled));
}

void CommandMessage::decode(const uint8_t* data, int len)
{
  memcpy(&command, data, sizeof(command));
  memcpy(&enabled, data + sizeof(command), sizeof(enabled));
}

std::unique_ptr<MessageBase> decode_message(const uint8_t* data, int len)
{
  if (len < 1) return nullptr;  // Invalid message

  uint8_t type = data[0];
  std::unique_ptr<MessageBase> msg;

  switch (type)
  {
    case 0x01:
      msg = make_unique<HeartbeatMessage>();
      break;
    case 0x02:
      msg = make_unique<BeaconMessage>();
      break;
    case 0x03:
      msg = make_unique<CommandMessage>();
      break;
    default:
      Serial.println("Bad message type, exiting");
      return nullptr;  // Unknown type
  }

  msg->decode(data + 1, len - 1);  // Skip the first byte
  return msg;
}

void encode_message(const MessageBase& msg, uint8_t*& output_data, int* len)
{
  *len = msg.length() + 1;

  // Allocate memory for the type byte + data
  output_data = new uint8_t[*len];

  // Set the type
  output_data[0] = msg.type();

  // If message payload is not empty
  if (*len > 1)
  {
    // Encode the message directly into the output_data buffer, after the type
    // byte
    msg.encode(output_data + 1);
  }
}
