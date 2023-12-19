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
    // Add more cases as needed
    default:
      Serial.println("Bad message type, exiting");
      return nullptr;  // Unknown type
  }

  msg->decode(data + 1, len - 1);  // Skip the first byte
  return msg;
}

void encode_message(const MessageBase& msg, uint8_t*& output_data, int* len)
{
  uint8_t* data = nullptr;
  msg.encode(data);
  *len = msg.length();

  // Allocate memory for the type byte + data
  output_data = new uint8_t[*len + 1];

  // Set the type and copy the data
  output_data[0] = msg.type();

  if (*len > 0)
  {
    memcpy(output_data + 1, data, *len);
  }

  delete[] data;

  // Adjust len to include the type byte
  (*len)++;
}
