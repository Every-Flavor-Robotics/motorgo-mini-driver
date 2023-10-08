#ifndef MOTOR_GO_COMM_TYPES_H
#define MOTOR_GO_COMM_TYPES_H

// typedef for time as long
typedef long time_t;

// Broadcast address
static uint8_t broadcast_address[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

struct DeviceInfo
{
  bool registered;
  time_t last_send_time;
  uint8_t mac[6];
};

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
