#ifndef MOTOR_GO_COMM_TYPES_H
#define MOTOR_GO_COMM_TYPES_H

// typedef for time as long
typedef long time_t;

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

enum LEADER_STATE
{
  DISCOVERY,
  RUN,
  WAIT_FOR_RECONNECT,
  UNINITIALIZED
};

enum GROUPIE_STATE
{
  DISCOVERY,
  RUN,
  UNINITIALIZED
};

#endif  // MOTOR_GO_COMM_TYPES_H
