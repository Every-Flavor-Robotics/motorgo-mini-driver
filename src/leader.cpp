#include <Arduino_JSON.h>
#include <WiFi.h>
#include <esp_now.h>
#include <motorgo_group_leader.h>

MotorGoGroupLeader leader;

void setup()
{
  Serial.begin(115200);

  delay(2000);

  Serial.println("Starting up leader");

  // Create array of strings of device names
  std::vector<String> device_names;
  device_names.push_back("Follower_1");

  pinMode(8, OUTPUT);

  digitalWrite(8, HIGH);

  leader.init(device_names);
}

void loop()
{
  leader.loop();

  CommandMessage message;
  message.command = 10;
  message.enabled = false;

  leader.send_message("Follower_1", message);

  delay(100);
}