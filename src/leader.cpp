#include <Arduino_JSON.h>
#include <WiFi.h>
#include <esp_now.h>
#include <motorgo_group_leader.h>

#include <map>

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

void loop() { leader.loop(); }