#include <WiFi.h>
// #include <esp_now.h>

#include <motorgo_groupie.h>

MotorGoGroupie groupie;

void setup()
{
  Serial.begin(115200);

  delay(3000);

  Serial.println("Starting up groupie");

  pinMode(8, OUTPUT);

  digitalWrite(8, HIGH);

  groupie.init("Follower_1");
}

void loop() { groupie.loop(); }
