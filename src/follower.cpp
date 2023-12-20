#include <WiFi.h>
// #include <esp_now.h>

#include <motorgo_groupie.h>

MotorGoGroupie groupie;

// Function to print at a maximum frequency
void freq_println(String str, int freq)
{
  static unsigned long last_print_time = 0;
  unsigned long now = millis();

  if (now - last_print_time > 1000 / freq)
  {
    Serial.println(str);
    last_print_time = now;
  }
}

void setup()
{
  Serial.begin(115200);

  delay(3000);

  Serial.println("Starting up groupie");

  pinMode(8, OUTPUT);

  digitalWrite(8, HIGH);

  groupie.init("Follower_1");
}

void loop()
{
  groupie.loop();

  //   Craft a string that is "Command: {command}, Enabled: {enabled}"
  String str = "Command: ";
  str += String(groupie.get_command());
  str += ", Enabled: ";
  str += String(groupie.get_enabled());

  freq_println(str, 10);
}
