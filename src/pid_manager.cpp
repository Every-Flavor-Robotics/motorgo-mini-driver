#include "pid_manager.h"

#include "web_server.h"

MotorGo::PIDManagerConfig::PIDManagerConfig(const String& name,
                                            PIDParameters& params,
                                            std::function<void()> callback)
    : name(name),
      params(params),
      update_controller_callback(callback),
      configurable(this->params, "/pid" + name, "PID Controller")
{
}

MotorGo::PIDManagerConfig::PIDManagerConfig(const PIDManagerConfig& other)
    : name(other.name),
      params(other.params),  // Copying the reference
      update_controller_callback(other.update_controller_callback),
      configurable(other.configurable)
{
}

MotorGo::PIDManager::PIDManager() {}

void MotorGo::PIDManager::add_controller(
    String name, PIDParameters& params,
    std::function<void()> update_controller_callback)
{
  PIDManagerConfig config(name, params, update_controller_callback);
  configs.push_back(config);
}

void MotorGo::PIDManager::init(String wifi_ssid, String wifi_password)
{
  Serial.println("Initializing PIDManager");
  for (auto& config : configs)
  {
    config.configurable.set_post_callback(
        [this, &config](PIDParameters params)
        { config.update_controller_callback(); });

    Serial.println(config.configurable.get_endpoint());
  }

  generate_schema();

  schema_readable = ESPWifiConfig::Readable<JSONVar>(
      [this]() { return schema; }, "/pid/schema",
      "Description of available PID controllers");

  ESPWifiConfig::WebServer::getInstance().start(wifi_ssid, wifi_password,
                                                LED_STATUS);

  //   Display URL to access webserver. http://{ip_address}:{port}
  Serial.println("Webserver available at:");
  Serial.print("http://");
  Serial.print(ESPWifiConfig::WebServer::getInstance().get_ip_address());
  Serial.print(":");
  Serial.println(ESPWifiConfig::WebServer::getInstance().get_port());
}

void MotorGo::PIDManager::generate_schema()
{
  // Generate API schema
  // A list of controller endpoints, each with an associated description

  schema = JSONVar();

  for (size_t i = 0; i < configs.size(); i++)
  {
    JSONVar endpoint;
    endpoint["endpoint"] = configs[i].configurable.get_endpoint();
    endpoint["name"] = configs[i].name;
    schema[i] = endpoint;
  }
}