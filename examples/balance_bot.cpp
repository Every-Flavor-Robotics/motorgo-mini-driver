#include <Adafruit_MPU6050.h>
#include <Arduino.h>

#include "Wire.h"
#include "configurable.h"
#include "motorgo_mini.h"
#include "readable.h"
#include "web_server.h"

MotorGo::MotorGoMini* motorgo_mini;
MotorGo::MotorParameters motor_params_ch0;
MotorGo::MotorParameters motor_params_ch1;

// instantiate pid motorgo pid params
MotorGo::PIDParameters current_pid_params_ch0;
MotorGo::PIDParameters current_pid_params_ch1;

MotorGo::PIDParameters velocity_pid_params_ch0;
MotorGo::PIDParameters velocity_pid_params_ch1;

MotorGo::PIDParameters position_pid_params_ch0;
MotorGo::PIDParameters position_pid_params_ch1;

Adafruit_MPU6050 mpu;

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

  //   Configure Wire on pins 8, 38
  Wire.begin(8, 38);

  //   Begin IMU on Wire
  if (!mpu.begin(MPU6050_I2CADDR_DEFAULT, &Wire))
  {
    Serial.println("Failed to find MPU6050 chip");
    while (1)
    {
      delay(10);
    }
  }

  delay(3000);

  // Setup motor parameters
  motor_params_ch0.pole_pairs = 7;
  motor_params_ch0.power_supply_voltage = 5.0;
  motor_params_ch0.voltage_limit = 5.0;
  motor_params_ch0.current_limit = 300;
  motor_params_ch0.velocity_limit = 100.0;
  motor_params_ch0.calibration_voltage = 2.0;

  motor_params_ch1.pole_pairs = 7;
  motor_params_ch1.power_supply_voltage = 5.0;
  motor_params_ch1.voltage_limit = 5.0;
  motor_params_ch1.current_limit = 300;
  motor_params_ch1.velocity_limit = 100.0;
  motor_params_ch1.calibration_voltage = 2.0;

  // Instantiate motorgo mini board
  motorgo_mini = new MotorGo::MotorGoMini();

  // Setup Ch0 with FOCStudio enabled
  bool calibrate = false;
  bool enable_foc_studio = false;
  motorgo_mini->init_ch0(motor_params_ch0, calibrate, enable_foc_studio);
  motorgo_mini->init_ch1(motor_params_ch1, calibrate, enable_foc_studio);

  // Set velocity controller parameters
  // Setup PID parameters - velocity

  float vel_p = 0.5;
  float vel_i = 0.0;
  float vel_d = 0.0;

  velocity_pid_params_ch0.p = vel_p;
  velocity_pid_params_ch0.i = vel_i;
  velocity_pid_params_ch0.d = vel_d;
  velocity_pid_params_ch0.output_ramp = 10000.0;
  velocity_pid_params_ch0.lpf_time_constant = 0.11;

  velocity_pid_params_ch1.p = vel_p;
  velocity_pid_params_ch1.i = vel_i;
  velocity_pid_params_ch1.d = vel_d;
  velocity_pid_params_ch1.output_ramp = 10000.0;
  velocity_pid_params_ch1.lpf_time_constant = 0.11;

  // Setup PID parameters - position
  // set up p controller only for position control.
  float pos_p = 5.0;
  float pos_i = 0.5;
  float pos_d = 0.0;

  position_pid_params_ch0.p = pos_p;
  position_pid_params_ch0.i = pos_i;
  position_pid_params_ch0.d = pos_d;
  position_pid_params_ch0.output_ramp = 10000.0;
  position_pid_params_ch0.lpf_time_constant = 0.11;

  position_pid_params_ch1.p = pos_p;
  position_pid_params_ch1.i = pos_i;
  position_pid_params_ch1.d = pos_d;
  position_pid_params_ch1.output_ramp = 10000.0;
  position_pid_params_ch1.lpf_time_constant = 0.11;

  // Instantiate controllers
  motorgo_mini->set_velocity_controller_ch0(velocity_pid_params_ch0);
  motorgo_mini->set_velocity_controller_ch1(velocity_pid_params_ch1);

  motorgo_mini->set_position_controller_ch0(position_pid_params_ch0);
  motorgo_mini->set_position_controller_ch1(position_pid_params_ch1);

  //   Set closed-loop position mode
  motorgo_mini->set_control_mode_ch0(MotorGo::ControlMode::Position);
  motorgo_mini->set_control_mode_ch1(MotorGo::ControlMode::Position);

  //   Print url: http://{IP_ADDRESS}:PORT
  Serial.print("Please connect to http://");
  Serial.print(WiFi.localIP());
  Serial.print(":");
  Serial.println(8080);

  // enable controllers and prepare for the loop
  //   motorgo_mini->enable_ch0();
  //   motorgo_mini->enable_ch1();
}

void loop()
{
  // Print IMU data
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  Serial.println("IMU Data:");
  Serial.print("aX = ");
  Serial.print(a.acceleration.x);
  Serial.print(" aY = ");
  Serial.print(a.acceleration.y);
  Serial.print(" aZ = ");
  Serial.print(a.acceleration.z);

  Serial.print(" gX = ");
  Serial.print(g.gyro.x);
  Serial.print(" gY = ");
  Serial.print(g.gyro.y);
  Serial.print(" gZ = ");
  Serial.print(g.gyro.z);

  Serial.print(" t = ");
  Serial.println(temp.temperature);

  delay(100);

  // Run Ch0
  //   motorgo_mini->loop_ch0();
  //   motorgo_mini->loop_ch1();

  // measure positions
  //   float ch0_pos = motorgo_mini->get_ch0_position();
  //   float ch1_pos = motorgo_mini->get_ch1_position();

  //   // set target positions between each motor
  //   motorgo_mini->set_target_position_ch0(ch1_pos);
  //   motorgo_mini->set_target_position_ch1(ch0_pos);

  //   String x = "ch0 pos: " + String(ch0_pos);
  //   String y = "ch1 pos: " + String(ch1_pos);
}