#include <Arduino.h>

#include "Wire.h"
#include "common/lowpass_filter.h"
#include "common/pid.h"
#include "configurable.h"
#include "motorgo_mini.h"
#include "pid_manager.h"
#include "sensorgo_mpu6050.h"

// UPDATE THESE VALUES
String WIFI_SSID = "YOUR_SSID";
String WIFI_PASSWORD = "YOUR_PASSWORD";

MotorGo::MotorGoMini motorgo_mini;
MotorGo::MotorChannel& motor_left = motorgo_mini.ch0;
MotorGo::MotorChannel& motor_right = motorgo_mini.ch1;

MotorGo::MotorParameters motor_params_left;
MotorGo::MotorParameters motor_params_right;

// declare PID manager object
MotorGo::PIDManager pid_manager;

// declare and configure custom velocity controller object
MotorGo::PIDParameters velocity_controller_params;
LowPassFilter velocity_lpf(velocity_controller_params.lpf_time_constant);
PIDController velocity_controller(velocity_controller_params.p,
                                  velocity_controller_params.i,
                                  velocity_controller_params.d,
                                  velocity_controller_params.output_ramp,
                                  velocity_controller_params.limit);

// declare and configure custom balance controller object
MotorGo::PIDParameters balancing_controller_params;
LowPassFilter balancing_lpf(balancing_controller_params.lpf_time_constant);
PIDController balancing_controller(balancing_controller_params.p,
                                   balancing_controller_params.i,
                                   balancing_controller_params.d,
                                   balancing_controller_params.output_ramp,
                                   balancing_controller_params.limit);

// declare and configure custom steering controller object
MotorGo::PIDParameters steering_controller_params;
LowPassFilter steering_lpf(steering_controller_params.lpf_time_constant);
PIDController steering_controller(steering_controller_params.p,
                                  steering_controller_params.i,
                                  steering_controller_params.d,
                                  steering_controller_params.output_ramp,
                                  steering_controller_params.limit);

// declare a balance point controller, where p is the balance point.
// NOTE: THIS IS NOT A PID CONTROLLER. This hack lets you set the balance point
// of your robot over wifi without needing to re-flash the code.
// declare and configure custom balance setpoint controller object
// to use this hack, make sure to
// 1) instantiate this controller in setup
// 2) update the balance point in the control loop
MotorGo::PIDParameters balance_point_params;
LowPassFilter balance_point_lpf(balance_point_params.lpf_time_constant);
PIDController balance_point_controller(balance_point_params.p,
                                       balance_point_params.i,
                                       balance_point_params.d,
                                       balance_point_params.output_ramp,
                                       balance_point_params.limit);

// configure wifi communications
bool motors_enabled = false;
ESPWifiConfig::Configurable<bool> enable_motors(motors_enabled, "/enable",
                                                "Enable motors");

// declare IMU, initial zero pitch (balance point), and steering initial angles
SensorGoMPU6050 mpu;
float pitch_zero = 0.0;
float initial_angle_left;
float initial_angle_right;

void enable_motors_callback(bool value)
{
  if (value)
  {
    Serial.println("Enabling motors");
    motor_left.enable();
    motor_right.enable();
  }
  else
  {
    Serial.println("Disabling motors");
    motor_left.disable();
    motor_right.disable();
  }
}

// Helper function to print at a maximum frequency
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

  //   Configure Wire on pins 38, 47
  Wire1.begin(38, 47);

  //   Begin IMU on Wire
  if (!mpu.begin(&Wire1))
  {
    Serial.println("------------- ERROR ---------------");
    Serial.println("Failed to find MPU6050 chip");
    //   Restart the board
    Serial.println("Restarting to try again...");
    Serial.println("------------------------------------");
    ESP.restart();
  }

  // prepare to calibrate IMU
  Serial.println("Calibrating IMU in 3 seconds, do not move robot");
  Serial.println("3...");
  delay(1000);
  Serial.println("2...");
  delay(1000);
  Serial.println("1...");
  delay(1000);
  Serial.println("Calibrating IMU...");
  mpu.calibrate();

  // Setup motor parameters
  motor_params_left.pole_pairs = 7;
  motor_params_left.power_supply_voltage = 5.0;
  motor_params_left.voltage_limit = 5.0;
  motor_params_left.current_limit = 300;
  motor_params_left.velocity_limit = 100.0;
  motor_params_left.calibration_voltage = 2.0;

  motor_params_right.pole_pairs = 7;
  motor_params_right.power_supply_voltage = 5.0;
  motor_params_right.voltage_limit = 5.0;
  motor_params_right.current_limit = 300;
  motor_params_right.velocity_limit = 100.0;
  motor_params_right.calibration_voltage = 2.0;

  // Setup both motor channels
  bool calibrate = false;
  motor_left.init(motor_params_left, calibrate);
  motor_right.init(motor_params_right, calibrate);

  // Set motor control mode to voltage control
  motor_left.set_control_mode(MotorGo::ControlMode::Voltage);
  motor_right.set_control_mode(MotorGo::ControlMode::Voltage);

  // wrap controller params into a configurable object, pass anonymous function
  // to allow board to update controller values after receiving input over wifi.
  pid_manager.add_controller(
      "/velocity", velocity_controller_params,
      []()
      {
        velocity_controller.P = velocity_controller_params.p;
        velocity_controller.I = velocity_controller_params.i;
        velocity_controller.D = velocity_controller_params.d;
        velocity_controller.output_ramp =
            velocity_controller_params.output_ramp;
        velocity_controller.limit = velocity_controller_params.limit;
        velocity_lpf.Tf = velocity_controller_params.lpf_time_constant;
        velocity_controller.reset();
      });

  // wrap controller params into a configurable object, pass anonymous function
  // to allow board to update controller values after receiving input over wifi.
  pid_manager.add_controller(
      "/balancing", balancing_controller_params,
      []()
      {
        balancing_controller.P = balancing_controller_params.p;
        balancing_controller.I = balancing_controller_params.i;
        balancing_controller.D = balancing_controller_params.d;
        balancing_controller.output_ramp =
            balancing_controller_params.output_ramp;
        balancing_controller.limit = balancing_controller_params.limit;
        balancing_lpf.Tf = balancing_controller_params.lpf_time_constant;
        balancing_controller.reset();
      });

  // wrap controller params into a configurable object, pass anonymous function
  // to allow board to update controller values after receiving input over wifi.
  pid_manager.add_controller(
      "/steering", steering_controller_params,
      []()
      {
        if (motors_enabled)
        {
          motor_left.disable();
          motor_right.disable();
        }

        // Compute new initial angle
        initial_angle_left = motor_left.get_position();
        initial_angle_right = motor_right.get_position();

        steering_controller.P = steering_controller_params.p;
        steering_controller.I = steering_controller_params.i;
        steering_controller.D = steering_controller_params.d;
        steering_controller.output_ramp =
            steering_controller_params.output_ramp;
        steering_controller.limit = steering_controller_params.limit;
        steering_lpf.Tf = steering_controller_params.lpf_time_constant;
        steering_controller.reset();

        if (motors_enabled)
        {
          motor_left.enable();
          motor_right.enable();
        }
      });

  // instantiate a balance point controller, where p is the balance point.
  // NOTE: THIS IS NOT A PID CONTROLLER. This hack lets you set the balance
  // point of your robot over wifi without needing to re-flash the code. to use
  // this hack, make sure to 1) declare the controller above beefore setup 2)
  // update the balance point of the robot in the loop
  pid_manager.add_controller(
      "/balance point", balance_point_params,
      []()
      {
        balance_point_controller.P = balance_point_params.p;
        balance_point_controller.I = balance_point_params.i;
        balance_point_controller.D = balance_point_params.d;
        balance_point_controller.output_ramp = balance_point_params.output_ramp;
        balance_point_controller.limit = balance_point_params.limit;
        balancing_lpf.Tf = balance_point_params.lpf_time_constant;
        balance_point_controller.reset();
      });

  enable_motors.set_post_callback(enable_motors_callback);

  // initialize the PID manager
  pid_manager.init(WIFI_SSID, WIFI_PASSWORD);

  // update steering initial positions to current motor positions.
  // steering controller will try to hold this steering state until steering
  // controller is updated.
  initial_angle_left = motor_left.get_position();
  initial_angle_right = motor_right.get_position();

  // Normally, we'd enable the motors here. However, since the GUI can enable,
  // leave them disabled so the robot doesn't run away
  //   motorgo_mini->enable_ch0();
  //   motorgo_mini->enable_ch1();
}

void loop()
{
  // Only update if new data is ready from IMU
  // Else, just keep running the controllers
  if (mpu.data_ready())
  {
    //  Roll is actually pitch for the balance bot IMU
    float pitch = mpu.get_pitch();

    // Optional print statment: print pitch using frequency print
    freq_println("Pitch: " + String(pitch, 5), 10);

    // calculate average wheel velocity for the velocity controller
    float wheel_velocity =
        (motor_left.get_velocity() + motor_right.get_velocity()) / 2;

    // read motor positions against initial measured angle - for steering
    // controller
    float ch0_pos = motor_left.get_position() - initial_angle_left;
    float ch1_pos = motor_right.get_position() - initial_angle_right;

    // calculate control signals by calling each PID controller
    // each controller function call aims to make the input term 0 with the
    // command value calculated. Commands are calculated based on the PID gains
    // and time constant specified in the web app.
    float velocity_command = velocity_controller(wheel_velocity);
    float balance_command = balancing_controller(pitch - pitch_zero);
    float steering_command = steering_controller(ch0_pos - ch1_pos);

    // update zero pitch with balance point p
    // NOTE: THIS IS NOT A PID CONTROLLER. This hack lets you set the balance
    // point of your robot over wifi without needing to re-flash the code. To
    // use this hack, 1) declare the controller before setup 2) instantiate the
    // controller in setup 3) uncomment the line below:
    pitch_zero = balance_point_controller.P;

    // Optional print statement: balance_command
    // freq_println("Balancing error: " + String(pitch - pitch_zero, 5), 10);

    // Combine controller outputs to compute motor commands
    float command_ch0 = balance_command - steering_command - velocity_command;
    float command_ch1 = balance_command + steering_command - velocity_command;

    // optional print statement for each wheel position
    // freq_println("ch0_pos: "+String(ch0_pos) + " | ch1_pos:
    // "+String(ch1_pos), 10);

    // Set target voltage for each motor
    motor_left.set_target_voltage(command_ch0);
    motor_right.set_target_voltage(command_ch1);
  }

  // the loop function does the actual motor control.
  motor_left.loop();
  motor_right.loop();
}
