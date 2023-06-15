#include "motorgo_mini.h"

// Encoder I2C bus, define and initialize here to avoid multiple definitions
SPIClass MotorGo::hspi = SPIClass(HSPI);
bool hspi_initialized = false;

Commander MotorGo::command = Commander(Serial);
BLDCMotor MotorGo::motor_ch0 = BLDCMotor(11);
BLDCMotor MotorGo::motor_ch1 = BLDCMotor(11);

void do_target_ch0(char* cmd)
{
  MotorGo::command.motor(&MotorGo::motor_ch0, cmd);
}
void do_target_ch1(char* cmd)
{
  MotorGo::command.motor(&MotorGo::motor_ch1, cmd);
}

MotorGo::MotorGoMini::MotorGoMini()
    : encoder_ch0(MagneticSensorMT6701SSI(k_ch0_enc_cs)),
      driver_ch0(BLDCDriver6PWM(k_ch0_gpio_uh, k_ch0_gpio_ul, k_ch0_gpio_vh,
                                k_ch0_gpio_vl, k_ch0_gpio_wh, k_ch0_gpio_wl)),
      sensor_calibrated_ch0(CalibratedSensor(encoder_ch0)),
      encoder_ch1(MagneticSensorMT6701SSI(k_ch1_enc_cs)),
      sensor_calibrated_ch1(CalibratedSensor(encoder_ch1)),
      driver_ch1(BLDCDriver6PWM(k_ch1_gpio_uh, k_ch1_gpio_ul, k_ch1_gpio_vh,
                                k_ch1_gpio_vl, k_ch1_gpio_wh, k_ch1_gpio_wl))
{
  pinMode(k_ch0_current_u, INPUT);
  pinMode(k_ch0_current_w, INPUT);
  pinMode(k_ch1_current_u, INPUT);
  pinMode(k_ch1_current_w, INPUT);
}

void MotorGo::MotorGoMini::init_ch0() { init_ch0(false, false); }

void MotorGo::MotorGoMini::init_helper(BLDCMotor& motor, BLDCDriver6PWM& driver,
                                       CalibratedSensor& sensor_calibrated,
                                       MagneticSensorMT6701SSI& encoder,
                                       const char* name)
{
  // Init encoder
  encoder.init(&MotorGo::hspi);

  // Link encoder to motor
  motor.linkSensor(&encoder);

  // Init driver and link to motor
  driver.voltage_power_supply = k_voltage_power_supply;
  driver.voltage_limit = k_voltage_limit;
  driver.init();
  motor.linkDriver(&driver);

  // Set motor control parameters
  motor.foc_modulation = FOCModulationType::SpaceVectorPWM;
  motor.controller = MotionControlType::torque;
  motor.torque_controller = TorqueControlType::voltage;
  motor.velocity_limit = k_velocity_limit;
  motor.voltage_limit = k_voltage_limit;
  motor.current_limit = k_current_limit;

  // FOCStudio options
  if (enable_foc_studio)
  {
    // use monitoring
    motor.monitor_variables = _MON_TARGET | _MON_VEL | _MON_ANGLE;
    motor.useMonitoring(Serial);
  }

  // Initialize motor
  motor.init();

  // Calibrate encoders
  sensor_calibrated.voltage_calibration = k_voltage_calibration;
  if (should_calibrate)
  {
    sensor_calibrated.calibrate(motor, name);
  }
  // Use calibration data if it exists
  else if (!sensor_calibrated.loadCalibrationData(motor, name))
  {
    // If no data was found, calibrate the sensor
    sensor_calibrated.calibrate(motor, name);
  }

  // Link the calibrated sensor to the motor
  motor.linkSensor(&sensor_calibrated);

  // Init FOC
  motor.initFOC();

  // Print init message
  Serial.print("Initialized ");
  Serial.println(name);
}

void MotorGo::MotorGoMini::init_ch0(bool should_calibrate,
                                    bool enable_foc_studio)
{
  //   Guard to prevent multiple initializations, which could cause a crash
  if (!hspi_initialized)
  {
    hspi_initialized = true;
    MotorGo::hspi.begin(enc_scl, enc_sda, 45, 46);
  }

  this->should_calibrate = should_calibrate;
  this->enable_foc_studio = enable_foc_studio;
  Serial.print("Enable FOC Studio? ");
  Serial.println(enable_foc_studio ? "Yes" : "No");

  // Initialize motors
  init_helper(MotorGo::motor_ch0, driver_ch0, sensor_calibrated_ch0,
              encoder_ch0, "ch0");
  //   init_helper(MotorGo::motor_ch1, driver_ch1, sensor_calibrated_ch1,
  //   encoder_ch1, "ch1");

  // Set PID parameters for both motors
  //   MotorGo::motor_ch1.PID_velocity.P = 0.75;
  //   MotorGo::motor_ch1.PID_velocity.I = 0.09;
  //   MotorGo::motor_ch1.PID_velocity.D = 0.001;
  //   MotorGo::motor_ch1.PID_velocity.output_ramp = 10000.0;

  MotorGo::motor_ch0.PID_velocity.P = 0.75;
  MotorGo::motor_ch0.PID_velocity.I = 0.09;
  MotorGo::motor_ch0.PID_velocity.D = 0.001;
  MotorGo::motor_ch0.PID_velocity.output_ramp = 10000.0;

  // add command to commander
  if (enable_foc_studio)
  {
    MotorGo::command.add('0', do_target_ch0, (char*)"target");
    command.add('1', do_target_ch1, (char*)"target");
  }

  MotorGo::motor_ch0.disable();
  //   MotorGo::motor_ch1.disable();
}

void MotorGo::MotorGoMini::loop_ch0()
{
  MotorGo::motor_ch0.loopFOC();
  //   MotorGo::motor_ch1.loopFOC();

  // this function can be run at much lower frequency than loopFOC()
  MotorGo::motor_ch0.move();
  //   MotorGo::motor_ch1.move();

  // Monitoring, use only if necessary as it slows loop down significantly
  if (enable_foc_studio)
  {
    // user communication
    MotorGo::command.run();

    MotorGo::motor_ch0.monitor();
    // MotorGo::motor_ch1.monitor();
  }

  // Print u and w currents
  //   Serial.print("Current u: ");
  //   Serial.print(analogRead(k_ch0_current_u));
  //   Serial.print(" Current w: ");
  //   Serial.println(analogRead(k_ch0_current_w));
}

////// Helper Functions
void set_control_mode_helper(BLDCMotor& motor,
                             MotorGo::ControlMode control_mode)
{
  //   Switch for control mode
  switch (control_mode)
  {
    case MotorGo::ControlMode::Voltage:
      motor.torque_controller = TorqueControlType::voltage;
      motor.controller = MotionControlType::torque;
      break;
    case MotorGo::ControlMode::Velocity:
      motor.torque_controller = TorqueControlType::voltage;
      motor.controller = MotionControlType::velocity;
      break;
    case MotorGo::ControlMode::Position:
      motor.torque_controller = TorqueControlType::voltage;
      motor.controller = MotionControlType::angle;
      break;
    case MotorGo::ControlMode::Torque:
      motor.torque_controller = TorqueControlType::dc_current;
      motor.controller = MotionControlType::torque;
      break;
    case MotorGo::ControlMode::VelocityOpenLoop:
      motor.torque_controller = TorqueControlType::voltage;
      motor.controller = MotionControlType::velocity_openloop;
      break;
    case MotorGo::ControlMode::PositionOpenLoop:
      motor.torque_controller = TorqueControlType::voltage;
      motor.controller = MotionControlType::angle_openloop;
      break;
    default:
      Serial.println("Invalid control mode");
      break;
  }
}

void MotorGo::MotorGoMini::set_target_helper_ch0()
{
  // Switch command based on current control mode
  switch (control_mode_ch0)
  {
    case MotorGo::ControlMode::Voltage:
      motor_ch0.move(target_voltage_ch0);
      break;
    case MotorGo::ControlMode::Torque:
      motor_ch0.move(target_torque_ch0);
      break;
    case MotorGo::ControlMode::Velocity:
      motor_ch0.move(target_velocity_ch0);
      break;
    case MotorGo::ControlMode::Position:
      motor_ch0.move(target_position_ch0);
      break;
    case MotorGo::ControlMode::VelocityOpenLoop:
      motor_ch0.move(target_velocity_ch0);
      break;
    case MotorGo::ControlMode::PositionOpenLoop:
      motor_ch0.move(target_position_ch0);
      break;
  }
}

void MotorGo::MotorGoMini::set_target_helper_ch1()
{
  if (!enable_foc_studio)
  {
    // Switch command based on current control mode
    switch (control_mode_ch1)
    {
      case MotorGo::ControlMode::Voltage:
        motor_ch1.move(target_voltage_ch1);
        break;
      case MotorGo::ControlMode::Torque:
        motor_ch1.move(target_torque_ch1);
        break;
      case MotorGo::ControlMode::Velocity:
        motor_ch1.move(target_velocity_ch1);
        break;
      case MotorGo::ControlMode::Position:
        motor_ch1.move(target_position_ch1);
        break;
      case MotorGo::ControlMode::VelocityOpenLoop:
        motor_ch1.move(target_velocity_ch1);
        break;
      case MotorGo::ControlMode::PositionOpenLoop:
        motor_ch1.move(target_position_ch1);
        break;
    }
  }
}

// Setters
void MotorGo::MotorGoMini::enable_ch0() { MotorGo::motor_ch0.enable(); }
void MotorGo::MotorGoMini::enable_ch1() { MotorGo::motor_ch1.enable(); }
void MotorGo::MotorGoMini::disable_ch0() { MotorGo::motor_ch0.disable(); }
void MotorGo::MotorGoMini::disable_ch1() { MotorGo::motor_ch1.disable(); }

void MotorGo::MotorGoMini::set_control_mode_ch0(
    MotorGo::ControlMode control_mode)
{
  MotorGo::MotorGoMini::control_mode_ch0 = control_mode;
  set_control_mode_helper(MotorGo::motor_ch0, control_mode);
}

void MotorGo::MotorGoMini::set_control_mode_ch1(
    MotorGo::ControlMode control_mode)
{
  MotorGo::MotorGoMini::control_mode_ch1 = control_mode;
  set_control_mode_helper(MotorGo::motor_ch1, control_mode);
}

void MotorGo::MotorGoMini::set_target_velocity_ch0(float target)
{
  target_velocity_ch0 = target;
  set_target_helper_ch0();
}
void MotorGo::MotorGoMini::set_target_velocity_ch1(float target)
{
  target_velocity_ch1 = target;
  set_target_helper_ch1();
}

void MotorGo::MotorGoMini::set_target_torque_ch0(float target)
{
  target_torque_ch0 = target;
  set_target_helper_ch0();
}
void MotorGo::MotorGoMini::set_target_torque_ch1(float target)
{
  target_torque_ch1 = target;
  set_target_helper_ch1();
}

void MotorGo::MotorGoMini::set_target_position_ch0(float target)
{
  target_position_ch0 = target;
  set_target_helper_ch0();
}
void MotorGo::MotorGoMini::set_target_position_ch1(float target)
{
  target_position_ch1 = target;
  set_target_helper_ch1();
}

// Getters
float MotorGo::MotorGoMini::get_ch0_position()
{
  return MotorGo::motor_ch0.shaftAngle();
}
float MotorGo::MotorGoMini::get_ch0_velocity()
{
  return MotorGo::motor_ch0.shaftVelocity();
}
float MotorGo::MotorGoMini::get_ch0_voltage()
{
  return MotorGo::motor_ch0.voltage.q;
}

float MotorGo::MotorGoMini::get_ch1_position()
{
  return MotorGo::motor_ch1.shaftAngle();
}
float MotorGo::MotorGoMini::get_ch1_velocity()
{
  return MotorGo::motor_ch1.shaftVelocity();
}
float MotorGo::MotorGoMini::get_ch1_voltage()
{
  return MotorGo::motor_ch1.voltage.q;
}

// TODO: Implement get torque
