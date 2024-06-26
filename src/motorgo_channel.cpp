#include "motorgo_channel.h"

// MotorChannel class constructor
MotorGo::MotorChannel::MotorChannel(BLDCChannelParameters params,
                                    const char* name)
    : name(name),
      motor(BLDCMotor(0)),  // Placeholder value, will be set in init
      driver(BLDCDriver6PWM(params.uh, params.ul, params.vh, params.vl,
                            params.wh, params.wl)),
      encoder(MagneticSensorMT6701SSI(params.enc_cs)),
      sensor_calibrated(CalibratedSensor(encoder))
{
  // Set current sensing pins to input
  //   If params.current_u is not set (255), set the pin mode to INPUT
  if (params.current_u != MOTORGO_GPIO_NOT_SET)
  {
    pinMode(params.current_u, INPUT);
  }
  if (params.current_v != MOTORGO_GPIO_NOT_SET)
  {
    pinMode(params.current_v, INPUT);
  }
  if (params.current_w != MOTORGO_GPIO_NOT_SET)
  {
    pinMode(params.current_w, INPUT);
  }
}

void MotorGo::MotorChannel::init(ChannelConfiguration config,
                                 bool should_calibrate)
{
  // Save motor parameters
  channel_config = config;
  this->should_calibrate = should_calibrate;

  // Set the correct number of pole pairs
  motor.pole_pairs = channel_config.motor_config.pole_pairs;
  motor.KV_rating = channel_config.motor_config.kv;
  motor.phase_resistance = channel_config.motor_config.phase_resistance;
  motor.phase_inductance = channel_config.motor_config.phase_inductance;
  //   Set default LPF time constants
  motor.LPF_velocity.Tf = 0.001;
  motor.LPF_angle.Tf = 0.001;

  // Init encoder
  encoder.init(&MotorGo::hspi);

  // Link encoder to motor
  motor.linkSensor(&encoder);

  // Init driver and link to motor
  driver.voltage_power_supply = channel_config.power_supply_voltage;
  driver.voltage_limit = DRIVER_VOLTAGE_LIMIT;
  driver.init();
  motor.linkDriver(&driver);

  // Set motor control parameters
  motor.foc_modulation = FOCModulationType::SpaceVectorPWM;
  motor.voltage_limit = channel_config.motor_config.voltage_limit;

  //   Set the motor current limit to the lesser of the current limit and the
  //   driver current limit
  motor.current_limit =
      min(channel_config.motor_config.current_limit, DRIVER_CURRENT_LIMIT);

  // Initialize motor
  motor.init();

  // Calibrate encoders/motors if flag is set
  sensor_calibrated.voltage_calibration =
      channel_config.motor_config.calibration_voltage;

  if (should_calibrate)
  {
    sensor_calibrated.calibrate(motor, name);
    calibration_loaded = true;
  }
  // Use calibration data if it exists
  else if (sensor_calibrated.loadCalibrationData(motor, name))
  {
    // Set flag to indicate that calibration data was loaded
    calibration_loaded = true;
  }
  else
  {
    // If no data was found, issue a warning that only open-loop control will
    // be available
    Serial.print("WARNING: No calibration data found for ");
    Serial.print(name);
    Serial.println("... Motor will only be usable in open-loop control modes");
    Serial.println("Set should_calibrate to true to calibrate the motor.");
    calibration_loaded = false;
  }

  // Link the calibrated sensor to the motor
  motor.linkSensor(&sensor_calibrated);

  // Init FOC
  motor.initFOC();

  // Set direction of motor
  // Motor will spin counter-clockwise when viewed from the front if reversed is
  // false Motor will spin clockwise when viewed from the front if reversed is
  // true
  // Multiply by sensor direction to correctly handle the motor being plugged
  //   in any configuration
  if (channel_config.reversed)
  {
    // Direction enum is defined with looking at the motor from the back.
    // motor_direction_ch0 = Direction::CCW * motor_ch0.sensor_direction;
    // Add cast
    motor_direction =
        static_cast<Direction>(Direction::CCW * motor.sensor_direction);
  }
  else
  {
    motor_direction =
        static_cast<Direction>(Direction::CW * motor.sensor_direction);
  }

  // Print init message
  Serial.print("Initialized ");
  Serial.println(name);

  // Set the motor to be disabled
  disable();
}

void MotorGo::MotorChannel::init(ChannelConfiguration config)
{
  // Call the other init function with should_calibrate set to false
  init(config, false);
}

void MotorGo::MotorChannel::loop()
{
  // Update the PID controlller state
  motor.move();

  //   Update the commutation
  motor.loopFOC();
}

// Getters
float MotorGo::MotorChannel::get_position()
{
  return motor.shaftAngle() * motor_direction;
}
float MotorGo::MotorChannel::get_velocity()
{
  return motor.shaftVelocity() * motor_direction;
}
float MotorGo::MotorChannel::get_voltage()
{
  return motor.voltage.q * motor_direction;
}

// Setters

void MotorGo::MotorChannel::enable() { motor.enable(); }

void MotorGo::MotorChannel::disable() { motor.disable(); }

void MotorGo::MotorChannel::set_control_mode(MotorGo::ControlMode control_mode)
{
  // Save control mode
  this->control_mode = control_mode;

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
      motor.torque_controller = TorqueControlType::foc_current;
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

void MotorGo::MotorChannel::set_target_velocity(float target)
{
  target_velocity = target;

  if (velocity_limit_enabled)
  {
    target_velocity =
        _constrain(target_velocity, -velocity_limit, velocity_limit);
  }

  // Multiply by the motor direction to correct the sign
  target_velocity *= motor_direction;

  // If the control mode is velocity open loop, move the motor
  // If closed loop velocity, move the motor only if PID params are set
  switch (control_mode)
  {
    case MotorGo::ControlMode::VelocityOpenLoop:
      motor.move(target_velocity);
      break;
    case MotorGo::ControlMode::Velocity:
      if (pid_velocity_enabled)
      {
        motor.move(target_velocity);
      }
      else
      {
        disable();
      }
      break;
  }
}
void MotorGo::MotorChannel::set_target_torque(float target)
{
  target_torque = target;
  if (torque_limit_enabled)
  {
    target_torque = _constrain(target_torque, -torque_limit, torque_limit);
  }

  // Multiply by the motor direction to correct the sign
  target_torque *= motor_direction;

  if (control_mode == MotorGo::ControlMode::Torque)
  {
    if (pid_torque_enabled)
    {
      motor.move(target_torque);
    }
    else
    {
      disable();
    }
  }
}

void MotorGo::MotorChannel::set_target_position(float target)
{
  target_position = target;

  if (position_limit_enabled)
  {
    target_position =
        _constrain(target_position, position_limit_low, position_limit_high);
  }

  //  Multiply by the motor direction to correct the sign
  target_position *= motor_direction;

  // If the control mode is position open loop, move the motor
  // If closed loop position, move the motor only if PID params are set
  switch (control_mode)
  {
    case MotorGo::ControlMode::PositionOpenLoop:
      motor.move(target_position);
      break;
    case MotorGo::ControlMode::Position:
      if (pid_position_enabled)
      {
        motor.move(target_position);
      }
      else
      {
        disable();
      }
      break;
  }
}

void MotorGo::MotorChannel::set_target_voltage(float target)
{
  target_voltage = target;

  if (voltage_limit_enabled)
  {
    target_voltage = _constrain(target_voltage, -voltage_limit, voltage_limit);
  }

  // Multiply by the motor direction to correct the sign
  target_voltage *= motor_direction;

  if (control_mode == MotorGo::ControlMode::Voltage)
  {
    // With the phase resistance and KV set, voltage control behaves
    // similarly to an estimated current control. However, SimpleFOC does not
    // account for the current limits set in this mode. Since we use the current
    // limit as a way to protect the motor, we also constrain the voltage
    // command to the current limit.
    // motor.move(
    //     _constrain(target_voltage, -motor.current_limit,
    //     motor.current_limit));

    motor.move(target_voltage);
  }
}

float MotorGo::MotorChannel::get_torque_limit() { return torque_limit; }

float MotorGo::MotorChannel::get_velocity_limit() { return velocity_limit; }

float MotorGo::MotorChannel::get_position_limit_low()
{
  return position_limit_low;
}

float MotorGo::MotorChannel::get_position_limit_high()
{
  return position_limit_high;
}

float MotorGo::MotorChannel::get_voltage_limit() { return voltage_limit; }

void MotorGo::MotorChannel::set_torque_limit(float limit)
{
  torque_limit = limit;
  torque_limit_enabled = true;
}

void MotorGo::MotorChannel::set_velocity_limit(float limit)
{
  velocity_limit = limit;
  velocity_limit_enabled = true;
}

void MotorGo::MotorChannel::set_position_limit(float low, float high)
{
  position_limit_low = low;
  position_limit_high = high;
  position_limit_enabled = true;
}

void MotorGo::MotorChannel::set_voltage_limit(float limit)
{
  voltage_limit = limit;
  voltage_limit_enabled = true;
}

void MotorGo::MotorChannel::zero_position()
{
  // Shaft angle returns the filtered angle, with the sensor offset applied
  //   Add current sensor_offset back to the current shaft angle to
  //   recover the filtered (but not offset) angle
  motor.sensor_offset = motor.shaftAngle() + motor.sensor_offset;
}

// Built In PID controller functions
void MotorGo::MotorChannel::set_torque_controller(MotorGo::PIDParameters params)
{
  motor.PID_current_q.P = params.p;
  motor.PID_current_q.I = params.i;
  motor.PID_current_q.D = params.d;
  motor.PID_current_q.output_ramp = params.output_ramp;
  //   motor.PID_current_q.limit = params.limit;
  motor.LPF_current_q.Tf = params.lpf_time_constant;

  //   If calibration data is loaded, enable the torque controller
  //   Else, keep the torque controller disabled
  pid_torque_enabled = calibration_loaded;
}

void MotorGo::MotorChannel::set_velocity_controller(
    MotorGo::PIDParameters params)
{
  motor.PID_velocity.P = params.p;
  motor.PID_velocity.I = params.i;
  motor.PID_velocity.D = params.d;
  motor.PID_velocity.output_ramp = params.output_ramp;
  //   motor.PID_velocity.limit = params.limit;
  motor.LPF_velocity.Tf = params.lpf_time_constant;

  //   If calibration data is loaded, enable the velocity controller
  //   Else, keep the velocity controller disabled
  pid_velocity_enabled = calibration_loaded;
}

void MotorGo::MotorChannel::set_position_controller(
    MotorGo::PIDParameters params)
{
  motor.P_angle.P = params.p;
  motor.P_angle.I = params.i;
  motor.P_angle.D = params.d;
  motor.P_angle.output_ramp = params.output_ramp;
  //   motor.P_angle.limit = params.limit;
  motor.LPF_angle.Tf = params.lpf_time_constant;

  //   If calibration data is loaded, enable the position controller
  //   Else, keep the position controller disabled
  pid_position_enabled = calibration_loaded;
}

void MotorGo::MotorChannel::reset_torque_controller()
{
  motor.PID_current_q.reset();
}

void MotorGo::MotorChannel::reset_velocity_controller()
{
  motor.PID_velocity.reset();
}

void MotorGo::MotorChannel::reset_position_controller()
{
  motor.P_angle.reset();
}

void MotorGo::MotorChannel::save_controller_helper(
    const char* key, const PIDController& controller, const LowPassFilter& lpf)
{
  packed_pid_parameters_t packed_params;
  packed_params.p = controller.P;
  packed_params.i = controller.I;
  packed_params.d = controller.D;
  packed_params.output_ramp = controller.output_ramp;
  packed_params.lpf_time_constant = lpf.Tf;
  packed_params.limit = controller.limit;

  Preferences preferences;
  preferences.begin("pid", false);

  preferences.putBytes(key, packed_params.raw, sizeof(packed_params));

  preferences.end();
}

void MotorGo::MotorChannel::load_controller_helper(const char* key,
                                                   PIDController& controller,
                                                   LowPassFilter& lpf)
{
  Preferences preferences;
  preferences.begin("pid", true);

  packed_pid_parameters_t packed_params;

  //   Confirm key exists
  if (preferences.isKey(key))
  {
    preferences.getBytes(key, packed_params.raw, sizeof(packed_params));
  }

  preferences.end();

  controller.P = packed_params.p;
  controller.I = packed_params.i;
  controller.D = packed_params.d;
  controller.output_ramp = packed_params.output_ramp;
  //   controller.limit = packed_params.limit;
  lpf.Tf = packed_params.lpf_time_constant;
}

void MotorGo::MotorChannel::save_position_controller()
{
  save_controller_helper("ch0_position", motor.P_angle, motor.LPF_angle);
}

void MotorGo::MotorChannel::save_velocity_controller()
{
  save_controller_helper("ch0_velocity", motor.PID_velocity,
                         motor.LPF_velocity);
}

void MotorGo::MotorChannel::save_torque_controller()
{
  save_controller_helper("ch0_torque", motor.PID_current_q,
                         motor.LPF_current_q);
}

void MotorGo::MotorChannel::load_position_controller()
{
  load_controller_helper("ch0_position", motor.P_angle, motor.LPF_angle);
}

void MotorGo::MotorChannel::load_velocity_controller()
{
  load_controller_helper("ch0_velocity", motor.PID_velocity,
                         motor.LPF_velocity);
}

void MotorGo::MotorChannel::load_torque_controller()
{
  load_controller_helper("ch0_torque", motor.PID_current_q,
                         motor.LPF_current_q);
}