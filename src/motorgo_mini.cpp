#include "motorgo_mini.h"

#include "Preferences.h"

// Encoder I2C bus, define and initialize here to avoid multiple definitions
SPIClass MotorGo::hspi = SPIClass(HSPI);
bool hspi_initialized = false;

// TODO: Motors are currently instantiated with a default number of pole pairs
// This is not great design, would be better to not initialize motors
// until the user calls init_ch0() or init_ch1()
BLDCMotor MotorGo::motor_ch0 = BLDCMotor(3);
BLDCMotor MotorGo::motor_ch1 = BLDCMotor(3);



MotorGo::MotorGoMini::MotorGoMini()
    : encoder_ch0(MagneticSensorMT6701SSI(CH0_ENC_CS)),
      driver_ch0(BLDCDriver6PWM(CH0_GPIO_UH, CH0_GPIO_UL, CH0_GPIO_VH,
                                CH0_GPIO_VL, CH0_GPIO_WH, CH0_GPIO_WL)),
      sensor_calibrated_ch0(CalibratedSensor(encoder_ch0)),
      encoder_ch1(MagneticSensorMT6701SSI(CH1_ENC_CS)),
      sensor_calibrated_ch1(CalibratedSensor(encoder_ch1)),
      driver_ch1(BLDCDriver6PWM(CH1_GPIO_UH, CH1_GPIO_UL, CH1_GPIO_VH,
                                CH1_GPIO_VL, CH1_GPIO_WH, CH1_GPIO_WL))
{
  // Set current sensing pins to input
  pinMode(CH0_CURRENT_U, INPUT);
  pinMode(CH0_CURRENT_W, INPUT);
  pinMode(CH1_CURRENT_U, INPUT);
  pinMode(CH1_CURRENT_W, INPUT);
}

void MotorGo::MotorGoMini::init_helper(MotorParameters params,
                                       bool should_calibrate,
                                       BLDCMotor& motor,
                                       BLDCDriver6PWM& driver,
                                       CalibratedSensor& sensor_calibrated,
                                       MagneticSensorMT6701SSI& encoder,
                                       const char* name)
{
  // Reconfigure number of pole pairs
  motor.pole_pairs = params.pole_pairs;

  // Init encoder
  encoder.init(&MotorGo::hspi);

  // Link encoder to motor
  motor.linkSensor(&encoder);

  // Init driver and link to motor
  driver.voltage_power_supply = params.power_supply_voltage;
  driver.voltage_limit = params.voltage_limit;
  driver.init();
  motor.linkDriver(&driver);

  // Set motor control parameters
  motor.foc_modulation = FOCModulationType::SpaceVectorPWM;
  motor.voltage_limit = params.voltage_limit;
  motor.current_limit = params.current_limit;


  // Initialize motor
  motor.init();

  // Calibrate encoders
  sensor_calibrated.voltage_calibration = params.calibration_voltage;
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

void MotorGo::MotorGoMini::init_ch0(MotorParameters params)
{
  init_ch0(params, false);
}

void MotorGo::MotorGoMini::init_ch1(MotorParameters params)
{
  init_ch1(params, false);
}

void MotorGo::MotorGoMini::init_ch0(MotorParameters params,
                                    bool should_calibrate)
{
  //   Guard to prevent multiple initializations, which could cause a crash
  if (!hspi_initialized)
  {
    hspi_initialized = true;
    MotorGo::hspi.begin(ENC_SCL, ENC_SDA, 45, 46);
  }

  this->should_calibrate_ch0 = should_calibrate;

  //   Save motor parameters
  motor_params_ch0 = params;

  // Initialize motors
  init_helper(params, should_calibrate, MotorGo::motor_ch0,
              driver_ch0, sensor_calibrated_ch0, encoder_ch0, "ch0");

  disable_ch0();
}

void MotorGo::MotorGoMini::init_ch1(MotorParameters params,
                                    bool should_calibrate)
{
  //   Guard to prevent multiple initializations, which could cause a crash
  if (!hspi_initialized)
  {
    hspi_initialized = true;
    MotorGo::hspi.begin(ENC_SCL, ENC_SDA, 45, 46);
  }

  this->should_calibrate_ch1 = should_calibrate;

  //   Save motor parameters
  motor_params_ch1 = params;

  // Initialize motors
  init_helper(params, should_calibrate, MotorGo::motor_ch1,
              driver_ch1, sensor_calibrated_ch1, encoder_ch1, "ch1");

  disable_ch1();
}

void MotorGo::MotorGoMini::loop_ch0()
{
  MotorGo::motor_ch0.loopFOC();
  //   MotorGo::motor_ch1.loopFOC();

  // this function can be run at much lower frequency than loopFOC()
  MotorGo::motor_ch0.move();

}

void MotorGo::MotorGoMini::loop_ch1()
{
  MotorGo::motor_ch1.loopFOC();
  //   MotorGo::motor_ch1.loopFOC();

  // this function can be run at much lower frequency than loopFOC()
  MotorGo::motor_ch1.move();
  //   MotorGo::motor_ch1.move();

}

////// Helper Functions
void MotorGo::MotorGoMini::set_control_mode_helper(
    BLDCMotor& motor, MotorGo::ControlMode control_mode)
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
      //  Disable motor if PID params not set
      if (!pid_torque_ch0_enabled)
      {
        disable_ch0();
      }
      motor_ch0.move(target_torque_ch0);
      break;
    case MotorGo::ControlMode::Velocity:
      // Disable motor if PID params not set
      if (!pid_velocity_ch0_enabled)
      {
        disable_ch0();
      }

      motor_ch0.move(target_velocity_ch0);
      break;
    case MotorGo::ControlMode::Position:
      // Disable motor if PID params not set
      if (!pid_position_ch0_enabled)
      {
        disable_ch0();
      }
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
    // Switch command based on current control mode
    switch (control_mode_ch1)
    {
      case MotorGo::ControlMode::Voltage:
        motor_ch1.move(target_voltage_ch1);
        break;
      case MotorGo::ControlMode::Torque:
        //   Disable motor if PID params not set
        if (!pid_torque_ch1_enabled)
        {
          disable_ch1();
        }
        motor_ch1.move(target_torque_ch1);
        break;
      case MotorGo::ControlMode::Velocity:
        //  Disable motor if PID params not set
        if (!pid_velocity_ch1_enabled)
        {
          disable_ch1();
        }
        motor_ch1.move(target_velocity_ch1);
        break;
      case MotorGo::ControlMode::Position:
        //  Disable motor if PID params not set
        if (!pid_position_ch1_enabled)
        {
          disable_ch1();
        }
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

void MotorGo::MotorGoMini::set_torque_controller_helper(
    BLDCMotor& motor, MotorGo::PIDParameters params)
{
  motor.PID_current_q.P = params.p;
  motor.PID_current_q.I = params.i;
  motor.PID_current_q.D = params.d;
  motor.PID_current_q.output_ramp = params.output_ramp;
  motor.PID_current_q.limit = params.limit;
  motor.LPF_current_q.Tf = params.lpf_time_constant;
}

void MotorGo::MotorGoMini::set_velocity_controller_helper(
    BLDCMotor& motor, MotorGo::PIDParameters params)
{
  motor.PID_velocity.P = params.p;
  motor.PID_velocity.I = params.i;
  motor.PID_velocity.D = params.d;
  motor.PID_velocity.output_ramp = params.output_ramp;
  motor.PID_velocity.limit = params.limit;
  motor.LPF_velocity.Tf = params.lpf_time_constant;
}

void MotorGo::MotorGoMini::set_position_controller_helper(
    BLDCMotor& motor, MotorGo::PIDParameters params)
{
  motor.P_angle.P = params.p;
  motor.P_angle.I = params.i;
  motor.P_angle.D = params.d;
  motor.P_angle.output_ramp = params.output_ramp;
  motor.P_angle.limit = params.limit;
  motor.LPF_angle.Tf = params.lpf_time_constant;
}

// Getters
MotorGo::PIDParameters MotorGo::MotorGoMini::get_torque_controller_ch0()
{
  MotorGo::PIDParameters params;
  params.p = MotorGo::motor_ch0.PID_current_q.P;
  params.i = MotorGo::motor_ch0.PID_current_q.I;
  params.d = MotorGo::motor_ch0.PID_current_q.D;
  params.output_ramp = MotorGo::motor_ch0.PID_current_q.output_ramp;
  params.lpf_time_constant = MotorGo::motor_ch0.LPF_current_q.Tf;
  params.limit = MotorGo::motor_ch0.PID_current_q.limit;

  return params;
}

MotorGo::PIDParameters MotorGo::MotorGoMini::get_torque_controller_ch1()
{
  MotorGo::PIDParameters params;
  params.p = MotorGo::motor_ch1.PID_current_q.P;
  params.i = MotorGo::motor_ch1.PID_current_q.I;
  params.d = MotorGo::motor_ch1.PID_current_q.D;
  params.output_ramp = MotorGo::motor_ch1.PID_current_q.output_ramp;
  params.lpf_time_constant = MotorGo::motor_ch1.LPF_current_q.Tf;
  params.limit = MotorGo::motor_ch1.PID_current_q.limit;

  return params;
}

MotorGo::PIDParameters MotorGo::MotorGoMini::get_velocity_controller_ch0()
{
  MotorGo::PIDParameters params;
  params.p = MotorGo::motor_ch0.PID_velocity.P;
  params.i = MotorGo::motor_ch0.PID_velocity.I;
  params.d = MotorGo::motor_ch0.PID_velocity.D;
  params.output_ramp = MotorGo::motor_ch0.PID_velocity.output_ramp;
  params.lpf_time_constant = MotorGo::motor_ch0.LPF_velocity.Tf;
  params.limit = MotorGo::motor_ch0.PID_velocity.limit;

  return params;
}

MotorGo::PIDParameters MotorGo::MotorGoMini::get_velocity_controller_ch1()
{
  MotorGo::PIDParameters params;
  params.p = MotorGo::motor_ch1.PID_velocity.P;
  params.i = MotorGo::motor_ch1.PID_velocity.I;
  params.d = MotorGo::motor_ch1.PID_velocity.D;
  params.output_ramp = MotorGo::motor_ch1.PID_velocity.output_ramp;
  params.lpf_time_constant = MotorGo::motor_ch1.LPF_velocity.Tf;
  params.limit = MotorGo::motor_ch1.PID_velocity.limit;

  return params;
}

MotorGo::PIDParameters MotorGo::MotorGoMini::get_position_controller_ch0()
{
  MotorGo::PIDParameters params;
  params.p = MotorGo::motor_ch0.P_angle.P;
  params.i = MotorGo::motor_ch0.P_angle.I;
  params.d = MotorGo::motor_ch0.P_angle.D;
  params.output_ramp = MotorGo::motor_ch0.P_angle.output_ramp;
  params.lpf_time_constant = MotorGo::motor_ch0.LPF_angle.Tf;
  params.limit = MotorGo::motor_ch0.P_angle.limit;

  return params;
}

MotorGo::PIDParameters MotorGo::MotorGoMini::get_position_controller_ch1()
{
  MotorGo::PIDParameters params;
  params.p = MotorGo::motor_ch1.P_angle.P;
  params.i = MotorGo::motor_ch1.P_angle.I;
  params.d = MotorGo::motor_ch1.P_angle.D;
  params.output_ramp = MotorGo::motor_ch1.P_angle.output_ramp;
  params.lpf_time_constant = MotorGo::motor_ch1.LPF_angle.Tf;
  params.limit = MotorGo::motor_ch1.P_angle.limit;

  return params;
}

// Setters
void MotorGo::MotorGoMini::set_torque_controller_ch0(
    MotorGo::PIDParameters params)
{
  set_torque_controller_helper(MotorGo::motor_ch0, params);
  pid_torque_ch0_enabled = true;
}

void MotorGo::MotorGoMini::set_torque_controller_ch1(
    MotorGo::PIDParameters params)
{
  set_torque_controller_helper(MotorGo::motor_ch1, params);
  pid_torque_ch1_enabled = true;
}

void MotorGo::MotorGoMini::set_velocity_controller_ch0(
    MotorGo::PIDParameters params)
{
  set_velocity_controller_helper(MotorGo::motor_ch0, params);
  pid_velocity_ch0_enabled = true;
}

void MotorGo::MotorGoMini::set_velocity_controller_ch1(
    MotorGo::PIDParameters params)
{
  set_velocity_controller_helper(MotorGo::motor_ch1, params);
  pid_velocity_ch1_enabled = true;
}

void MotorGo::MotorGoMini::set_position_controller_ch0(
    MotorGo::PIDParameters params)
{
  set_position_controller_helper(MotorGo::motor_ch0, params);
  pid_position_ch0_enabled = true;
}

void MotorGo::MotorGoMini::set_position_controller_ch1(
    MotorGo::PIDParameters params)
{
  set_position_controller_helper(MotorGo::motor_ch1, params);
  pid_position_ch1_enabled = true;
}

void MotorGo::MotorGoMini::reset_torque_controller_ch0()
{
  MotorGo::motor_ch0.PID_current_q.reset();
}

void MotorGo::MotorGoMini::reset_torque_controller_ch1()
{
  MotorGo::motor_ch1.PID_current_q.reset();
}

void MotorGo::MotorGoMini::reset_velocity_controller_ch0()
{
  MotorGo::motor_ch0.PID_velocity.reset();
}

void MotorGo::MotorGoMini::reset_velocity_controller_ch1()
{
  MotorGo::motor_ch1.PID_velocity.reset();
}

void MotorGo::MotorGoMini::reset_position_controller_ch0()
{
  MotorGo::motor_ch0.P_angle.reset();
}

void MotorGo::MotorGoMini::reset_position_controller_ch1()
{
  MotorGo::motor_ch1.P_angle.reset();
}

void MotorGo::MotorGoMini::save_controller_helper(
    const char* key, const packed_pid_parameters_t& packed_params)
{
  Preferences preferences;
  preferences.begin("pid", false);

  preferences.putBytes(key, packed_params.raw, sizeof(packed_params));

  preferences.end();
}

MotorGo::packed_pid_parameters_t MotorGo::MotorGoMini::load_controller_helper(
    const char* key)
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

  return packed_params;
}

void MotorGo::MotorGoMini::save_position_controller_ch0()
{
  packed_pid_parameters_t packed_params;
  packed_params.p = MotorGo::motor_ch0.P_angle.P;
  packed_params.i = MotorGo::motor_ch0.P_angle.I;
  packed_params.d = MotorGo::motor_ch0.P_angle.D;
  packed_params.output_ramp = MotorGo::motor_ch0.P_angle.output_ramp;
  packed_params.lpf_time_constant = MotorGo::motor_ch0.LPF_angle.Tf;
  packed_params.limit = MotorGo::motor_ch0.P_angle.limit;

  save_controller_helper("ch0_position", packed_params);
}

void MotorGo::MotorGoMini::save_position_controller_ch1()
{
  packed_pid_parameters_t packed_params;
  packed_params.p = MotorGo::motor_ch1.P_angle.P;
  packed_params.i = MotorGo::motor_ch1.P_angle.I;
  packed_params.d = MotorGo::motor_ch1.P_angle.D;
  packed_params.output_ramp = MotorGo::motor_ch1.P_angle.output_ramp;
  packed_params.lpf_time_constant = MotorGo::motor_ch1.LPF_angle.Tf;
  packed_params.limit = MotorGo::motor_ch1.P_angle.limit;

  save_controller_helper("ch1_position", packed_params);
}

void MotorGo::MotorGoMini::save_velocity_controller_ch0()
{
  packed_pid_parameters_t packed_params;
  packed_params.p = MotorGo::motor_ch0.PID_velocity.P;
  packed_params.i = MotorGo::motor_ch0.PID_velocity.I;
  packed_params.d = MotorGo::motor_ch0.PID_velocity.D;
  packed_params.output_ramp = MotorGo::motor_ch0.PID_velocity.output_ramp;
  packed_params.lpf_time_constant = MotorGo::motor_ch0.LPF_velocity.Tf;
  packed_params.limit = MotorGo::motor_ch0.PID_velocity.limit;

  save_controller_helper("ch0_velocity", packed_params);
}

void MotorGo::MotorGoMini::save_velocity_controller_ch1()
{
  packed_pid_parameters_t packed_params;
  packed_params.p = MotorGo::motor_ch1.PID_velocity.P;
  packed_params.i = MotorGo::motor_ch1.PID_velocity.I;
  packed_params.d = MotorGo::motor_ch1.PID_velocity.D;
  packed_params.output_ramp = MotorGo::motor_ch1.PID_velocity.output_ramp;
  packed_params.lpf_time_constant = MotorGo::motor_ch1.LPF_velocity.Tf;
  packed_params.limit = MotorGo::motor_ch1.PID_velocity.limit;

  save_controller_helper("ch1_velocity", packed_params);
}

void MotorGo::MotorGoMini::save_torque_controller_ch0()
{
  packed_pid_parameters_t packed_params;
  packed_params.p = MotorGo::motor_ch0.PID_current_q.P;
  packed_params.i = MotorGo::motor_ch0.PID_current_q.I;
  packed_params.d = MotorGo::motor_ch0.PID_current_q.D;
  packed_params.output_ramp = MotorGo::motor_ch0.PID_current_q.output_ramp;
  packed_params.lpf_time_constant = MotorGo::motor_ch0.LPF_current_q.Tf;
  packed_params.limit = MotorGo::motor_ch0.PID_current_q.limit;

  save_controller_helper("ch0_torque", packed_params);
}

void MotorGo::MotorGoMini::save_torque_controller_ch1()
{
  packed_pid_parameters_t packed_params;
  packed_params.p = MotorGo::motor_ch1.PID_current_q.P;
  packed_params.i = MotorGo::motor_ch1.PID_current_q.I;
  packed_params.d = MotorGo::motor_ch1.PID_current_q.D;
  packed_params.output_ramp = MotorGo::motor_ch1.PID_current_q.output_ramp;
  packed_params.lpf_time_constant = MotorGo::motor_ch1.LPF_current_q.Tf;
  packed_params.limit = MotorGo::motor_ch1.PID_current_q.limit;

  save_controller_helper("ch1_torque", packed_params);
}

void MotorGo::MotorGoMini::load_position_controller_ch0()
{
  packed_pid_parameters_t packed_params =
      load_controller_helper("ch0_position");

  MotorGo::motor_ch0.P_angle.P = packed_params.p;
  MotorGo::motor_ch0.P_angle.I = packed_params.i;
  MotorGo::motor_ch0.P_angle.D = packed_params.d;
  MotorGo::motor_ch0.P_angle.output_ramp = packed_params.output_ramp;
  MotorGo::motor_ch0.LPF_angle.Tf = packed_params.lpf_time_constant;
  MotorGo::motor_ch0.P_angle.limit = packed_params.limit;
}

void MotorGo::MotorGoMini::load_position_controller_ch1()
{
  packed_pid_parameters_t packed_params =
      load_controller_helper("ch1_position");

  MotorGo::motor_ch1.P_angle.P = packed_params.p;
  MotorGo::motor_ch1.P_angle.I = packed_params.i;
  MotorGo::motor_ch1.P_angle.D = packed_params.d;
  MotorGo::motor_ch1.P_angle.output_ramp = packed_params.output_ramp;
  MotorGo::motor_ch1.LPF_angle.Tf = packed_params.lpf_time_constant;
  MotorGo::motor_ch1.P_angle.limit = packed_params.limit;
}

void MotorGo::MotorGoMini::load_velocity_controller_ch0()
{
  packed_pid_parameters_t packed_params =
      load_controller_helper("ch0_velocity");

  MotorGo::motor_ch0.PID_velocity.P = packed_params.p;
  MotorGo::motor_ch0.PID_velocity.I = packed_params.i;
  MotorGo::motor_ch0.PID_velocity.D = packed_params.d;
  MotorGo::motor_ch0.PID_velocity.output_ramp = packed_params.output_ramp;
  MotorGo::motor_ch0.LPF_velocity.Tf = packed_params.lpf_time_constant;
  MotorGo::motor_ch0.PID_velocity.limit = packed_params.limit;
}

void MotorGo::MotorGoMini::load_velocity_controller_ch1()
{
  packed_pid_parameters_t packed_params =
      load_controller_helper("ch1_velocity");

  MotorGo::motor_ch1.PID_velocity.P = packed_params.p;
  MotorGo::motor_ch1.PID_velocity.I = packed_params.i;
  MotorGo::motor_ch1.PID_velocity.D = packed_params.d;
  MotorGo::motor_ch1.PID_velocity.output_ramp = packed_params.output_ramp;
  MotorGo::motor_ch1.LPF_velocity.Tf = packed_params.lpf_time_constant;
  MotorGo::motor_ch1.PID_velocity.limit = packed_params.limit;
}

void MotorGo::MotorGoMini::load_torque_controller_ch0()
{
  packed_pid_parameters_t packed_params = load_controller_helper("ch0_torque");

  MotorGo::motor_ch0.PID_current_q.P = packed_params.p;
  MotorGo::motor_ch0.PID_current_q.I = packed_params.i;
  MotorGo::motor_ch0.PID_current_q.D = packed_params.d;
  MotorGo::motor_ch0.PID_current_q.output_ramp = packed_params.output_ramp;
  MotorGo::motor_ch0.LPF_current_q.Tf = packed_params.lpf_time_constant;
  MotorGo::motor_ch0.PID_current_q.limit = packed_params.limit;
}

void MotorGo::MotorGoMini::load_torque_controller_ch1()
{
  packed_pid_parameters_t packed_params = load_controller_helper("ch1_torque");

  MotorGo::motor_ch1.PID_current_q.P = packed_params.p;
  MotorGo::motor_ch1.PID_current_q.I = packed_params.i;
  MotorGo::motor_ch1.PID_current_q.D = packed_params.d;
  MotorGo::motor_ch1.PID_current_q.output_ramp = packed_params.output_ramp;
  MotorGo::motor_ch1.LPF_current_q.Tf = packed_params.lpf_time_constant;
  MotorGo::motor_ch1.PID_current_q.limit = packed_params.limit;
}

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

void MotorGo::MotorGoMini::set_target_voltage_ch0(float target)
{
  target_voltage_ch0 = target;
  set_target_helper_ch0();
}

void MotorGo::MotorGoMini::set_target_voltage_ch1(float target)
{
  target_voltage_ch1 = target;
  set_target_helper_ch1();
}

void MotorGo::MotorGoMini::zero_position_ch0()
{
  MotorGo::motor_ch0.sensor_offset = MotorGo::motor_ch0.shaftAngle();
}

void MotorGo::MotorGoMini::zero_position_ch1()
{
  MotorGo::motor_ch1.sensor_offset = MotorGo::motor_ch1.shaftAngle();
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
