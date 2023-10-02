// #include <Arduino.h>

// #include "motorgo_mini.h"

// // Servo
// #include <Servo.h>

// #define LOOP_HZ 1000
// #define LOOP_US 1000000 / LOOP_HZ

// MotorGo::MotorGoMini* motorgo_mini;
// MotorGo::MotorParameters motor_params_ch0;
// MotorGo::PIDParameters velocity_pid_params_ch0;
// MotorGo::PIDParameters position_pid_params_ch0;

// Servo servo;
// const int servoPin = 8;
// const int frequency = 200;  // Hz

// void setup()
// {
//   Serial.begin(115200);

//   // Servo setup
//   servo.attach(servoPin, frequency);

//   //   Configure Servo on pin 8

//   // Setup motor parameters
//   //   motor_params_ch0.pole_pairs = 11;
//   //   motor_params_ch0.power_supply_voltage = 5.0;
//   //   motor_params_ch0.voltage_limit = 5.0;
//   //   motor_params_ch0.current_limit = 320;
//   //   motor_params_ch0.velocity_limit = 100.0;
//   //   motor_params_ch0.calibration_voltage = 3.0;

//   //   // Setup PID parameters
//   //   velocity_pid_params_ch0.p = 20.0;
//   //   velocity_pid_params_ch0.i = 0.5;
//   //   velocity_pid_params_ch0.d = 0.0;
//   //   velocity_pid_params_ch0.output_ramp = 10000.0;
//   //   velocity_pid_params_ch0.lpf_time_constant = 0.1;
//   //   velocity_pid_params_ch0.limit = 320.0;

//   //   position_pid_params_ch0.p = 1.0;
//   //   position_pid_params_ch0.i = 0.0;
//   //   position_pid_params_ch0.d = 0.0;
//   //   position_pid_params_ch0.output_ramp = 10000.0;
//   //   position_pid_params_ch0.lpf_time_constant = 0.1;
//   //   position_pid_params_ch0.limit = 320.0;

//   //   // Instantiate motorgo mini board
//   //   motorgo_mini = new MotorGo::MotorGoMini();

//   //   delay(1000);

//   //   // Setup Ch0 with FOCStudio enabled
//   //   motorgo_mini->init_ch0(motor_params_ch0, false, true);
//   //   // Set velocity controller parameters
//   //   motorgo_mini->set_velocity_controller_ch0(velocity_pid_params_ch0);
//   //   motorgo_mini->set_position_controller_ch0(position_pid_params_ch0);
//   //   // Set closed-loop velocity control mode
//   //   motorgo_mini->set_control_mode_ch0(MotorGo::ControlMode::Velocity);

//   // Setup Ch1 with FOCStudio enabled
//   //   motorgo_mini->init_ch1(motor_params_ch0, false, true);
//   //   // Set velocity controller parameters
//   //   motorgo_mini->set_velocity_controller_ch1(velocity_pid_params_ch0);
//   //   // Set closed-loop velocity control mode
//   //   motorgo_mini->set_control_mode_ch1(MotorGo::ControlMode::Velocity);

//   //   motorgo_mini->enable_ch0();

//   // Config analog input 47
//   //   pinMode(7, INPUT);
//   //   pinMode(47, )
// }

// // int i = 0;
// // Constrain loop speed to 250 Hz
// unsigned long last_loop_time = 0;
// void loop()
// {
//   // Servo sweep
//   for (int i = 0; i < 180; i++)
//   {
//     servo.write(i);
//     delay(15);
//   }
//   // Run Ch0
//   //   motorgo_mini->loop_ch0();

//   //   Serial.println(analogRead(7));
//   //   motorgo_mini->loop_ch1();

//   //   Print shaft velocity
//   //   Serial.print("Shaft velocity: ");

//   //   Spin forward
//   //   Enable
//   //   motorgo_mini->set_target_velocity_ch0(10.0);
//   //   i++;

//   // Delay necessary amount micros
//   //   unsigned long now = micros();
//   //   unsigned long loop_time = now - last_loop_time;

//   //   if (loop_time < LOOP_US)
//   //   {
//   //     delayMicroseconds(LOOP_US - loop_time);

//   //     // Serial.print("Loop time: ");
//   //   }

//   //   last_loop_time = now;
// }

// Basic demo for accelerometer readings from Adafruit ICM20948

#include <Adafruit_ICM20649.h>
#include <Adafruit_ICM20X.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

Adafruit_ICM20649 icm;
uint16_t measurement_delay_us =
    65535;  // Delay between measurements for testing
// For SPI mode, we need a CS pin
#define ICM_CS 10
// For software-SPI mode we need SCK/MOSI/MISO pins
#define ICM_SCK 13
#define ICM_MISO 12
#define ICM_MOSI 11

void setup(void)
{
  Serial.begin(115200);
  while (!Serial)
    delay(10);  // will pause Zero, Leonardo, etc until serial console opens

  Serial.println("Adafruit ICM20649 test!");

  // Try to initialize!
  Wire.begin(2, 1);

  Serial.println("ICM20649 test!");
  if (!icm.begin_I2C(0x68, &Wire))
  {
    // if (!icm.begin_SPI(ICM_CS)) {
    // if (!icm.begin_SPI(ICM_CS, ICM_SCK, ICM_MISO, ICM_MOSI)) {

    Serial.println("Failed to find ICM20649 chip");
    while (1)
    {
      delay(10);
    }
  }
  Serial.println("ICM20649 Found!");
  // icm.setAccelRange(ICM20649_ACCEL_RANGE_4_G);
  Serial.print("Accelerometer range set to: ");
  switch (icm.getAccelRange())
  {
    case ICM20649_ACCEL_RANGE_4_G:
      Serial.println("+-4G");
      break;
    case ICM20649_ACCEL_RANGE_8_G:
      Serial.println("+-8G");
      break;
    case ICM20649_ACCEL_RANGE_16_G:
      Serial.println("+-16G");
      break;
    case ICM20649_ACCEL_RANGE_30_G:
      Serial.println("+-30G");
      break;
  }

  // icm.setGyroRange(ICM20649_GYRO_RANGE_500_DPS);
  Serial.print("Gyro range set to: ");
  switch (icm.getGyroRange())
  {
    case ICM20649_GYRO_RANGE_500_DPS:
      Serial.println("500 degrees/s");
      break;
    case ICM20649_GYRO_RANGE_1000_DPS:
      Serial.println("1000 degrees/s");
      break;
    case ICM20649_GYRO_RANGE_2000_DPS:
      Serial.println("2000 degrees/s");
      break;
    case ICM20649_GYRO_RANGE_4000_DPS:
      Serial.println("4000 degrees/s");
      break;
  }

  //  icm.setAccelRateDivisor(4095);
  uint16_t accel_divisor = icm.getAccelRateDivisor();
  float accel_rate = 1125 / (1.0 + accel_divisor);

  Serial.print("Accelerometer data rate divisor set to: ");
  Serial.println(accel_divisor);
  Serial.print("Accelerometer data rate (Hz) is approximately: ");
  Serial.println(accel_rate);

  //  icm.setGyroRateDivisor(255);
  uint8_t gyro_divisor = icm.getGyroRateDivisor();
  float gyro_rate = 1100 / (1.0 + gyro_divisor);

  Serial.print("Gyro data rate divisor set to: ");
  Serial.println(gyro_divisor);
  Serial.print("Gyro data rate (Hz) is approximately: ");
  Serial.println(gyro_rate);
  Serial.println();
}

void loop()
{
  //  /* Get a new normalized sensor event */
  sensors_event_t accel;
  sensors_event_t gyro;
  sensors_event_t temp;
  icm.getEvent(&accel, &gyro, &temp);

  Serial.print("\t\tTemperature ");
  Serial.print(temp.temperature);
  Serial.println(" deg C");

  /* Display the results (acceleration is measured in m/s^2) */
  Serial.print("\t\tAccel X: ");
  Serial.print(accel.acceleration.x);
  Serial.print(" \tY: ");
  Serial.print(accel.acceleration.y);
  Serial.print(" \tZ: ");
  Serial.print(accel.acceleration.z);
  Serial.println(" m/s^2 ");

  /* Display the results (acceleration is measured in m/s^2) */
  Serial.print("\t\tGyro X: ");
  Serial.print(gyro.gyro.x);
  Serial.print(" \tY: ");
  Serial.print(gyro.gyro.y);
  Serial.print(" \tZ: ");
  Serial.print(gyro.gyro.z);
  Serial.println(" radians/s ");
  Serial.println();

  delay(100);

  //  Serial.print(temp.temperature);
  //
  //  Serial.print(",");
  //
  //  Serial.print(accel.acceleration.x);
  //  Serial.print(","); Serial.print(accel.acceleration.y);
  //  Serial.print(","); Serial.print(accel.acceleration.z);
  //
  //  Serial.print(",");
  //  Serial.print(gyro.gyro.x);
  //  Serial.print(","); Serial.print(gyro.gyro.y);
  //  Serial.print(","); Serial.print(gyro.gyro.z);

  //  Serial.println();
  //
  //  delayMicroseconds(measurement_delay_us);
}