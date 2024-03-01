=================
Motor Calibration
=================

**Calibration Overview**

This section outlines the encoder calibration process for the MotorGo, which is crucial for accurate rotor position mapping in relation to the encoder.

The calibration step also automatically determines the direction of the motor. After calibration, any motor should spin the same direction when given the same command. However, if you flip the direction the motor is plugged in, you will have to re-calibrate!

**Pre-requisites**

- *Encoder Installation*: Ensure that the encoder, along with its magnet, is correctly installed prior to calibration.
- *Software Setup*: The calibration step is performed through the provided software API.


**Calibration Sketch**

This sketch will run the calibration routine for channel 0 on the MotorGo. If you want to calibrate channel 1, simply change the function calls to ``init_ch1`` and ``loop_ch1``.

- The calibration sweep involves a slow, 360-degree rotation of the rotor in both directions.
- This process maps the rotor's position to the encoder's readings.
- The sweep is initiated every time the code is run. If you flash calibration code to the board, it will perform a calibration every time the board resets.

.. code-block:: c++

    #include <Arduino.h>

    #include "motorgo_mini.h"

    MotorGo::MotorGoMini motorgo_mini;
    MotorGo::MotorChannel& motor_ch0 = motorgo_mini.ch0;
    MotorGo::MotorChannel& motor_ch1 = motorgo_mini.ch1;

    MotorGo::MotorParameters motor_params_ch0;
    MotorGo::MotorParameters motor_params_ch1;

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

    // Setup Ch0
    bool calibrate = true;
    motor_ch0.init(motor_params_ch0, calibrate);
    motor_ch1.init(motor_params_ch1, calibrate);
    }

    void loop()
    {
    // Run Ch0
    motor_ch0.loop();
    motor_ch1.loop();

    String str = "Velocity - Ch0: " + String(motor_ch0.get_velocity()) +
                " Ch1: " + String(motor_ch1.get_velocity());

    freq_println(str, 10);
    }


Let's break the code down line by line. You can segment all code for the MotorGo into three parts:

1. Define the MotorGo variables
2. Initialize and configure the motor controllers
3. Write motor commands and read encoder data


To-do: finish this page.
