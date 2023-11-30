=================
Motor Calibration
===========

**Calibration Overview**

This section outlines the encoder calibration process for the MotorGo, which is crucial for accurate rotor position mapping in relation to the encoder.

The calibration step also automatically determines the direction of the motor. After calibration, any motor should spin the same direction when given the same command. However, if you flip the direction the motor is plugged in, you will have to re-calibrate!

**Pre-requisites**

- *Encoder Installation*: Ensure that the encoder, along with its magnet, is correctly installed prior to calibration.
- *Software Setup*: The calibration step is performed through the provided software API.


**Calibration Sketch**

This sketch will run the calibration routine for channel 0 on the MotorGo. If you want to calibrate channel 1, simply change the function calls to ``init_ch1`` and ``loop_ch1``.

* The calibration sweep involves a slow, 360-degree rotation of the rotor in both directions.
* This process maps the rotor's position to the encoder's readings.
* The sweep is initiated every time the code is run. If you flash calibration code to the board, it will perform a calibration every time the board resets.

.. code-block:: c++

    // Include Arduino and motorgo mini driver library
    #include <Arduino.h>

    #include "motorgo_mini.h"

    ////// MotorGo Variables //////
    MotorGo::MotorGoMini* motorgo_mini;
    MotorGo::MotorParameters motor_params_ch0;

    // Utility function to print at a maximum frequency
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
        // Initialize serial port
        Serial.begin(115200);

        // Setup the motor parameter
        // These are the default parameters for the MotorGo brushless gimbal motor
        motor_params_ch0.pole_pairs = 7;
        motor_params_ch0.power_supply_voltage = 5.0;
        motor_params_ch0.voltage_limit = 5.0;
        motor_params_ch0.current_limit = 300;
        motor_params_ch0.velocity_limit = 100.0;
        motor_params_ch0.calibration_voltage = 2.0;

        // Instantiate motorgo mini board
        motorgo_mini = new MotorGo::MotorGoMini();

        // Run calibration on Channel 0
        bool calibrate = true;
        bool enable_foc_studio = false;
        motorgo_mini->init_ch0(motor_params_ch0, calibrate, enable_foc_studio);

    }

    void loop()
    {
        // Call loop to update motor commands and encoder data
        motorgo_mini->loop_ch0();

        motorgo_mini->set_target_velocity_ch0(10.0);
        motorgo_mini->set_target_velocity_ch1(10.0);

        String str = "Velocity - Ch0: " + String(motorgo_mini->get_ch0_velocity()) +
                    " Ch1: " + String(motorgo_mini->get_ch1_velocity());

        freq_println(str, 10);
    }

Let's break the code down line by line. You can segment all code for the MotorGo into three parts:

1. Define the MotorGo variables
2. Initialize and configure the motor controllers
3. Write motor commands and read encoder data


To-do: finish this page.
