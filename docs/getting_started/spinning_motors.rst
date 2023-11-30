===================
Spinning Two Motors
===================

**Overview**

This section explains how to operate the motors using the MotorGo system in various modes, including velocity, velocity open loop, position, position open loop, and torque modes. A crucial part of this process is enabling the motors and setting the desired control mode.

*Pre-requisites*

- Calibration: Ensure that the motors have been calibrated as outlined in the previous section. Calibration can be performed during this step or previously, and then disabled if it has already been completed.

**Motor Operation Process**

*Full Code Example:*
The following code initializes both motor controller channels for the MotorGo brushless gimbal motor. It then enables the motors and configures them in velocity control mode with a basic PID controller. Finally, in the loop, the motors are commanded to run at 10 rad/s and the velocity is printed to the serial terminal.

.. code-block:: c++

    #include <Arduino.h>

    #include "motorgo_mini.h"

    MotorGo::MotorGoMini* motorgo_mini;
    MotorGo::MotorParameters motor_params_ch0;
    MotorGo::MotorParameters motor_params_ch1;
    MotorGo::PIDParameters velocity_pid_params;

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

        // Instantiate motorgo mini board
        motorgo_mini = new MotorGo::MotorGoMini();

        // Setup Ch0 and Ch1 with FOCStudio disabled
        bool calibrate = false;
        bool enable_foc_studio = false;
        motorgo_mini->init_ch0(motor_params_ch0, calibrate, enable_foc_studio);
        motorgo_mini->init_ch1(motor_params_ch1, calibrate, enable_foc_studio);


        // Set velocity controller parameters
        // Setup PID parameters
        velocity_pid_params.p = 1.0;
        velocity_pid_params.i = 0.01;
        velocity_pid_params.d = 0.0;
        velocity_pid_params.output_ramp = 10000.0;
        velocity_pid_params.lpf_time_constant = 0.11;


        motorgo_mini->set_velocity_controller_ch0(velocity_pid_params);
        motorgo_mini->set_velocity_controller_ch1(velocity_pid_params);


        //   Set closed-loop velocity mode
        motorgo_mini->set_control_mode_ch0(MotorGo::ControlMode::Velocity);
        motorgo_mini->set_control_mode_ch1(MotorGo::ControlMode::Velocity);


        motorgo_mini->enable_ch0();
        motorgo_mini->enable_ch1();
    }
    void loop()
        {
        // Run Ch0 and Ch1
        motorgo_mini->loop_ch0();
        motorgo_mini->loop_ch1();


        // set target velocity (rad/s)
        motorgo_mini->set_target_velocity_ch0(10.0);
        motorgo_mini->set_target_velocity_ch1(10.0);


        // print velocity in serial terminal
        String str = "Velocity - Ch0: " + String(motorgo_mini->get_ch0_velocity()) +
                    " Ch1: " + String(motorgo_mini->get_ch1_velocity());
        freq_println(str, 10);
    }


