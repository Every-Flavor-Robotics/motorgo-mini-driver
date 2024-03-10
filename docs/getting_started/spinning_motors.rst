.. _spin-two-motors:

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

    MotorGo::MotorGoMini motorgo_mini;
    MotorGo::MotorChannel& motor_left = motorgo_mini.ch0;
    MotorGo::MotorChannel& motor_right = motorgo_mini.ch1;

    MotorGo::ChannelConfiguration config_left;
    MotorGo::ChannelConfiguration config_right;

    MotorGo::PIDParameters velocity_pid_params_left;
    MotorGo::PIDParameters velocity_pid_params_right;

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

        delay(3000);

        // Setup motor parameters

        config_left.motor_config = MotorGo::MotorGoGreen;
        config_left.power_supply_voltage = 5.0;
        config_left.reversed = false;

        config_right.motor_config = MotorGo::MotorGoGreen;
        config_right.power_supply_voltage = 5.0;
        config_right.reversed = true;

        // Setup
        bool calibrate = false;
        motor_left.init(config_left, calibrate);
        motor_right.init(config_right, calibrate);

        // Set velocity controller parameters
        // Setup PID parameters
        velocity_pid_params_left.p = 0.8;
        velocity_pid_params_left.i = 0.01;
        velocity_pid_params_left.d = 0.0;
        velocity_pid_params_left.output_ramp = 10000.0;
        velocity_pid_params_left.lpf_time_constant = 0.11;

        velocity_pid_params_right.p = 0.8;
        velocity_pid_params_right.i = 0.01;
        velocity_pid_params_right.d = 0.0;
        velocity_pid_params_right.output_ramp = 10000.0;
        velocity_pid_params_right.lpf_time_constant = 0.11;

        motor_left.set_velocity_controller(velocity_pid_params_left);
        motor_right.set_velocity_controller(velocity_pid_params_right);

        //   Set closed-loop velocity mode
        motor_left.set_control_mode(MotorGo::ControlMode::Velocity);
        motor_right.set_control_mode(MotorGo::ControlMode::Velocity);

        // Set a velocity limit of 5.0 rad/s for the left motor
        motor_left.set_velocity_limit(5.0);

        //   Enable motors
        motor_left.enable();
        motor_right.enable();
    }

    void loop()
    {
        motor_left.loop();
        motor_right.loop();

        // Left Motor should only spin at 5.0 rad/s, because of the velocity limit
        motor_left.set_target_velocity(10.0);
        motor_right.set_target_velocity(10.0);

        String str = "Velocity - Ch0: " + String(motor_left.get_velocity()) +
                    " Ch1: " + String(motor_right.get_velocity());

        freq_println(str, 10);
    }