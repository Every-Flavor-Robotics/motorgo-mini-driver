.. _calibrate-motors:


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

You can open this sketch from the MotorGo Mini Driver examples in the Arduino IDE or by `downloading this project <https://github.com/Every-Flavor-Robotics/motorgo-mini-driver/tree/main/examples/calibrate_motors>`_ for PlatformIO.

This sketch will run the calibration routine for channel 0 and channel 1 on the MotorGo.

- The calibration sweep involves a slow, 360-degree rotation of the rotor in both directions.
- This process maps the rotor's position to the encoder's readings.
- The sweep is initiated every time the code is run. If you flash calibration code to the board, it will perform a calibration every time the board resets.

.. code-block:: c++

    #include <Arduino.h>

    #include "motorgo_mini.h"

    MotorGo::MotorGoMini motorgo_mini;
    MotorGo::MotorChannel& motor_ch0 = motorgo_mini.ch0;
    MotorGo::MotorChannel& motor_ch1 = motorgo_mini.ch1;

    MotorGo::ChannelConfiguration config_ch0;
    MotorGo::ChannelConfiguration config_ch1;

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
        config_ch0.motor_config = MotorGo::MotorGoGreen;
        config_ch0.power_supply_voltage = 5.0;

        config_ch1.motor_config = MotorGo::MotorGoGreen;
        config_ch1.power_supply_voltage = 5.0;

        // Setup Ch0
        bool calibrate = true;
        motor_ch0.init(config_ch0, calibrate);
        motor_ch1.init(config_ch1, calibrate);
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

If you are using PlatformIO, you can configure the project with the following platformio.ini file.

.. code-block:: ini

    [env:calibrate_motors]
    platform = platformio/espressif32@^6.1.0
    board = motorgo_mini_1
    framework = arduino
    monitor_speed = 115200
    lib_deps =
        https://github.com/Every-Flavor-Robotics/motorgo-mini-driver.git#v1.0.0


Let's break the code down line by line. You can segment all code for the MotorGo into three parts:

1. Define the MotorGo variables
2. Initialize and configure the motor controllers
3. Write motor commands and read encoder data


**1. Define the MotorGo variables**

This section defines the MotorGo variables and the motor channels. The MotorGoMini class is used to define the MotorGo object, and the MotorChannel class is used to define the motor channels.
The MotorGoMini object provides access to the motor channels, which are used to control each motor independently. In the code snippet, the `motorgo_mini` object is an instance of the MotorGoMini class.

The MotorChannel class represents a single motor channel and is used to configure and control the motor. In the code snippet, `motor_ch0` and `motor_ch1` are instances of the MotorChannel class, representing channel 0 and channel 1 respectively.

To configure each motor channel, we use a `ChannelConfiguration` object. This object contains information about the motor being controlled, such as the motor configuration and the power supply voltage. In the code snippet, `config_ch0` and `config_ch1` are instances of the `ChannelConfiguration` class, representing the configuration for channel 0 and channel 1 respectively.

**2. Initialize and configure the motor controllers**
After defining the MotorGo variables, the next step is to initialize and configure the motor controllers. This involves setting up the motor parameters and performing calibration.

- The `config_ch0` and `config_ch1` objects of the `ChannelConfiguration` class are used to configure the motor channels.
- The `config_ch0.motor_config` and `config_ch1.motor_config` variables specify the motor configuration. In this case, the `MotorGoGreen` configuration is used, which provides specifications for the official MotorGoGreen motor. However, you can create your own motor configuration to define specifications for other motors as well.
- The `config_ch0.power_supply_voltage` and `config_ch1.power_supply_voltage` variables specify the power supply voltage for each motor channel. In the example, we assume the board is powered over USB at 5 volts. However, if you are using the screw terminals to provide power, you can set a higher voltage.

The `init` function sets up the motor using the `ChannelConfiguration` and optionally performs calibration as detailed below.

**Calibration**

Calibration is required each time a new motor/encoder is connected or when the motor encoder combo is reassembled. However, once calibration is performed, it is saved and only needs to be run once. The calibration process determines the direction of the motor and maps the rotor's position to the encoder's readings, ensuring accurate rotor position mapping.

After the setup() function is executed, the motor controllers are initialized and calibrated, ready to execute motor commands and read encoder data in the loop() function.

**3. Write motor commands and read encoder data**
Finally, the loop() section of the code snippet is where motor commands are written and encoder data is read. The `loop()` function is called on each motor channel to update the motor controls and read encoder data. It is important to note that the `loop()` function should be executed as fast as possible without any delays.

To ensure that the `loop()` function is executed at a high frequency, we use the convenience function `freq_println()`, which will print output at a specified frequency. In this case, it is printing the velocity of each motor channel at a frequency of 10 Hz. Spinning the motors by hand should result in a change in velocity.

While not in this example, the loop is also where you would compute a target for the motor to command (as a position, velocity, voltage, or torque). As you will see in other examples, you can configure PID controllers to track those targets, which would be executed in the loop as well.
