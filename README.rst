==================
MotorGo Mini Driver
==================

The `MotorGo Mini <https://motorgo.net>`_ is a 2-channel brushless motor controller and microcontroller combo designed to accelerate the process of robot prototype development. MotorGo Mini handles the low-level details of motor control, including power distribution, current sensing, encoder reading, PID controls, and communications, so that you can focus on the high-level details of your robot's behavior.

Key Hardware Features:

- 2-channel brushless motor controller
    - 2.1 A continuous current per channel
    - Field Oriented Control (FOC)
    - Current sensing
- 2-channel EncoderGo encoder support
    - 14 bit resolution, 16384 counts per revolution
- ESP32-S3 microcontroller
    - 240 MHz dual-core processor
    - Wifi and Bluetooh communication support
    - 8 MB flash memory
- 7-11 V input voltage
- Exposed 3.3 V, 5 V, and unregulated voltage rails
- 2x QWIIC connectors for easy sensor integration
- 8 GPIO (PWM) pin for additional sensor and motor integration
- USB-C port

Unlike other motor controllers, you can program the MotorGo Mini to do whatever you want using the Arduino IDE (or PlatformIO). This means that for most basic robots, you don't need another Arduino, Raspberry Pi, or other microcontroller to control your robot. The MotorGo Mini can do it all. For more complex robots, the MotorGo can still offload much of the low-level computation from your main computers, so they have more headroom for the high-level tasks.

Key Software Features:

- Automatic wireless multi-board communication
- Automatic motor pole pair discovery
- Encoder magnet eccentricity compensation
- PID Tuning via web interface
- (Planned) ROS/ROS2 support

The MotorGo Mini Driver is the software that runs on the ESP32 onboard to enable all of the features. Follow the Getting Started guide to get the driver up and running.

.. end_intro

Installation
============
MotorGo supports development in PlatformIO and Arduino IDE.

Follow the instructions [here](https://github.com/Every-Flavor-Robotics/motorgo-board-definitions) to install the MotorGo board definitions and this driver.
