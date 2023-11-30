===============
Getting Started
===============

Getting motors running with the MotorGo involves a couple of key steps:

1. **Setup the motor hardware and electronics**: MotorGo requires that your EncoderGo is installed to measure the position of your motor and that the MotorGo is connected to a power supply (battery, USB, benchtop power supply, etc).

2. **Setup Arduino IDE or PlatformIO**: MotorGo is designed to work with the Arduino IDE or PlatformIO. You will need to install the MotorGo library to make use of the motor controllers. You will also need to select the correct board and port in the Arduino IDE or PlatformIO.

3. **Calibrate the motor and encoder**: The first time that you set up a motor and EncoderGo pair, you will need to run the calibration routine. This determines the relationship between encoder counts and the motor position. Without this calibration, the MotorGo will not be able to spin the motor at all.

4. **Write your code to control the motor**: This is the last step! Now you can write whatever code you want to control the motor. The exammples section has a bunch of example projects to get you started.

Below are detailed instructions for each of these steps, with code examples that you can follow to get going.

.. toctree::
   :maxdepth: 3

   hardware_setup
   software_setup
   calibration
   spinning_motors
