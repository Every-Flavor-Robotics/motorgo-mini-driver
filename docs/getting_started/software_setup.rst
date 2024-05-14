========================================
Get started writing code for the MotorGo
========================================

The MotorGo is programmable in the Arduino IDE or in PlatformIO. At it's core, the MotorGo is an ESP32-S3 microcontroller with built-in motor drivers, so it can be programmed just like any other ESP32. Follow the steps below to get started writing code for the MotorGo in Arduino or PlatformIO.

There are 3 tools you need to get started writing code for the MotorGo:

* ESP32 Boards: Provides board definitions for the MotorGo
* MotorGo Mini Driver: Provides API/functions for controlling the motors with the onboard motor controllers
* MotorGo Mini GUI: Lets you tune your PID controllers over WiFi (optional)

Let's walk through the steps of setting each of these up.

Arduino IDE
-----------
First, install the ESP32 board definitions, if you have not already done so. You need at least version 2.0.16 of the ESP32 board definitions for the MotorGo to work.

* Go to `Tools > Board > Boards Manager...`
* Search for `ESP32`
* Install `esp32` by Espressif Systems

Next, install the MotorGo Mini driver library.

* Download the libary by clicking `here <https://github.com/Every-Flavor-Robotics/motorgo-arduino/raw/main/motorgo-mini-driver.zip>`_
* Go to `Sketch > Include Library > Add .ZIP Library...`
* Select the downloaded file

To test if everything is working as expected, you can upload an example sketch to the board.

* Go to `File > Examples > MotorGo Mini Driver (under Examples from Custom Libaries) > read_encoders`
* Next, select the MotorGo Mini 1 board by clicking `Select Board > Select Other Board and Port > MotorGo Mini 1 (ESP32-S3)`. If you're in Arduino IDE 1.X, you can find in the board in `Tools > Board > MotorGo Mini 1 (ESP32-S3)`
* Click the upload button (right arrow) to upload the sketch to the board

You should see the encoder values printed to the serial monitor. If you see this, you're good to go!

PlatformIO
----------
First, install the ESP32 board definitions, if you have not already done so.

* Navigate to PlatformIO Home
* Go to `Platforms`
* Search for `Espressif 32`
* Click `Install`

To use the board definition, either select the MotorGo Mini 1 from the list of boards when creating a new project OR modify the board in your `platformio.ini` file as follows:

.. code-block:: ini

  board = motorgo_mini_1

Next, to setup the MotorGo Mini driver library, you can reference the github repo directly in your `platformio.ini` file.

.. code-block:: ini

  lib_deps =
    https://github.com/Every-Flavor-Robotics/motorgo-mini-driver.git#v1.0.0
..

You should modify v1.0.0 to whichever release tag you prefer, or switch to #dev for the latest experimental changes. You can find the latest releases on the `Github repo for the driver <https://github.com/Every-Flavor-Robotics/motorgo-mini-driver/releases>`_

You can find an example PlatformIO project for reading the encoder data `here <https://github.com/Every-Flavor-Robotics/motorgo-mini-driver/tree/main/examples/read_encoders>`_


Setting up the MotorGo Mini GUI
-------------------------------

The MotorGo Mini GUI provides an interface on your computer to wirelessly tune PID controllers on the MotorGo Mini. You can find the latest version of the GUI `here <https://github.com/Every-Flavor-Robotics/motorgo-mini-gui/releases>`_. Download and install the correct version for your operating system (.dmg for Mac, .exe for Windows, and .AppImage or .deb for Linux).

Check out the `balance_bot` or `tune_controllers` examples for a demonstration of setting up the MotorGo to communicate with the GUI.
