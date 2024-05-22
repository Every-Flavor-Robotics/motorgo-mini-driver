========================================
Get started writing code for the MotorGo
========================================

The MotorGo is programmable in the Arduino IDE or in PlatformIO. At it's core, the MotorGo is an ESP32-S3 microcontroller with built-in motor drivers, so it can be programmed just like any other ESP32. To make it easy to use the onboard features, we have published custom board definitions for the MotorGo and a MotorGo Mini driver library. The MotorGo Mini board defintions define the onboard led pins, I2C, and SPI pins so that you can use them in your code without having to look up the pin numbers. The MotorGo Mini driver library provides a simple interface for controlling the motors and reading the encoders. You only need to use the driver if you want to use the motors or encoders. If you want to use the onboard peripherals, you can use the MotorGo Mini board definitions without the driver library.

There are 3 tools you need to get started writing code for the MotorGo:

* MotorGo Mini Board Definitions: Let's you program the MotorGo Mini in Arduino/PlatformIO
* MotorGo Mini Driver: Provides functions for controlling the motors with the onboard motor controllers
* MotorGo Mini GUI: Let's you tune your PID controllers over WiFi (optional)

Let's walk through the steps of setting each of these up.

Setting up board definitions
----------------------------

Follow the steps below to set up the board definitions.

First, install the ESP32 board definitions, if you have not already done so.

**Arduino IDE 1.X/2.X**

* Go to `Tools > Board > Boards Manager...`
* Search for `ESP32`
* Install `esp32` by Espressif Systems

**PlatformIO**

* Navigate to PlatformIO Home
* Go to `Platforms`
* Search for `Espressif 32`
* Click `Install`

Next, follow the steps below to setup the MotorGo board definitions.

**Arduino IDE 1.X/2.X**

* Go to `File > Preferences`
* Add `https://raw.githubusercontent.com/Every-Flavor-Robotics/motorgo-board-definitions/main/package_motorgo_index.json` to `Additional Boards Manager URLs`
* Go to `Tools > Board > Boards Manager...`
* Search for `MotorGo`
* Click `Install`
* Select MotorGo > MotorGo Mini 1 (ESP32-S3) from the list of boards (Tools > Boards) to use the board.

**PlatformIO**

Run the following commands to set up the MotorGo board definitions in PlatformIO.

.. code-block:: bash

  git clone https://github.com/Every-Flavor-Robotics/motorgo-board-definitions.git
  cd motorgo-board-definitions
  python3 setup_platformio.py    

To use the board definition, either select the MotorGo Mini 1 from the list of boards when creating a new project OR modify the board in your `platformio.ini` file as follows:

.. code-block:: ini

  board = motorgo_mini_1



To test this is working as expected, you can upload the `Blink` example to the MotorGo Mini 1 board. If the onboard LED blinks, you're good to go!

Setting up the MotorGo Mini driver library
------------------------------------------

The MotorGo Mini library provides an easy interface to control motors and read encoders using the onboard hardware. Below are instructions for setting the library up.

**Arduino IDE 1.X/2.X**

* Download the libary by clicking `here <https://github.com/Every-Flavor-Robotics/motorgo-arduino/raw/main/motorgo-mini-driver.zip>`_
* Go to `Sketch > Include Library > Add .ZIP Library...`
* Select the downloaded file

**PlatformIO**
You can reference the github repo directly in your `platformio.ini` file.

.. code-block:: ini

  lib_deps =
    https://github.com/Every-Flavor-Robotics/motorgo-mini-driver.git#v1.0.0
..

You should modify v1.0.0 to whichever release tag you prefer, or switch to #dev for the latest experimental changes. You can find the latest releases on the `Github repo for the driver <https://github.com/Every-Flavor-Robotics/motorgo-mini-driver/releases>`_

Example code is available in the examples directory of the library. You can open examples directly in Arduino by going to `File > Examples > MotorGo Mini Driver` (under `Examples from Custom Libaries`). We recommend continuing to :ref:`calibrate-motors` to step through the process of calibrating the motors and encoders before running other examples.

Setting up the MotorGo Mini GUI
-------------------------------

The MotorGo Mini GUI provides an interface on your computer to wirelessly tune PID controllers on the MotorGo Mini. You can find the latest version of the GUI `here <https://github.com/Every-Flavor-Robotics/motorgo-mini-gui/releases>`_. Download and install the correct version for your operating system (.dmg for Mac, .exe for Windows, and .AppImage or .deb for Linux). 

Check out the `balance_bot` or `tune_controllers` examples for a demonstration of setting up the MotorGo to communicate with the GUI.
