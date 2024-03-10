========================================
Get started writing code for the MotorGo
========================================

The MotorGo is programmable in the Arduino IDE or in PlatformIO. At it's core, the MotorGo is an ESP32-S3 microcontroller with built-in motor drivers, so it can be programmed just like any other ESP32. To make it easy to use the onboard features, we have published custom board definitions for the MotorGo and a MotorGo Mini driver library. The MotorGo Mini board defintions define the onboard led pins, I2C, and SPI pins so that you can use them in your code without having to look up the pin numbers. The MotorGo Mini driver library provides a simple interface for controlling the motors and reading the encoders. You only need to use the driver if you want to use the motors or encoders. If you want to use the onboard peripherals, you can use the MotorGo Mini board definitions without the driver library.


Let's walk through the steps to get started writing code for the MotorGo.

Setting up board definitions
----------------------------

The board definitions are available `here <https://github.com/Every-Flavor-Robotics/motorgo-board-definitions>`_. These will soon be available through the Arduino and PlatformIO board managers. For now, you can download the board definitions and install them manually.

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

Next, you'll have to install the MotorGo board definition, which is available in this respository.

**Arduino IDE 1.X/2.X**

* Go to `File > Preferences`
* Add `https://raw.githubusercontent.com/Every-Flavor-Robotics/motorgo-board-definitions/main/package_motorgo_index.json` to `Additional Boards Manager URLs`
* Go to `Tools > Board > Boards Manager...`
* Search for `MotorGo`
* Click `Install`

**PlatformIO**

* Clone this respotory
* Navigate to the root of the this repository
* Run the python script `python3 setup_platformio.py`


If you're in Arduino IDE, you should be able to now select the MotorGo Mini 1 from the board list. If you're in PlatformIO, you should be able to select the MotorGo Mini 1 from the board list OR modify the `board` in `platformio.ini` to `motorgo_mini_1`. To test this is working as expected, you can upload the `Blink` example to the MotorGo Mini 1 board. If the onboard LED blinks, you're good to go!

Setting up the MotorGo Mini driver library
------------------------------------------

The MotorGo Mini library provides an easy interface to control motors and read encoders using the onboard hardware. The main code for the library is available `here <https://github.com/Every-Flavor-Robotics/motorgo-mini-driver>`. The Arduino compatible library is available `here <https://github.com/Every-Flavor-Robotics/motorgo-arduino>`. Below are instructions for setting the library up.

**Arduino IDE 1.X/2.X**

* Download the libary by clicking `here <https://github.com/Every-Flavor-Robotics/motorgo-arduino/raw/main/motorgo-mini-driver.zip>`
* Go to `Sketch > Include Library > Add .ZIP Library...`
* Select the downloaded file

**PlatformIO**
You can reference the github repo directly in your `platformio.ini` file.

```ini
lib_deps =
    https://github.com/Every-Flavor-Robotics/motorgo-mini-driver.git#v1.0.0
```

You can modify v1.0.0 to whichever release tag you prefer, or switch to dev for the latest experimental changes. Example code is available in the examples directory of the library. You can open examples directly in Arduino by going to `File > Examples > MotorGo Mini Driver` (under `Examples from Custom Libaries`). We recommend continuing to :ref:`calibrate-motors` to step through the process of calibrating the motors and encoders before running other examples.
