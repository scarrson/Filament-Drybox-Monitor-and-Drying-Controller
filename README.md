# Filament Drybox Monitor and Drying Controller

This code was developed for use on a Raspberry Pi Pico W running CircuitPython. A Pico W communicates with an HTU31 sensor (temphumidity) via I2C and displays pertinent information to an OLED display and also to MQTT for remote monitoring. The Pico also controls a 120V heater and can be configured for any heating setpoint. Some fault detection and crash recoveryerror handling is included.

## Installation

 Install CircuitPython on your Pico W and copy all files from this repository to the CircuitPy drive.
 Note The boot.py file and watchdog can cause your Pico to become unresponsive or constantly reboot, refer to the documentation for the watchdog function and boot.py readwrite control for more info.
 Build the circuit as indicated in the schematic.
 Test and troubleshoot your WiFi, secrets.py, and MQTT configuration until the Pico W successfully connects and uploads all information.

## Usage

 On power-up If the rotary encoder button is NOT pressed, the Pico W will boot with ReadWrite control and the user will be unable to modify any data on the Pico (normal operation, this allows the Pico to write a persistent heating setpoint and recover from crashes). If the rotary encoder button IS pressed, the Pico W will boot without ReadWrite control, and the user will have the opportunity to modify files. Note this will cause the Pico to restart due to the reset and watchdog functionality of the main code, and therefore a user must be quick to remove the code.py or boot.py file while the Pico is writeable. Fun!

 Use the Heat ONOFF switch to control whether the system is monitoring the drybox or actively heating.
       Monitor Mode
         Read TempRH via I2C + calculate dewpoint. Report to OLED and MQTT every n seconds (default 60 sec).
         If the Rotary Encoder hasn't been pressed in n seconds, put it to sleep (default 600 sec).
     Heating Mode
         Read TempRH via I2C + calculate dewpoint. Report to OLED and MQTT every n seconds (default 5 sec).
         Never sleep the OLED.
         User enters a heating setpoint by pressing the rotary encoder and selecting a setpoint. The relay will turn on until the setpoint is reached, using deadband control (ie for a deadband of 2°C and setpoint of 45°C, heating turns off at 46°C and turns on again at 44C°).
         If a FAULT is detected - stop heating, resume when fault clears. Fault conditions are out-of-range temperature values or an I2C device that fails to communicate for n seconds (default 10 sec).
         Indicate status of heating mode active, and heating output active on local LEDs.
         If Pico W crashes during heating, watchdog detects and resets board. On bootup, Heat ONOFF switch in ON position, code recovers setpoint from file and resumes heating indefinitely.
 MQTT Variables Reported
       Statuses Fault, Heating Mode, Heating Output
       Analog Values Fan RPM, Temperature, Relative Humidity, Dewpoint, Setpoint
       Error Monitoring Reboots, Num of I2C errors per sample cycle


## Bill of Materials

 1 x Raspberry Pi Pico W
     0.1 uf and 10 uf capacitor
 Fused IEC Power Entry Module
 5-15R outlet
 120VAC - 12VDC PSU (~1A)
     Fuseholder for 120VAC input
 DPDT switch
 5V Relay shield with low-level trigger (minimum 10A rated contacts)
 120VAC heater (up to 500W suggested)
 7805 - 5V linear regulator
 SSD1306 on breakout board for SPI
 Adafruit 4832 HTU31 breakout
 2 x PN2222A BJT transistors
 2 x 5-12V panel mount LED's
 14W Resistors 0.33k, 1k, 3.3k, 10k
 CPU Fan (40mm, 80mm, or 120mm) with tachometer pin
 RJ45-to-RJ45 bulkhead
 120VAC extension cable and plugs for box entry
 Watertight strain relief connnector for 120VAC extension cable.
 Rotary encoder with push button.
 Digikey Solderful prototyping board
 Various sta-kon connectors and 0.100 pitch Dupont style connectors and pins

## 3D Printing Files
Refer to the [Printables submission] for the most up-to-date 3D modelling files.

[] # (These are reference links used in the body of this note and get stripped out when the markdown processor does its job. There is no need to format nicely because it shouldn't be seen. Thanks SO - httpstackoverflow.comquestions4823468store-comments-in-markdown-syntax)

   [Printables submission] httpswww.printables.commodel570550-filament-drybox-monitor-and-drying-controller-for-
