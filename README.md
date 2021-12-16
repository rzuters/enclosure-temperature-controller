# Enclosure temperature controller

## Description

Enclosure used for 3d printer has built-in controller to sustain constant
environment temperature. PID controlled 12V enclosure fan and 220V AC heater as
a heat source is used to regulate enclosure temperature and DS18B20 temperature
sensor to measure enclosure (env.) temperature. Three buttons, LCD 16x2 and
Bluetooth module provides an option to set desired env. temp. Temperature
measurement unit is in Celsius degrees. Desired minimum and maximum target
temperature which is supported accordingly in a range: +20...+59°.

## Used components

- MCU				        Arduino Uno Rev3
- fan               Noctua NF-P14s (12V)
- relay             JQC-3FF-S-Z 5V (220V heater connected to "Normally Open")
- temp. sensor      DS18B20
- LCD               1602A (16x2) with I2C module
- Bluetooth module	HC-06


## Used code libraries

- mjson
- LiquidCrystal_I2C
- SoftwareSerial
- DallasTemperature
- TimerOne
- PID
- Wire
- OneWire

## Circuit

![Enclosure temperature controller circuit](/assets/images/enclosure_controller_bb.png)
