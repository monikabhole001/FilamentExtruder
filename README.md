# Filament Extruder Temperature Control

This project implements a PID controller for temperature regulation of a filament extruder machine using an Arduino.

## Description

The filament extruder machine consists of:
- Motor
- Hopper
- Barrel
- Heater
- Die

Resin pellets are fed from the hopper and transported towards the heater by motors. The system controls the heater temperature using a PID algorithm based on readings from a MAX6675 thermocouple sensor. The current and set temperatures are displayed on an I2C LCD.

## Features

- Temperature reading via MAX6675 thermocouple module
- PID control of heater using PWM signal
- I2C LCD display showing setpoint and real temperature
- Adjustable PID constants for tuning

## Hardware Used

- Arduino Uno
- MAX6675 thermocouple module
- I2C LCD display (16x2)
- MOSFET for heater control via PWM
- Rotary Encoder (optional for setting RPM/temperature)
- Power supply, relays, SMPS, heater element

## Pin Connections

| Module       | Arduino Pin  |
|--------------|--------------|
| MAX6675 CS   | D10          |
| MAX6675 SO   | D12          |
| MAX6675 SCK  | D13          |
| I2C LCD SDA  | A4           |
| I2C LCD SCL  | A5           |
| Heater PWM   | D3           |

## Usage

- Load the FilamentExtruderPID.ino sketch onto your Arduino.
- Adjust the set_temperature variable in the code to the desired temperature.
- Monitor the temperature on the LCD and adjust PID constants if needed for stable control.

## Internship Experience

During the internship, I also worked on:
- Designing parts using SolidWorks
- 3D printing components
- Using relays and SMPS units in the system
- Integrating rotary encoders for motor speed control
- Recycling materials like PLA and ABS

---

Feel free to contribute or raise issues.

---

**Author:** Monika Bhole  
**Date:** 2021

