Arduino based dual axis light tracking solar panel

This project controls a small solar panel mount using:
  A 28ByJ-48 stepper motor + a ULN2003 board for azimuth control
  A standard hobby ervo motor for elevation
  4 LDR photoresistors

This project takes the inputs from the photoresistors and averages the values to gain a light average for each side. Then, the code will move the stepper and the servo in unison until the panel is pointing directly at the brightest light source.

Panel will park after a set time (30s) and revert to a set/calibrated home position.

Serial Commands:
"c" -Run azimuth calibration sweep
"p" -Park command, returning the system to home positions
"u" -Unpark and resume tracking
"s" -Print instant status

Calibration Overview
When calibration is run:
The stepper performs a full (or partial) revolution in increments, at each step, it reads all four LDRs and records the total brightness. The brightest position is saved as the new "home zero" position, and the stepper returns there and zeroes the position counter.

Required Libraries:
  -AccelStepper
  -Built in Servo Library
