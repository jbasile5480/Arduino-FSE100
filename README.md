Solar Tracker (AccelStepper + Servo)
Arduino-based dual-axis solar tracker with calibration, parking, safety limits, and serial debugging
This project controls a small solar-panel mount using:
A 28BYJ-48 stepper motor + ULN2003 board for azimuth (left/right)
A standard hobby servo for elevation (up/down)
Four LDR photoresistors (top-left, top-right, bottom-left, bottom-right)
The AccelStepper library for smooth stepper motion
Automatic calibration + parking routines
Safety bounds to prevent wire wrapping
Serial debugging and manual commands

Features
✔ Dual-axis tracking
Azimuth uses a stepper for fine positioning.
Elevation uses a servo with small incremental adjustments.
✔ Light averaging
A moving average filter smooths LDR noise for stable tracking.
✔ Deadband
Small sensor differences don't cause movement, preventing "twitching."
✔ Auto and manual calibration
A full azimuth sweep finds the brightest angle and zeroes that position.
✔ Parking routine
When it's dark for a set time, the tracker parks the system:
Servo moves to a flat/safe angle.
Stepper rotates back to a safe home/park position.
✔ Cable-wrap protection (azimuth bounds)
The stepper's allowed movement range is restricted to prevent:
Twisting wires
Pulling connectors
Damaging the LDR/servo harness
✔ Serial debugging
Regular status prints + manual commands.

Serial Commands
Command	Action
c	Run calibration sweep
p	Park immediately
u	Unpark and resume tracking
s	Print status instantly

Wiring
Stepper (28BYJ-48 → ULN2003 → Arduino)
ULN2003 IN	Arduino Pin
IN1	8
IN2	9
IN3	10
IN4	11
Servo Signal → pin 6
Power → 5V
GND → GND
LDRs
LDR	Arduino Analog Pin
Top Left	A0
Top Right	A1
Bottom Left	A2
Bottom Right	A3
Make sure each LDR is in a voltage divider with a ~10k resistor.

Calibration Overview
When calibration is run:
The stepper performs a full (or partial) revolution in increments.
At each step, it reads all four LDRs and records total brightness.
The brightest position is saved as the new "home zero" position.
The stepper returns there and zeroes the position counter.
This ensures the azimuth axis starts pointing toward the sun's average direction.
Tracking Logic
Azimuth (stepper)
azDiff = (left LDRs) - (right LDRs)

If abs(azDiff) exceeds the deadband, a proportional number of steps is queued.

AccelStepper handles acceleration + deceleration automatically.

Elevation (servo)
elDiff = (top LDRs) - (bottom LDRs)

A small bounded delta adjusts the servo angle (with limits such as 20°–170°).

Parking Behavior
Triggered when total light stays below a threshold for a specified duration (e.g., 30 seconds).
Parking actions:
Move servo to a resting angle (e.g., downwards).
Move stepper back to the configured park/home position.
Disable coil power to reduce heat.
Tracking pauses until unparked.
Azimuth Safety Bounds (Prevent Wire Wrap)
To prevent cable twisting, define limits around the calibrated zero point:

Example:

AZ_MIN_POS = -7000
AZ_MAX_POS = +7000


Any move that would exceed these limits is automatically clamped.
This guarantees the stepper never rotates far enough to rip out wires or wrap the harness.

File Structure
/SolarTracker
├── SolarTracker.ino    (main firmware)
└── README.txt           (this file)

Libraries Required
Install through Arduino Library Manager:
AccelStepper by Mike McCauley
Servo (built-in)
