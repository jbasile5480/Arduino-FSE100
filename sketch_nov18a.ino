// Solar tracker with AccelStepper + Servo, calibration, parking, serial debug.
// This version uses a SOFTWARE HOMING sequence (moving to the mechanical stop)
// to define absolute azimuth bounds and prevent wire wrapping.
//
// - Azimuth: 28BYJ-48 via ULN2003 driven with AccelStepper (HALF4WIRE)
// - Elevation: hobby servo
// - LDRs on A0..A3 as TL, TR, BL, BR.
// - ULN2003 IN1..IN4 on pins STEPPER_PIN[0..3].
//
// !!! WARNING !!!
// This method relies on forcing the motor to stall against its mechanical stop.
// This is generally safe for small, low-power motors like the 28BYJ-48,
// but should be done at reduced speed and acceleration to minimize strain.
// !!! WARNING !!!

#include <AccelStepper.h>
#include <Servo.h>
#include <EEPROM.h>

// --- Pins ---
// analog pins (photoresistors)
const int LDR_TL = A0;
const int LDR_TR = A1;
const int LDR_BL = A2;
const int LDR_BR = A3;

// servo digital pin
const int SERVO_PIN = 6;

// stepper motor pins IN1,2,3,4 on ULN2003 input pins
const int STEPPER_PIN[4] = {8, 9, 10, 11};

// --- AccelStepper (HALF4WIRE used for stepper motor half-step) ---
// PIN ORDER REVERSED HERE: {3], [2], [1], [0] instead of {0], [1], [2], [3]
// This reverses the motor direction to fix the orientation issue.
AccelStepper azStepper(AccelStepper::HALF4WIRE,
                       STEPPER_PIN[3], STEPPER_PIN[2],
                       STEPPER_PIN[1], STEPPER_PIN[0]);

// --- Settings (tweakable) ---
const int NUM_AVG = 10;            // moving average window for LDRs

// Deadbands (deadspace / space of non-movement)
const int AZ_DEADBAND = 40;        // ADC units deadband for azimuth
const int EL_DEADBAND = 40;        // ADC units deadband for elevation

// How aggressively ADC diff translated to steps per loop
const int AZ_DIFF_TO_STEP_DIV = 20; // larger -> less aggressive
const int MAX_STEPS_PER_CYCLE = 400; // maximum relative steps requested per loop (clamped)

// servo travel limits & center
const int SERVO_MIN = 40;           // servo lower limit [deg]
const int SERVO_MAX = 190;          // servo upper limit [deg]
const int SERVO_CENTER = 90;        // center angle at startup
const int SERVO_STEP_PER_DIFF = 80; // larger => smaller servo movement per ADC unit

// AccelStepper motion tuning
const float STEPPER_MAX_SPEED = 3000.0;   // [steps per second]
const float STEPPER_ACCEL = 1000.0;        // [steps per second squared]
const float STEPPER_HOME_SPEED = 800.0;   // Slower speed for finding mechanical stop

const long STEPS_PER_REV = 2048L; // assumed steps per rotation (28BYJ-48)

// --- Azimuth limiting (prevents cable wrap and wire jumble area) ---
// AZ_MIN_POS is 0, set by the 'h' (Homing) command hitting the mechanical stop.
const bool AZ_LIMITS_ENABLED = true;
const long AZ_MIN_POS = 0;     
// AZ_MAX_POS defines the maximum safe position, beyond which the tracker
// enters the cable jumble zone or risks twisting wires.
// It is adjusted via the 'B' command (Set Boundary).
long AZ_MAX_POS = 2048;  // Default: slightly less than 2 full revs (2 * 2048 = 4096)

// --- Calibration settings ---
const bool AUTO_CALIBRATE_ON_BOOT = false; // set true to home and scan on boot
const int CAL_SCAN_STEP = 16;  // coarse step increment during scan
const int CAL_SCAN_DELAY_MS = 5; // delay between step moves during scan
const long HOME_BIND_DISTANCE = STEPS_PER_REV * 3; // Steps to ensure motor hits stop

// --- Parking / night detection ---
const int PARK_SERVO_ANGLE = SERVO_MIN;    // servo angle when parked
const long PARK_AZ_REL_POS = 0;            // parking azimuth relative to *calibrated sun*
const int PARK_LIGHT_THRESHOLD = 150;      // if total light below this => consider dark
const unsigned long PARK_LIGHT_TIME_MS = 30UL * 1000UL; // 30s

// --- Debug / serial ---
bool SERIAL_DEBUG = true;
unsigned long lastDebugMillis = 0;
const unsigned long DEBUG_INTERVAL_MS = 300;

// --- EEPROM addresses ---
const int EEPROM_SOLAR_HOME_ADDR = 0;   // 4 bytes (long)
const int EEPROM_SERVO_ADDR      = 8;   // 2 bytes (int) at offset
const int EEPROM_AZ_MAX_ADDR     = 12;  // 4 bytes (long) for max limit

// --- Globals ---
Servo tiltServo;
int ldrBuffers[4][NUM_AVG];
int ldrIdx = 0;
int currentServo = SERVO_CENTER;

bool isParked = false;
unsigned long lightBelowSince = 0;
bool calibrated = false; // Tracks if we have run a light-scan

// This stores the STEPPER POSITION (absolute) that represents the "sun home"
long solarHomePosition = (AZ_MAX_POS - AZ_MIN_POS) / 2; // Default to center

// --- Forward declarations ---
int avgRead(int sensorPin, int bufferIndex);
void forceStepperLowOutputs();
long clampAzTarget(long target);
void homeAzimuth();
void scanForSun();
void runFullCalibration();
void parkNow();
void unparkNow();
void printStatusNow(int tl, int tr, int bl, int br, int azDiff, int elDiff);
void saveSolarHome();
void saveServoAngle();
void saveAzMaxPos();
void loadSavedSettings();
void printHelp();

void setup() {
  Serial.begin(115500); // Increased baud rate for faster logging
  delay(50);
  
  // configure stepper
  azStepper.setMaxSpeed(STEPPER_MAX_SPEED);
  azStepper.setAcceleration(STEPPER_ACCEL);

  // ensure stepper pins are outputs
  for (int i = 0; i < 4; ++i) {
    pinMode(STEPPER_PIN[i], OUTPUT);
    digitalWrite(STEPPER_PIN[i], LOW);
  }

  // initialize LDR buffers
  for (int b = 0; b < NUM_AVG; ++b) {
    ldrBuffers[0][b] = analogRead(LDR_TL);
    ldrBuffers[1][b] = analogRead(LDR_TR);
    ldrBuffers[2][b] = analogRead(LDR_BL);
    ldrBuffers[3][b] = analogRead(LDR_BR);
    delay(5);
  }

  // servo
  tiltServo.attach(SERVO_PIN);

  // load persisted settings (solar home position, servo angle, AZ_MAX_POS)
  loadSavedSettings();
  tiltServo.write(currentServo);

  // Initialize stepper position to the middle of the range if we are not using
  // a physical home switch. The user must run 'h' or 'c' to set the true '0'.
  azStepper.setCurrentPosition((AZ_MAX_POS + AZ_MIN_POS) / 2);

  if (SERIAL_DEBUG) {
    Serial.println();
    Serial.println("Solar Tracker with Software Limits started");
    printHelp(); // Print help menu
    Serial.print("Az limits enabled: "); Serial.println(AZ_LIMITS_ENABLED ? "YES" : "NO");
    if (AZ_LIMITS_ENABLED) {
      Serial.print("ABSOLUTE AZ limits: "); Serial.print(AZ_MIN_POS); Serial.print(" .. "); Serial.println(AZ_MAX_POS);
      Serial.print("Loaded solarHomePosition (absolute steps) = "); Serial.println(solarHomePosition);
    }
  }

  // optional: run auto calibration at boot
  if (AUTO_CALIBRATE_ON_BOOT) {
    delay(200); // allow sensor settle
    runFullCalibration();
  }
}

void saveAzMaxPos() {
  EEPROM.put(EEPROM_AZ_MAX_ADDR, AZ_MAX_POS);
  if (SERIAL_DEBUG) {
    Serial.print("Saved AZ_MAX_POS = ");
    Serial.println(AZ_MAX_POS);
  }
}

void loadSavedSettings() {
  // Read stored solar home position (long)
  EEPROM.get(EEPROM_SOLAR_HOME_ADDR, solarHomePosition);
  
  // NEW: Read stored AZ_MAX_POS (long)
  long tempMaxPos = 0;
  EEPROM.get(EEPROM_AZ_MAX_ADDR, tempMaxPos);

  // If the loaded AZ_MAX_POS is a reasonable number, use it, otherwise use the default constant
  if (tempMaxPos > AZ_MIN_POS && tempMaxPos < 8000) { // arbitrary sanity check
      AZ_MAX_POS = tempMaxPos;
  }
  
  // Validate loaded solar home position against the potentially new bounds
  if (solarHomePosition < AZ_MIN_POS || solarHomePosition > AZ_MAX_POS) {
    solarHomePosition = (AZ_MAX_POS - AZ_MIN_POS) / 2; // Default to center
    EEPROM.put(EEPROM_SOLAR_HOME_ADDR, solarHomePosition);
  }

  // Read servo angle (int)
  int s = 0;
  EEPROM.get(EEPROM_SERVO_ADDR, s);
  if (s < SERVO_MIN || s > SERVO_MAX) s = SERVO_CENTER;
  currentServo = s;
}

void saveSolarHome() {
  EEPROM.put(EEPROM_SOLAR_HOME_ADDR, solarHomePosition);
  if (SERIAL_DEBUG) {
    Serial.print("Saved solarHomePosition (absolute steps) = ");
    Serial.println(solarHomePosition);
  }
}

void saveServoAngle() {
  EEPROM.put(EEPROM_SERVO_ADDR, currentServo);
  if (SERIAL_DEBUG) {
    Serial.print("Saved servo angle = ");
    Serial.println(currentServo);
  }
}

int avgRead(int sensorPin, int bufferIndex) {
  // Read the current sensor value and update the buffer
  ldrBuffers[bufferIndex][ldrIdx] = analogRead(sensorPin);
  
  // Calculate the sum of values in the buffer
  long s = 0;
  for (int i = 0; i < NUM_AVG; ++i) s += ldrBuffers[bufferIndex][i];
  
  // Return the average
  return (int)(s / NUM_AVG);
}

void forceStepperLowOutputs() {
  // Explicitly set motor pins low to stop holding torque and save power
  for (int i = 0; i < 4; ++i) digitalWrite(STEPPER_PIN[i], LOW);
}

// Function to clamp the target position to the absolute software bounds.
long clampAzTarget(long target) {
  if (!AZ_LIMITS_ENABLED) return target;
  if (target < AZ_MIN_POS) return AZ_MIN_POS;
  if (target > AZ_MAX_POS) return AZ_MAX_POS;
  return target;
}

// Software Homing: Moves the stepper until it hits the mechanical stop.
void homeAzimuth() {
  if (SERIAL_DEBUG) Serial.println("Homing: Moving to physical MIN stop (software limit)...");
  
  // Use slower speed for pushing against the stop
  azStepper.setMaxSpeed(STEPPER_HOME_SPEED);
  azStepper.setAcceleration(STEPPER_ACCEL / 2);
  
  // Set a blocking move far outside the range to ensure it hits the stop.
  azStepper.moveTo(AZ_MIN_POS - HOME_BIND_DISTANCE);
  
  // Run until the target is reached (or motor stalls against the mechanical stop)
  while (azStepper.distanceToGo() != 0) {
    azStepper.run();
  }
  
  // At this point, the motor should be physically pressed against the MIN stop.
  // Set the current position to the logical start (AZ_MIN_POS = 0)
  azStepper.setCurrentPosition(AZ_MIN_POS); 
  
  if (SERIAL_DEBUG) Serial.println("Homing: Assumed physical MIN stop reached. Stepper position 0 set.");
  
  // Move slightly *off* the stop (e.g., 20 steps) to reduce power draw/strain
  azStepper.moveTo(AZ_MIN_POS + 20); 
  while (azStepper.distanceToGo() != 0) azStepper.run();
  
  // Restore normal motor speeds
  azStepper.setMaxSpeed(STEPPER_MAX_SPEED);
  azStepper.setAcceleration(STEPPER_ACCEL);
}

// Scans for the brightest light *within the safe bounds* (AZ_MIN_POS to AZ_MAX_POS).
void scanForSun() {
  if (SERIAL_DEBUG) Serial.println("Calibration: starting sun scan...");
  delay(200); // settle

  long bestPos = azStepper.currentPosition();
  long bestLight = -1;

  // Move to the starting bound (AZ_MIN_POS) to begin the scan
  azStepper.moveTo(AZ_MIN_POS);
  while (azStepper.distanceToGo() != 0) azStepper.run();

  long stepsToScan = AZ_MAX_POS - AZ_MIN_POS;
  long stepsMoved = 0;

  // Move in positive direction to sample across the entire allowed range
  while (stepsMoved < stepsToScan) {
    int tl = avgRead(LDR_TL, 0);
    int tr = avgRead(LDR_TR, 1);
    int bl = avgRead(LDR_BL, 2);
    int br = avgRead(LDR_BR, 3);
    ldrIdx = (ldrIdx + 1) % NUM_AVG;

    long totalLight = (long)tl + tr + bl + br;
    if (totalLight > bestLight) {
      bestLight = totalLight;
      bestPos = azStepper.currentPosition();
    }

    // Step forward by coarse increment
    azStepper.move(CAL_SCAN_STEP); 
    while (azStepper.distanceToGo() != 0) {
      azStepper.run();
      delay(CAL_SCAN_DELAY_MS);
    }
    stepsMoved += CAL_SCAN_STEP;
  }

  if (SERIAL_DEBUG) {
    Serial.print("Calibration: bestLight=");
    Serial.print(bestLight);
    Serial.print(" at pos=");
    Serial.println(bestPos);
  }

  // Move to bestPos
  azStepper.moveTo(bestPos);
  while (azStepper.distanceToGo() != 0) azStepper.run();

  // Set this absolute stepper position as the stored *solar* home
  solarHomePosition = azStepper.currentPosition();
  saveSolarHome();
  calibrated = true;

  forceStepperLowOutputs();
  if (SERIAL_DEBUG) Serial.println("Calibration: done. Solar home set to best sun angle.");
}

// This function combines Homing and Scanning
void runFullCalibration() {
  if (SERIAL_DEBUG) Serial.println("--- Starting Full Calibration ---");
  homeAzimuth();      // 1. Find physical zero using software stall
  scanForSun();       // 2. Find sun zero within the new bounds
  if (SERIAL_DEBUG) Serial.println("--- Full Calibration Complete ---");
}

void parkNow() {
  if (SERIAL_DEBUG) Serial.println("Parking: moving to park position...");

  // Move azimuth relative to the *solar* home position
  long targetAbs = solarHomePosition + PARK_AZ_REL_POS;
  long safeTarget = clampAzTarget(targetAbs); // Clamp to machine bounds
  azStepper.moveTo(safeTarget);

  // move servo to park angle
  currentServo = PARK_SERVO_ANGLE;
  tiltServo.write(currentServo);
  saveServoAngle();

  // run to target (blocking)
  while (azStepper.distanceToGo() != 0) azStepper.run();

  forceStepperLowOutputs();
  isParked = true;
  if (SERIAL_DEBUG) Serial.println("Parking: complete.");
}

void unparkNow() {
  if (SERIAL_DEBUG) Serial.println("Unpark: resuming tracking.");
  isParked = false;
}

void printStatusNow(int tl, int tr, int bl, int br, int azDiff, int elDiff) {
  long queued = azStepper.distanceToGo();
  long curPos = azStepper.currentPosition();
  Serial.print("LDRs: "); Serial.print(tl); Serial.print(' ');
  Serial.print(tr); Serial.print(' ');
  Serial.print(bl); Serial.print(' ');
  Serial.print(br); Serial.print(" | Diff: Az=");
  Serial.print(azDiff); Serial.print(" El=");
  Serial.print(elDiff); Serial.print(" | Pos: Servo=");
  Serial.print(currentServo); Serial.print(" Stepper=");
  Serial.print((long)curPos); Serial.print(" (Max Pos=");
  Serial.print((long)AZ_MAX_POS); Serial.print(") (distToGo=");
  Serial.print((long)queued); Serial.println(")");
}

void printHelp() {
  Serial.println("Commands:");
  Serial.println("  c     -> RUN FULL CALIBRATION (Software Home + Sun Scan)");
  Serial.println("  h     -> Home (find mechanical MIN stop, set 0)");
  Serial.println("  B<int>-> Set AZIMUTH MAX BOUNDARY (saved), e.g., B4000"); 
  Serial.println("  M<int>-> Move AZIMUTH to absolute step position, e.g., M2000 (0 is MIN)");
  Serial.println("  Z     -> set CURRENT position as SOLAR HOME (saved)");
  Serial.println("  p     -> park");
  Serial.println("  u     -> unpark/resume");
  Serial.println("  s     -> status (print)");
  Serial.println("  S<int>-> set servo angle (save), e.g. S90");
  Serial.println("  ?     -> help");
}

void loop() {
  // ----- Serial command handling (non-blocking) -----
  static String cmdBuf = "";
  while (Serial.available()) {
    char rc = Serial.read();
    if (rc == '\n' || rc == '\r') {
      if (cmdBuf.length() > 0) {
        cmdBuf.trim();
        // single-char commands first
        if (cmdBuf.equalsIgnoreCase("c")) {
          // 'c' runs the full sequence: Home -> Scan
          runFullCalibration();
        } else if (cmdBuf.equalsIgnoreCase("h")) {
          // 'h' for just homing
          homeAzimuth();
        } else if (cmdBuf.equalsIgnoreCase("Z")) {
          // 'Z' sets the *solar* home
          solarHomePosition = azStepper.currentPosition();
          saveSolarHome();
          if (SERIAL_DEBUG) {
            Serial.print("Manual SOLAR HOME set to absolute stepper pos: ");
            Serial.println(solarHomePosition);
          }
        } else if (cmdBuf.equalsIgnoreCase("p")) {
          parkNow();
        } else if (cmdBuf.equalsIgnoreCase("u")) {
          unparkNow();
        } else if (cmdBuf.equalsIgnoreCase("s")) {
          // Status command
          int tl = avgRead(LDR_TL, 0);
          int tr = avgRead(LDR_TR, 1);
          int bl = avgRead(LDR_BL, 2);
          int br = avgRead(LDR_BR, 3);
          int azLeft = tl + bl;
          int azRight = tr + br;
          int azDiff = azLeft - azRight;
          int elTop = tl + tr;
          int elBottom = bl + br;
          int elDiff = elTop - elBottom;
          printStatusNow(tl, tr, bl, br, azDiff, elDiff);
        } else if (cmdBuf.startsWith("S")) {
          // Servo command
          String num = cmdBuf.substring(1);
          int v = num.toInt();
          if (v < SERVO_MIN) v = SERVO_MIN;
          if (v > SERVO_MAX) v = SERVO_MAX;
          currentServo = v;
          tiltServo.write(currentServo);
          saveServoAngle();
          if (SERIAL_DEBUG) {
            Serial.print("Servo manually set to ");
            Serial.println(currentServo);
          }
        } else if (cmdBuf.startsWith("B")) {
          // Boundary command
          String num = cmdBuf.substring(1);
          long v = num.toInt();
          // Safety check: ensure boundary is greater than MIN (0)
          if (v > AZ_MIN_POS) {
              AZ_MAX_POS = v;
              saveAzMaxPos();
              if (SERIAL_DEBUG) {
                  Serial.print("AZ_MAX_POS manually set to ");
                  Serial.println(AZ_MAX_POS);
              }
          } else {
              if (SERIAL_DEBUG) Serial.println("Boundary must be greater than 0.");
          }
        } else if (cmdBuf.startsWith("M")) {
          // NEW: Move command
          String num = cmdBuf.substring(1);
          long targetPos = num.toInt();
          long safeTarget = clampAzTarget(targetPos);
          azStepper.moveTo(safeTarget);
          if (SERIAL_DEBUG) {
            Serial.print("Azimuth moving to absolute step position: ");
            Serial.print(safeTarget);
            Serial.print(". (Requested: ");
            Serial.print(targetPos);
            Serial.println(")");
          }
        } else if (cmdBuf.equals("?")) {
          printHelp();
        } else {
          Serial.print("Unknown cmd: ");
          Serial.println(cmdBuf);
          printHelp();
        }
      }
      cmdBuf = "";
    } else {
      cmdBuf += rc;
      if (cmdBuf.length() > 32) cmdBuf = cmdBuf.substring(cmdBuf.length() - 32);
    }
  }

  // ----- Sensor Reading and Difference Calculation -----
  int tl = avgRead(LDR_TL, 0);
  int tr = avgRead(LDR_TR, 1);
  int bl = avgRead(LDR_BL, 2);
  int br = avgRead(LDR_BR, 3);
  ldrIdx = (ldrIdx + 1) % NUM_AVG;

  int azLeft  = tl + bl;
  int azRight = tr + br;
  int azDiff  = azLeft - azRight;

  int elTop    = tl + tr;
  int elBottom = bl + br;
  int elDiff   = elTop - elBottom;

  unsigned long now = millis();

  // ----- Parking (automatic) -----
  long totalLight = (long)tl + tr + bl + br;
  if (totalLight < PARK_LIGHT_THRESHOLD) {
    if (lightBelowSince == 0) lightBelowSince = now;
    else if (!isParked && (now - lightBelowSince >= PARK_LIGHT_TIME_MS)) {
      parkNow();
    }
  } else {
    lightBelowSince = 0;
  }

  // If parked, skip active tracking logic
  if (isParked) {
    azStepper.run(); // Still need to run() to finish any queued moves
    if (SERIAL_DEBUG && now - lastDebugMillis >= DEBUG_INTERVAL_MS) {
      lastDebugMillis = now;
      printStatusNow(tl, tr, bl, br, azDiff, elDiff);
    }
    delay(50);
    return;
  }

  // ----- elevation (servo) control -----
  if (abs(elDiff) > EL_DEADBAND) {
    int delta = -elDiff / SERVO_STEP_PER_DIFF;
    // Limit step size to avoid fast jerky movements
    if (delta > 3) delta = 3;
    if (delta < -3) delta = -3; 
    
    if (abs(delta) >= 1) {
      currentServo = constrain(currentServo + delta, SERVO_MIN, SERVO_MAX);
      tiltServo.write(currentServo);
    }
  }

  // ----- azimuth (side-to-side) control -----
  if (abs(azDiff) > AZ_DEADBAND) {
    int dir = (azDiff > 0) ? 1 : -1;
    // Calculate steps needed based on the difference
    int requestedSteps = abs(azDiff) / AZ_DIFF_TO_STEP_DIV + 1;
    if (requestedSteps > MAX_STEPS_PER_CYCLE) requestedSteps = MAX_STEPS_PER_CYCLE;

    long cur = azStepper.currentPosition();
    long desired = cur + (long)dir * (long)requestedSteps;
    
    // Clamp the target to the absolute software bounds (0 to AZ_MAX_POS)
    long safeDesired = clampAzTarget(desired);

    azStepper.moveTo(safeDesired);
  } else {
    // No movement needed, turn off holding torque to save power
    if (azStepper.distanceToGo() == 0) forceStepperLowOutputs();
  }

  // Drive the stepper towards its queued target
  azStepper.run();

  // ----- Serial debug (throttled) -----
  if (SERIAL_DEBUG && now - lastDebugMillis >= DEBUG_INTERVAL_MS) {
    lastDebugMillis = now;
    printStatusNow(tl, tr, bl, br, azDiff, elDiff);
  }

  // small non-blocking delay
  delay(10);
}
