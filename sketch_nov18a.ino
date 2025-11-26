// Solar tracker with AccelStepper + Servo, calibration, parking, serial debug,
// plus azimuth bounds to avoid wire wrapping/unplugging.
//
// - Azimuth: 28BYJ-48 via ULN2003 driven with AccelStepper (HALF4WIRE)
// - Elevation: hobby servo
// LDRs on A0..A3 as TL, TR, BL, BR.
// ULN2003 IN1..IN4 on pins STEPPER_PIN[0..3].
// Servo signal on SERVO_PIN.

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
// Note: accStepper constructor order (pin1, pin3, pin2, pin4) can differ by wiring.
// We're using the 4-pin HALF4WIRE constructor with array order as given.
AccelStepper azStepper(AccelStepper::HALF4WIRE,
                       STEPPER_PIN[0], STEPPER_PIN[1],
                       STEPPER_PIN[2], STEPPER_PIN[3]);

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
const float STEPPER_MAX_SPEED = 3000.0;   // [steps per second] -- raised to make it faster
const float STEPPER_ACCEL = 1000.0;        // [steps per second squared] -- faster accel

const long STEPS_PER_REV = 2048L; // assumed steps per rotation (28BYJ-48)

// --- Azimuth limiting (prevents cable wrap)
// Enable limits and set min/max absolute stepper positions (in stepper steps).
// Choose values appropriate to physical limits. Limits are applied relative to stored zero.
const bool AZ_LIMITS_ENABLED = true;
// Example: allow roughly +/- one full rev (tweak to your needs)
const long AZ_MIN_POS = -1000;   // minimum allowed RELATIVE position (steps) from zero
const long AZ_MAX_POS =  1000;   // maximum allowed RELATIVE position (steps) from zero

// --- Calibration settings ---
const bool AUTO_CALIBRATE_ON_BOOT = false; // set true if want auto-calibration on boot
const int CAL_SCAN_STEP = 16;  // coarse step increment during scan (half-steps). larger -> faster scan, less resolution
const int CAL_SCAN_DELAY_MS = 5; // delay between step moves during scan

// --- Parking / night detection ---
const int PARK_SERVO_ANGLE = SERVO_MIN;    // servo angle when parked
const long PARK_AZ_REL_POS = 0;            // parking azimuth relative to calibrated home
const int PARK_LIGHT_THRESHOLD = 150;      // if total light (sum of 4 sensors) below this => consider dark
const unsigned long PARK_LIGHT_TIME_MS = 30UL * 1000UL; // time below threshold to trigger park (30s default)

// --- Debug / serial ---
bool SERIAL_DEBUG = true;
unsigned long lastDebugMillis = 0;
const unsigned long DEBUG_INTERVAL_MS = 300; // print debug every 300 ms

// --- EEPROM addresses ---
const int EEPROM_AZ_ZERO_ADDR = 0;   // 4 bytes (long)
const int EEPROM_SERVO_ADDR   = 8;   // 2 bytes (int) at offset

// --- Globals ---
Servo tiltServo;
int ldrBuffers[4][NUM_AVG];
int ldrIdx = 0;
int currentServo = SERVO_CENTER;

bool isParked = false;
unsigned long lightBelowSince = 0;
bool calibrated = false;

// Persisted zero: this stores the stepper position (absolute reading) that represents "home/zero".
// Limits are applied around this value (i.e. allowed absolute positions = azimuthZero + AZ_MIN_POS .. azimuthZero + AZ_MAX_POS)
long azimuthZero = 0;

// --- Forward declarations ---
int avgRead(int sensorPin, int bufferIndex);
void forceStepperLowOutputs();
long clampAzTarget(long target);
void calibrateAzimuth();
void parkNow();
void unparkNow();
void printStatusNow(int tl, int tr, int bl, int br, int azDiff, int elDiff);
void saveAzZero();
void saveServoAngle();
void loadSavedSettings();
void printHelp();

void setup() {
  Serial.begin(115200);
  delay(50);

  // configure stepper
  azStepper.setMaxSpeed(STEPPER_MAX_SPEED);
  azStepper.setAcceleration(STEPPER_ACCEL);

  // ensure stepper pins are outputs so we can force them low if needed
  for (int i = 0; i < 4; ++i) {
    pinMode(STEPPER_PIN[i], OUTPUT);
    digitalWrite(STEPPER_PIN[i], LOW);
  }

  // initialize LDR buffers with initial readings
  for (int b = 0; b < NUM_AVG; ++b) {
    ldrBuffers[0][b] = analogRead(LDR_TL);
    ldrBuffers[1][b] = analogRead(LDR_TR);
    ldrBuffers[2][b] = analogRead(LDR_BL);
    ldrBuffers[3][b] = analogRead(LDR_BR);
    delay(5);
  }

  // servo
  tiltServo.attach(SERVO_PIN);

  // load persisted settings (azimuth zero, servo angle)
  loadSavedSettings();

  // set initial servo position to loaded value (or center by default)
  tiltServo.write(currentServo);

  // set a rough current stepper position (we assume physical stepper position corresponds to currentPosition())
  // azimuthZero contains the stored 'home' absolute position; don't overwrite currentPosition() here.
  azStepper.setCurrentPosition(azStepper.currentPosition()); // no-op but explicit

  if (SERIAL_DEBUG) {
    Serial.println();
    Serial.println("Solar tracker (AccelStepper + Servo) started");
    Serial.println("Commands (send single char or line end):");
    Serial.println("  c    -> calibrate (coarse scan) and set home");
    Serial.println("  Z    -> set current position as ZERO (saved to EEPROM)");
    Serial.println("  p    -> park (move to park az + park servo)");
    Serial.println("  u    -> unpark / resume tracking");
    Serial.println("  s    -> status (print single-line status)");
    Serial.println("  S<int> -> set servo angle and save, e.g. S90");
    Serial.println("  ?    -> help");
    Serial.print("Az limits enabled: "); Serial.println(AZ_LIMITS_ENABLED ? "YES" : "NO");
    if (AZ_LIMITS_ENABLED) {
      Serial.print("REL AZ limits: "); Serial.print(AZ_MIN_POS); Serial.print(" .. "); Serial.println(AZ_MAX_POS);
      Serial.print("Loaded azimuthZero (absolute steps) = "); Serial.println(azimuthZero);
      Serial.print("Allowed absolute range: ");
      Serial.print(azimuthZero + AZ_MIN_POS);
      Serial.print(" .. ");
      Serial.println(azimuthZero + AZ_MAX_POS);
    }
  }

  // optional: run auto calibration at boot
  if (AUTO_CALIBRATE_ON_BOOT) {
    delay(200); // allow sensor settle
    calibrateAzimuth();
  }
}

void loadSavedSettings() {
  // Read stored azimuth zero (long)
  EEPROM.get(EEPROM_AZ_ZERO_ADDR, azimuthZero);
  // Validate loaded value (EEPROM might be uninitialized)
  if (azimuthZero < -10000000L || azimuthZero > 10000000L) {
    azimuthZero = 0;
    EEPROM.put(EEPROM_AZ_ZERO_ADDR, azimuthZero);
  }

  // Read servo angle (int)
  int s = 0;
  EEPROM.get(EEPROM_SERVO_ADDR, s);
  if (s < SERVO_MIN || s > SERVO_MAX) s = SERVO_CENTER;
  currentServo = s;
}

void saveAzZero() {
  EEPROM.put(EEPROM_AZ_ZERO_ADDR, azimuthZero);
  if (SERIAL_DEBUG) {
    Serial.print("Saved azimuthZero (absolute steps) = ");
    Serial.println(azimuthZero);
    Serial.print("Allowed absolute range now: ");
    Serial.print(azimuthZero + AZ_MIN_POS);
    Serial.print(" .. ");
    Serial.println(azimuthZero + AZ_MAX_POS);
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
  // put new reading into circular buffer at current index
  ldrBuffers[bufferIndex][ldrIdx] = analogRead(sensorPin);
  long s = 0;
  for (int i = 0; i < NUM_AVG; ++i) s += ldrBuffers[bufferIndex][i];
  return (int)(s / NUM_AVG);
}

void forceStepperLowOutputs() {
  // Reduce holding current when stepper is idle:
  for (int i = 0; i < 4; ++i) digitalWrite(STEPPER_PIN[i], LOW);
}

// clamp a desired absolute az target to AZ limits if enabled.
// AZ limits are relative ranges around azimuthZero (which stores an absolute stepper position representing home)
long clampAzTarget(long target) {
  if (!AZ_LIMITS_ENABLED) return target;
  long minAbs = azimuthZero + AZ_MIN_POS;
  long maxAbs = azimuthZero + AZ_MAX_POS;
  if (target < minAbs) return minAbs;
  if (target > maxAbs) return maxAbs;
  return target;
}

void calibrateAzimuth() {
  // Full coarse sweep calibration:
  // - Sweep a portion (or full) revolution in increments to find the position where total light is maximum,
  // - Move there and set that as home (azimuthZero = that absolute stepper position).
  if (SERIAL_DEBUG) Serial.println("Calibration: starting azimuth scan...");

  delay(200); // settle

  long bestPos = azStepper.currentPosition();
  long bestLight = -1;

  long stepsToScan = STEPS_PER_REV; // try a full revolution (can be reduced)
  long stepsMoved = 0;

  // move in coarse negative direction to sample (direction is arbitrary)
  while (stepsMoved < stepsToScan) {
    // read averaged sensors
    int tl = avgRead(LDR_TL, 0);
    int tr = avgRead(LDR_TR, 1);
    int bl = avgRead(LDR_BL, 2);
    int br = avgRead(LDR_BR, 3);
    // advance circular buffer index
    ldrIdx = (ldrIdx + 1) % NUM_AVG;

    long totalLight = (long)tl + tr + bl + br;
    if (totalLight > bestLight) {
      bestLight = totalLight;
      bestPos = azStepper.currentPosition();
    }

    // step forward by coarse increment
    long coarse = -CAL_SCAN_STEP; // coarse step negative; adjust sign if your wiring prefers other direction

    long intendedPos = azStepper.currentPosition() + coarse;
    // if next step would exceed absolute clamp, stop scanning early
    if (AZ_LIMITS_ENABLED) {
      long minAbs = azimuthZero + AZ_MIN_POS;
      long maxAbs = azimuthZero + AZ_MAX_POS;
      if (intendedPos < minAbs || intendedPos > maxAbs) {
        break; // stop scan to avoid hitting limits
      }
    }

    azStepper.move(coarse);
    while (azStepper.distanceToGo() != 0) {
      azStepper.run();
      delay(CAL_SCAN_DELAY_MS);
    }
    stepsMoved += abs(coarse);
  }

  if (SERIAL_DEBUG) {
    Serial.print("Calibration: bestLight=");
    Serial.print(bestLight);
    Serial.print(" at pos=");
    Serial.println(bestPos);
  }

  // Move to bestPos (clamp to absolute bounds)
  long safeBest = clampAzTarget(bestPos);
  azStepper.moveTo(safeBest);
  while (azStepper.distanceToGo() != 0) azStepper.run();

  // set this absolute stepper position as the stored zero/home
  //azimuthZero = azStepper.currentPosition();
  //saveAzZero();
  //calibrated = true;

  forceStepperLowOutputs();
  if (SERIAL_DEBUG) Serial.println("Calibration: done. Home set to best sun angle (stored as zero).");
}

void parkNow() {
  if (SERIAL_DEBUG) Serial.println("Parking: moving to park position...");

  // Move azimuth to PARK_AZ_REL_POS relative to calibrated home
  long targetAbs = azimuthZero + PARK_AZ_REL_POS;
  long safeTarget = clampAzTarget(targetAbs);
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
  // tracking resumes next loop
}

void printStatusNow(int tl, int tr, int bl, int br, int azDiff, int elDiff) {
  long queued = azStepper.distanceToGo();
  long curPos = azStepper.currentPosition();
  Serial.print(tl); Serial.print(' ');
  Serial.print(tr); Serial.print(' ');
  Serial.print(bl); Serial.print(' ');
  Serial.print(br); Serial.print(" | ");
  Serial.print(azDiff); Serial.print(' ');
  Serial.print(elDiff); Serial.print(" | ");
  Serial.print(currentServo); Serial.print(' ');
  Serial.print((long)queued); Serial.print(' ');
  Serial.print((long)curPos); Serial.print(' ');
  Serial.print(" distToGo=");
  Serial.println((long)azStepper.distanceToGo());
}

void printHelp() {
  Serial.println("Commands:");
  Serial.println("  c     -> calibrate (scan) and set home");
  Serial.println("  Z     -> set current position as ZERO (saved)");
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
          calibrateAzimuth();
        } else if (cmdBuf.equalsIgnoreCase("Z")) {
          // set current absolute stepper position as zero and persist
          azimuthZero = azStepper.currentPosition();
          saveAzZero();
          if (SERIAL_DEBUG) {
            Serial.print("Manual ZERO set to absolute stepper pos: ");
            Serial.println(azimuthZero);
          }
        } else if (cmdBuf.equalsIgnoreCase("p")) {
          parkNow();
        } else if (cmdBuf.equalsIgnoreCase("u")) {
          unparkNow();
        } else if (cmdBuf.equalsIgnoreCase("s")) {
          // immediate status
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
          // set servo angle and save: S90
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

  // ----- Moving average of light read values -----
  int tl = avgRead(LDR_TL, 0);
  int tr = avgRead(LDR_TR, 1);
  int bl = avgRead(LDR_BL, 2);
  int br = avgRead(LDR_BR, 3);
  ldrIdx = (ldrIdx + 1) % NUM_AVG;

  // light values on all four sides
  int azLeft  = tl + bl; // left side
  int azRight = tr + br; // right side
  int azDiff  = azLeft - azRight;     // positive => more light on left

  int elTop    = tl + tr; // top
  int elBottom = bl + br; // bottom
  int elDiff   = elTop - elBottom;    // positive => more light on top

  unsigned long now = millis();

  // ----- Parking (automatic) based on low ambient light for duration -----
  long totalLight = (long)tl + tr + bl + br;
  if (totalLight < PARK_LIGHT_THRESHOLD) {
    if (lightBelowSince == 0) lightBelowSince = now;
    else if (!isParked && (now - lightBelowSince >= PARK_LIGHT_TIME_MS)) {
      parkNow();
    }
  } else {
    lightBelowSince = 0; // reset timer
  }

  // If parked, skip active tracking (but still run stepper so queued moves finish and accept commands)
  if (isParked) {
    azStepper.run();
    if (SERIAL_DEBUG && now - lastDebugMillis >= DEBUG_INTERVAL_MS) {
      lastDebugMillis = now;
      printStatusNow(tl, tr, bl, br, azDiff, elDiff);
    }
    delay(50);
    return;
  }

  // ----- elevation (servo) control (small proportional increments) -----
  if (abs(elDiff) > EL_DEADBAND) {
    // negative elDiff -> bottom brighter -> tilt down (invert sign if needed physically)
    int delta = -elDiff / SERVO_STEP_PER_DIFF;
    // clamp per-loop delta
    if (delta > 3) delta = 3;
    if (delta < -3) delta = -3;
    if (abs(delta) >= 1) {
      currentServo = constrain(currentServo + delta, SERVO_MIN, SERVO_MAX);
      tiltServo.write(currentServo);
      // don't flood EEPROM; save servo angle only on explicit S<int> or when we park; if you want autosave, uncomment:
      // saveServoAngle();
    }
  }

  // ----- azimuth (side-to-side) control -----
  if (abs(azDiff) > AZ_DEADBAND) {
    int dir = (azDiff > 0) ? 1 : -1; // positive azDiff => more light left => move toward left. Sign may need inversion after testing.
    int requestedSteps = abs(azDiff) / AZ_DIFF_TO_STEP_DIV + 1; // basic proportional mapping
    if (requestedSteps > MAX_STEPS_PER_CYCLE) requestedSteps = MAX_STEPS_PER_CYCLE;

    long cur = azStepper.currentPosition();
    long desired = cur + (long)dir * (long)requestedSteps;
    long safeDesired = clampAzTarget(desired);

//    if (safeDesired != desired && SERIAL_DEBUG) {
  //    Serial.print("Az request clipped. desired=");
    // Serial.print(desired);
     //Serial.print(" clipped=");
      //Serial.println(safeDesired);
    //}

    azStepper.moveTo(safeDesired);
  } else {
    // no movement request; if at target, reduce holding current
    if (azStepper.distanceToGo() == 0) forceStepperLowOutputs();
  }

  // Drive the stepper towards its queued target
  azStepper.run();

  // ----- Serial debug (throttled) -----
  if (SERIAL_DEBUG && now - lastDebugMillis >= DEBUG_INTERVAL_MS) {
    lastDebugMillis = now;
    printStatusNow(tl, tr, bl, br, azDiff, elDiff);
  }

  // small non-blocking delay to give other tasks time
  delay(10);
}
