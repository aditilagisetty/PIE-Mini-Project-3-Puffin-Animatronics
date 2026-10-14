/*
 * Puffin Animatronic - starter sensor + motor code
 *
 * Behaviors:
 *   1. Chassis drives forward. If the IR distance sensor sees something too
 *      close, the wheels stop and the wings flap ("shake hands"). Once the path
 *      has been clear for a moment, the wings go down and the puffin drives again.
 *   2. Each button press opens the beak for 3 seconds, then it closes.
 *      Pressing again while it's open restarts the 3 seconds.
 *   3. When the photoresistor reads dark, the beak LED turns on.
 *
 * Drive logic is a small state machine (DRIVING / GREETING). The beak and LED
 * run independently every loop, so they work in either state.
 * Everything is non-blocking (millis(), no delay()) so all sensors stay live.
 *
 * Wiring (Arduino Uno + Adafruit Motor Shield v2) - change the constants below to match your build:
 *   Left wheel DC motor                   -> shield M1 terminals
 *   Right wheel DC motor                  -> shield M3 terminals
 *   Wing servo        (positional) signal -> D5
 *   Beak servo        (positional) signal -> D6
 *   IR distance sensor (analog out)       -> A0
 *   Photoresistor divider midpoint        -> A1   (LDR to 5V, 10k to GND)
 *   Button                                -> D2 and GND (uses INPUT_PULLUP)
 *   LED (+ ~220 ohm resistor)             -> D4
 *
 * The shield talks to the Uno over I2C (A4/A5), so leave those pins free.
 * Power the DC motors through the shield's motor power terminal block (remove
 * the VIN jumper if that supply is separate from the Arduino's).
 *
 * Power servos from a separate 5-6V supply (NOT the Arduino 5V pin) and tie
 * that supply's GND to Arduino GND. A 100-470uF capacitor across the servo
 * supply helps with brownouts/jitter when several servos move at once.
 *
 * Requires the "Adafruit Motor Shield V2 Library" (Arduino Library Manager).
 */

#include <Servo.h>
#include <Wire.h>
#include <Adafruit_MotorShield.h>

// ---------------- Pins ----------------
const int MOTOR_LEFT      = 4;       // shield port M1
const int MOTOR_RIGHT     = 3;       // shield port M3
const int PIN_WING        = 5;
const int PIN_BEAK        = 6;
const int PIN_IR          = A0;
const int PIN_LIGHT       = A1;
const int PIN_BUTTON      = 2;
const int PIN_LED         = 4;

// ---------------- Tuning ----------------
// Set to true to print sensor readings to Serial Monitor (9600 baud) for calibration.
const bool DEBUG = true;

// IR sensor (Sharp-style analog: HIGHER reading = CLOSER object).
// Two thresholds (hysteresis) so it doesn't flicker at the boundary.
// Values from calibration/ir_calibration.py: 20 cm reads ~476, 30 cm ~399, 40 cm ~332.
const int IR_TOO_CLOSE = 470;        // stop when reading goes above this (~20 cm)
const int IR_CLEAR     = 369;        // considered clear when below this (~35 cm)
const unsigned long CLEAR_TIME_MS = 1000;  // path must be clear this long before driving again

// Photoresistor (with LDR to 5V, 10k to GND: LOWER reading = DARKER).
// Values from calibration/light_calibration.py: covered ~369, shaded ~690, room ~811.
const int LIGHT_DARK   = 745;        // LED on below this (shaded or darker)
const int LIGHT_BRIGHT = 765;        // LED off above this (room light)

// DC wheel motors: speed 0-255. Start slow and raise once it drives straight.
// Wheels are mounted mirrored, so one spins "backward" to go forward.
// If the robot spins in place or drives backward, swap FORWARD/BACKWARD here.
const int DRIVE_SPEED = 120;
const uint8_t LEFT_FORWARD  = FORWARD;
const uint8_t RIGHT_FORWARD = BACKWARD;

// Wing servo angles + flap speed.
// The wing moves WING_STEP_DEG every WING_STEP_MS, so one sweep from down to up
// takes (WING_UP - WING_DOWN) / WING_STEP_DEG * WING_STEP_MS ms (~900 ms here).
// Raise WING_STEP_MS (or lower WING_STEP_DEG) to flap slower.
const int WING_DOWN = 40;           // resting position (slightly outward)
const int WING_UP   = 80;          // furthest outward point of a flap
const int WING_STEP_DEG = 2;
const unsigned long WING_STEP_MS = 20;

// Beak servo angles
const int BEAK_CLOSED = 10;
const int BEAK_OPEN   = 15;
const unsigned long BEAK_OPEN_MS = 3000;   // how long the beak stays open per press

const unsigned long DEBOUNCE_MS = 30;

// ---------------- State ----------------
enum DriveState { DRIVING, GREETING };
DriveState driveState = DRIVING;

Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_DCMotor *leftWheel  = AFMS.getMotor(MOTOR_LEFT);
Adafruit_DCMotor *rightWheel = AFMS.getMotor(MOTOR_RIGHT);
Servo wing, beak;

bool wheelsMoving = false;           // so we only send I2C commands when it changes

unsigned long clearSince = 0;        // when the IR path last became clear
int wingPos = WING_DOWN;             // angle last sent to the wing servo
int wingTarget = WING_DOWN;          // angle the wing is currently sweeping toward
unsigned long lastWingStep = 0;

bool buttonStable = HIGH;            // debounced reading (HIGH = not pressed with pullup)
bool buttonLastRaw = HIGH;
unsigned long buttonChangedAt = 0;
bool buttonWasPressed = false;       // last loop's debounced state, to catch new presses

bool beakIsOpen = false;
unsigned long beakOpenedAt = 0;

bool ledOn = false;

// Motor Helpers
void driveForward() {
  if (wheelsMoving) return;
  wheelsMoving = true;
  leftWheel->setSpeed(DRIVE_SPEED);
  rightWheel->setSpeed(DRIVE_SPEED);
  leftWheel->run(LEFT_FORWARD);
  rightWheel->run(RIGHT_FORWARD);
}

void stopWheels() {
  if (!wheelsMoving) return;
  wheelsMoving = false;
  leftWheel->run(RELEASE);
  rightWheel->run(RELEASE);
}

// Once the wing reaches the end of a sweep, head back the other way.
void flapWings() {
  if (wingPos == wingTarget) {
    wingTarget = (wingTarget == WING_UP) ? WING_DOWN : WING_UP;
  }
}

// Glide the wing back down (updateWing() does the actual movement).
void wingsDown() {
  wingTarget = WING_DOWN;
}

// Step the wing servo a few degrees toward wingTarget. Called every loop.
void updateWing(unsigned long now) {
  if (wingPos == wingTarget || now - lastWingStep < WING_STEP_MS) return;
  lastWingStep = now;
  if (wingPos < wingTarget) wingPos = min(wingPos + WING_STEP_DEG, wingTarget);
  else                      wingPos = max(wingPos - WING_STEP_DEG, wingTarget);
  wing.write(wingPos);
}

// ---------------- Sensor helpers ----------------
int readIR() {
  // Average a few samples - IR sensors are noisy.
  long sum = 0;
  for (int i = 0; i < 4; i++) sum += analogRead(PIN_IR);
  return sum / 4;
}

bool buttonPressed(unsigned long now) {
  bool raw = digitalRead(PIN_BUTTON);
  if (raw != buttonLastRaw) {
    buttonLastRaw = raw;
    buttonChangedAt = now;
  }
  if (now - buttonChangedAt >= DEBOUNCE_MS) {
    buttonStable = raw;
  }
  return buttonStable == LOW;        // pressed pulls the pin to GND
}

// ---------------- Behaviors ----------------
void updateDrive(unsigned long now, int ir) {
  switch (driveState) {
    case DRIVING:
      driveForward();
      if (ir > IR_TOO_CLOSE) {
        stopWheels();
        driveState = GREETING;
        clearSince = 0;
        wingTarget = WING_UP;                // start flapping right away
      }
      break;

    case GREETING:
      stopWheels();
      flapWings();
      if (ir < IR_CLEAR) {
        if (clearSince == 0) clearSince = now;
        if (now - clearSince >= CLEAR_TIME_MS) {
          wingsDown();
          driveState = DRIVING;
        }
      } else {
        clearSince = 0;                      // something is still there
      }
      break;
  }
}

void updateBeak(unsigned long now) {
  bool pressed = buttonPressed(now);
  if (pressed && !buttonWasPressed) {        // new press (not held)
    beakOpenedAt = now;
    if (!beakIsOpen) {
      beakIsOpen = true;
      beak.write(BEAK_OPEN);
    }
  }
  buttonWasPressed = pressed;

  if (beakIsOpen && now - beakOpenedAt >= BEAK_OPEN_MS) {
    beakIsOpen = false;
    beak.write(BEAK_CLOSED);
  }
}

void updateLED(int light) {
  if (!ledOn && light < LIGHT_DARK) ledOn = true;
  else if (ledOn && light > LIGHT_BRIGHT) ledOn = false;
  digitalWrite(PIN_LED, ledOn ? HIGH : LOW);
}

// ---------------- Main ----------------
void setup() {
  pinMode(PIN_BUTTON, INPUT_PULLUP);
  pinMode(PIN_LED, OUTPUT);

  if (DEBUG) Serial.begin(9600);

  if (!AFMS.begin()) {               // default I2C address 0x60
    if (DEBUG) Serial.println("Motor shield not found - check that it's seated.");
    while (true) {}
  }

  wing.attach(PIN_WING);
  beak.attach(PIN_BEAK);

  wheelsMoving = true;               // force stopWheels() to send the command
  stopWheels();
  wing.write(WING_DOWN);             // wingPos/wingTarget already start at WING_DOWN
  beak.write(BEAK_CLOSED);

  delay(1000);                       // give servos time to settle before moving
}

void loop() {
  unsigned long now = millis();
  int ir = readIR();
  int light = analogRead(PIN_LIGHT);

  updateDrive(now, ir);
  updateWing(now);
  updateBeak(now);
  updateLED(light);

  if (DEBUG) {
    static unsigned long lastPrint = 0;
    if (now - lastPrint >= 200) {
      lastPrint = now;
      Serial.print("IR: ");       Serial.print(ir);
      Serial.print("  Light: ");  Serial.print(light);
      Serial.print(" ("); Serial.print(light * 5.0 / 1023, 2); Serial.print("V)");
      Serial.print("  Button: "); Serial.print(buttonStable == LOW ? "PRESSED" : "-");
      Serial.print("  State: ");  Serial.println(driveState == DRIVING ? "DRIVING" : "GREETING");
    }
  }
}
