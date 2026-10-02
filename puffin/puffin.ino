/*
 * Puffin Animatronic - starter sensor + motor code
 *
 * Behaviors:
 *   1. Chassis drives forward. If the IR distance sensor sees something too
 *      close, the wheels stop and the wings flap ("shake hands"). Once the path
 *      has been clear for a moment, the wings go down and the puffin drives again.
 *   2. While the button is held, the beak servo opens. Released -> beak closes.
 *   3. When the photoresistor reads dark, the beak LED turns on.
 *
 * Drive logic is a small state machine (DRIVING / GREETING). The beak and LED
 * run independently every loop, so they work in either state.
 * Everything is non-blocking (millis(), no delay()) so all sensors stay live.
 *
 * Wiring (Arduino Uno) - change the pin constants below to match your build:
 *   Left wheel servo  (continuous) signal -> D9
 *   Right wheel servo (continuous) signal -> D10
 *   Wing servo        (positional) signal -> D5
 *   Beak servo        (positional) signal -> D6
 *   IR distance sensor (analog out)       -> A0
 *   Photoresistor divider midpoint        -> A1   (LDR to 5V, 10k to GND)
 *   Button                                -> D2 and GND (uses INPUT_PULLUP)
 *   LED (+ ~220 ohm resistor)             -> D4
 *
 * Power servos from a separate 5-6V supply (NOT the Arduino 5V pin) and tie
 * that supply's GND to Arduino GND. A 100-470uF capacitor across the servo
 * supply helps with brownouts/jitter when several servos move at once.
 */

#include <Servo.h>

// ---------------- Pins ----------------
const int PIN_LEFT_WHEEL  = 9;
const int PIN_RIGHT_WHEEL = 10;
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
const int IR_TOO_CLOSE = 400;        // stop when reading goes above this
const int IR_CLEAR     = 320;        // considered clear when below this
const unsigned long CLEAR_TIME_MS = 1000;  // path must be clear this long before driving again

// Photoresistor (with LDR to 5V, 10k to GND: LOWER reading = DARKER).
const int LIGHT_DARK   = 300;        // LED on below this
const int LIGHT_BRIGHT = 380;        // LED off above this

// Continuous-rotation wheel servos: 90 = stop, 0/180 = full speed each way.
// If a wheel creeps at "stop", adjust its STOP value (e.g. 88 or 92) or the servo's trim pot.
const int LEFT_STOP  = 90;
const int RIGHT_STOP = 90;
const int DRIVE_SPEED = 30;          // offset from stop (0-90); start slow

// Wing servo angles + flap speed
const int WING_DOWN = 20;
const int WING_UP   = 110;
const unsigned long FLAP_INTERVAL_MS = 300;

// Beak servo angles
const int BEAK_CLOSED = 10;
const int BEAK_OPEN   = 70;

const unsigned long DEBOUNCE_MS = 30;

// ---------------- State ----------------
enum DriveState { DRIVING, GREETING };
DriveState driveState = DRIVING;

Servo leftWheel, rightWheel, wing, beak;

unsigned long clearSince = 0;        // when the IR path last became clear
unsigned long lastFlap = 0;
bool wingIsUp = false;

bool buttonStable = HIGH;            // debounced reading (HIGH = not pressed with pullup)
bool buttonLastRaw = HIGH;
unsigned long buttonChangedAt = 0;

bool ledOn = false;

// ---------------- Motor helpers ----------------
void driveForward() {
  // Wheels are mounted mirrored, so one spins "backward" to go forward.
  // If the robot spins in place or drives backward, swap the + / - here.
  leftWheel.write(LEFT_STOP + DRIVE_SPEED);
  rightWheel.write(RIGHT_STOP - DRIVE_SPEED);
}

void stopWheels() {
  leftWheel.write(LEFT_STOP);
  rightWheel.write(RIGHT_STOP);
}

void flapWings(unsigned long now) {
  if (now - lastFlap >= FLAP_INTERVAL_MS) {
    lastFlap = now;
    wingIsUp = !wingIsUp;
    wing.write(wingIsUp ? WING_UP : WING_DOWN);
  }
}

void wingsDown() {
  wingIsUp = false;
  wing.write(WING_DOWN);
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
        lastFlap = now - FLAP_INTERVAL_MS;   // flap immediately
      }
      break;

    case GREETING:
      stopWheels();
      flapWings(now);
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
  beak.write(buttonPressed(now) ? BEAK_OPEN : BEAK_CLOSED);
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

  leftWheel.attach(PIN_LEFT_WHEEL);
  rightWheel.attach(PIN_RIGHT_WHEEL);
  wing.attach(PIN_WING);
  beak.attach(PIN_BEAK);

  stopWheels();
  wingsDown();
  beak.write(BEAK_CLOSED);

  if (DEBUG) Serial.begin(9600);
  delay(1000);                       // give servos time to settle before moving
}

void loop() {
  unsigned long now = millis();
  int ir = readIR();
  int light = analogRead(PIN_LIGHT);

  updateDrive(now, ir);
  updateBeak(now);
  updateLED(light);

  if (DEBUG) {
    static unsigned long lastPrint = 0;
    if (now - lastPrint >= 200) {
      lastPrint = now;
      Serial.print("IR: ");       Serial.print(ir);
      Serial.print("  Light: ");  Serial.print(light);
      Serial.print("  Button: "); Serial.print(buttonStable == LOW ? "PRESSED" : "-");
      Serial.print("  State: ");  Serial.println(driveState == DRIVING ? "DRIVING" : "GREETING");
    }
  }
}
