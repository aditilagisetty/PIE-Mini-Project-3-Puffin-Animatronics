/*
 * Puffin Animatronic - reset to initial position
 *
 * Upload this to put every actuator in its starting position and hold it there:
 *   - Both wheel motors stopped (released)
 *   - Wing servo down
 *   - Beak servo closed
 *   - Beak LED off
 *
 * Useful for lining up servo horns / linkages during assembly, or for parking
 * the puffin before switching it off. Nothing moves after setup().
 *
 * Pins and angles match puffin/puffin.ino - if you change them there, change
 * them here too.
 *
 * Requires the "Adafruit Motor Shield V2 Library" (Arduino Library Manager).
 */

#include <Servo.h>
#include <Wire.h>
#include <Adafruit_MotorShield.h>

// ---------------- Pins ----------------
const int MOTOR_LEFT  = 1;           // shield port M1
const int MOTOR_RIGHT = 3;           // shield port M3
const int PIN_WING    = 5;
const int PIN_BEAK    = 6;
const int PIN_LED     = 4;

// ---------------- Initial positions ----------------
const int WING_DOWN   = 20;
const int BEAK_CLOSED = 10;

Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_DCMotor *leftWheel  = AFMS.getMotor(MOTOR_LEFT);
Adafruit_DCMotor *rightWheel = AFMS.getMotor(MOTOR_RIGHT);
Servo wing, beak;

void setup() {
  Serial.begin(9600);

  pinMode(PIN_LED, OUTPUT);
  digitalWrite(PIN_LED, LOW);

  if (!AFMS.begin()) {               // default I2C address 0x60
    Serial.println("Motor shield not found - check that it's seated.");
  } else {
    leftWheel->setSpeed(0);
    rightWheel->setSpeed(0);
    leftWheel->run(RELEASE);
    rightWheel->run(RELEASE);
    Serial.println("Wheels stopped.");
  }

  wing.attach(PIN_WING);
  beak.attach(PIN_BEAK);
  wing.write(WING_DOWN);
  beak.write(BEAK_CLOSED);

  Serial.print("Wing -> ");  Serial.print(WING_DOWN);   Serial.println(" deg (down)");
  Serial.print("Beak -> ");  Serial.print(BEAK_CLOSED); Serial.println(" deg (closed)");
  Serial.println("Puffin is in its initial position.");
}

void loop() {
  // Servos stay attached so they keep holding their positions.
}
