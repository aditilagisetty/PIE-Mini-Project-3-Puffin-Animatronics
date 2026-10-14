/*
 * Puffin Animatronic - photoresistor calibration
 *
 * Streams one photoresistor reading (A1) per line, every STREAM_MS. Each
 * reading is a single analogRead(), the same as the main sketch uses.
 *
 */

#include <Wire.h>
#include <Adafruit_MotorShield.h>

const int PIN_LIGHT = A1;
const unsigned long STREAM_MS = 100;

Adafruit_MotorShield AFMS = Adafruit_MotorShield();

void setup() {
  Serial.begin(9600);

  // The shield keeps its last motor command across an Arduino reset, so stop
  // the wheels explicitly in case the main sketch left them driving.
  if (AFMS.begin()) {
    for (int m = 1; m <= 4; m++) AFMS.getMotor(m)->run(RELEASE);
  }
}

void loop() {
  Serial.println(analogRead(PIN_LIGHT));
  delay(STREAM_MS);
}
