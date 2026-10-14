// Puffin Animatronic - IR distance sensor calibration

#include <Wire.h>
#include <Adafruit_MotorShield.h>

const int PIN_IR = A0;
const unsigned long STREAM_MS = 100;

Adafruit_MotorShield AFMS = Adafruit_MotorShield();

int readIR() {
  long sum = 0;
  for (int i = 0; i < 4; i++) sum += analogRead(PIN_IR);
  return sum / 4;
}

void setup() {
  Serial.begin(9600);

  // The shield keeps its last motor command across an Arduino reset, so stop
  // the wheels explicitly in case the main sketch left them driving.
  if (AFMS.begin()) {
    for (int m = 1; m <= 4; m++) AFMS.getMotor(m)->run(RELEASE);
  }
}

void loop() {
  Serial.println(readIR());
  delay(STREAM_MS);
}
