#include <Wire.h>
#include <Adafruit_MotorShield.h>

const int PIN_IR    = A0;
const int PIN_LIGHT = A1;

const int SAMPLES = 50;
const unsigned long SAMPLE_GAP_MS = 10;

Adafruit_MotorShield AFMS = Adafruit_MotorShield();

struct Stats { float mean, std; int lo, hi; };

Stats summarize(const int *v, int n) {
  Stats s = { 0, 0, 1023, 0 };
  for (int i = 0; i < n; i++) {
    s.mean += v[i];
    s.lo = min(s.lo, v[i]);
    s.hi = max(s.hi, v[i]);
  }
  s.mean /= n;
  for (int i = 0; i < n; i++) s.std += (v[i] - s.mean) * (v[i] - s.mean);
  s.std = sqrt(s.std / n);
  return s;
}

void printStats(const Stats &s) {
  Serial.print(s.mean, 1); Serial.print(',');
  Serial.print(s.lo);      Serial.print(',');
  Serial.print(s.hi);      Serial.print(',');
  Serial.print(s.std, 2);
}

void setup() {
  Serial.begin(9600);

  // The shield keeps its last motor command across an Arduino reset, so stop
  // the wheels explicitly in case the main sketch left them driving.
  if (AFMS.begin()) {
    for (int m = 1; m <= 4; m++) AFMS.getMotor(m)->run(RELEASE);
  }

  Serial.println("label,n,ir_mean,ir_min,ir_max,ir_std,light_mean,light_min,light_max,light_std");
}

void loop() {
  if (!Serial.available()) return;
  String label = Serial.readStringUntil('\n');
  label.trim();
  if (label.length() == 0) return;

  static int ir[SAMPLES], light[SAMPLES];
  for (int i = 0; i < SAMPLES; i++) {
    ir[i] = analogRead(PIN_IR);
    light[i] = analogRead(PIN_LIGHT);
    delay(SAMPLE_GAP_MS);
  }

  Serial.print(label);   Serial.print(',');
  Serial.print(SAMPLES); Serial.print(',');
  printStats(summarize(ir, SAMPLES));    Serial.print(',');
  printStats(summarize(light, SAMPLES));
  Serial.println();
}
