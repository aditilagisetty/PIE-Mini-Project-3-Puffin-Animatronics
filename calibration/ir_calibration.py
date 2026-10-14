# Puffin - IR distance sensor calibration
#
# pip install pyserial numpy matplotlib
#
# usage:
#   python ir_calibration.py        collect readings, then plot
#   python ir_calibration.py plot   re-plot from ir_calibration_data.csv

import csv
import re
import sys
import time
from pathlib import Path

import numpy as np
import matplotlib.pyplot as plt

arduinoComPort = "COM3"
baudRate = 9600

distancesCm = [10, 15, 20, 25, 30, 40, 50, 60, 80]
readingsPerDistance = 20
excludeDistancesCm = [80]  # bad data, left in the csv but skipped when plotting

minRange = 15.0        # cm, fit only uses points at or beyond this
stopDistanceCm = 20.0  # where the puffin should stop and greet someone
clearMarginCm = 10.0   # how much farther away counts as "clear" again

adcMax = 1023
vRef = 5.0

here = Path(__file__).resolve().parent
dataFilename = here / "ir_calibration_data.csv"
plotFilename = here / "ir_calibration_plot.png"
mainSketch = here.parent / "puffin" / "puffin.ino"


def collect():
  import serial

  serialPort = serial.Serial(arduinoComPort, baudRate, timeout=1)
  time.sleep(2)  # opening the port resets the Arduino

  steps = [str(d) for d in distancesCm] + ["none"]
  rows = []  # (distance_cm, raw_sensor_value)

  try:
    for step in steps:
      if step == "none":
        input("\nRemove everything in front of the sensor, then press Enter...")
      else:
        input("\nPlace the object at " + step + " cm, then press Enter...")

      serialPort.reset_input_buffer()
      readings = []
      while len(readings) < readingsPerDistance:
        line = serialPort.readline().decode(errors="replace").strip()
        if line.isdigit():
          readings.append(int(line))

      print("  " + step + " -> mean = " + str(round(np.mean(readings), 1))
            + ", std = " + str(round(np.std(readings), 1))
            + " (" + str(len(readings)) + " readings)")
      rows += [(step, r) for r in readings]

  except KeyboardInterrupt:
    print("\nStopped early by user.")

  finally:
    serialPort.close()

  with open(dataFilename, "w", newline="") as csvFile:
    writer = csv.writer(csvFile)
    writer.writerow(["distance_cm", "raw_sensor_value"])
    writer.writerows(rows)
  print("\nSaved " + str(len(rows)) + " readings to " + dataFilename.name)


# ---- analysis ----
# returns arrays of distance (cm) and raw reading, plus the no-object readings
def loadReadings():
  distances, raws, noneRaws = [], [], []
  with open(dataFilename, newline="") as csvFile:
    for row in csv.DictReader(csvFile):
      if row["distance_cm"] == "none":
        noneRaws.append(float(row["raw_sensor_value"]))
      elif float(row["distance_cm"]) not in excludeDistancesCm:
        distances.append(float(row["distance_cm"]))
        raws.append(float(row["raw_sensor_value"]))
  return np.array(distances), np.array(raws), np.array(noneRaws)


# reads a "const int NAME = value;" line from puffin.ino
def readThreshold(name):
  match = re.search(r"const\s+int\s+" + name + r"\s*=\s*(\d+)", mainSketch.read_text())
  return int(match.group(1)) if match else None


# least-squares fit of ln(distance) = ln(A) + B*ln(reading)
# returns (A, B, rSquared); rSquared is computed in log-log space
def fitPowerLaw(distances, raws):
  lnR = np.log(raws)
  lnD = np.log(distances)
  b, lnA = np.polyfit(lnR, lnD, 1)
  residuals = lnD - (lnA + b * lnR)
  rSquared = 1 - np.sum(residuals ** 2) / np.sum((lnD - lnD.mean()) ** 2)
  return np.exp(lnA), b, rSquared


def plot():
  dist, raw, noneRaw = loadReadings()
  inFit = (dist >= minRange) & (raw > 0)

  calibA, calibB, rSquared = fitPowerLaw(dist[inFit], raw[inFit])
  distanceFromRaw = lambda r: calibA * np.power(r, calibB)
  rawFromDistance = lambda d: np.power(d / calibA, 1.0 / calibB)

  groupDist = np.unique(dist)
  groupMean = np.array([raw[dist == d].mean() for d in groupDist])
  groupStd  = np.array([raw[dist == d].std() for d in groupDist])
  groupInFit = groupDist >= minRange

  print("Fitted calibration:")
  print("  distance = " + str(round(calibA, 1)) + " * reading^" + str(round(calibB, 3))
        + "   (R^2 = " + str(round(rSquared, 4)) + " in log-log space)")

  fig, ax = plt.subplots(figsize=(9, 6))

  # individual readings, faded, so the spread at each distance is visible
  ax.scatter(dist[inFit], raw[inFit], s=10, alpha=0.3, color="tab:blue",
             label="Individual readings")
  if (~inFit).any():
    ax.scatter(dist[~inFit], raw[~inFit], s=10, alpha=0.3, color="tab:gray",
               label="Below " + str(int(minRange)) + " cm (not fit)")

  ax.errorbar(groupDist[groupInFit], groupMean[groupInFit], yerr=groupStd[groupInFit],
              fmt="o", color="tab:blue", capsize=4, label="Mean ± 1 std")

  curveDist = np.linspace(minRange, dist.max(), 200)
  ax.plot(curveDist, rawFromDistance(curveDist), color="tab:orange", linewidth=2,
          label="Fit: d = " + str(round(calibA, 1)) + " · reading^(" + str(round(calibB, 3))
                + "),  R² = " + str(round(rSquared, 3)))

  if len(noneRaw):
    ax.axhline(noneRaw.mean(), color="tab:gray", linestyle=":",
               label="No object: " + str(round(noneRaw.mean(), 1)))

  # thresholds from puffin.ino, and the distance each one corresponds to
  for name, color, action in [("IR_TOO_CLOSE", "tab:red", "stop above"),
                              ("IR_CLEAR", "tab:green", "drive again below")]:
    value = readThreshold(name)
    if value:
      ax.axhline(value, color=color, linestyle="--",
                 label=name + " = " + str(value) + " (" + action + ", ≈ "
                       + str(round(float(distanceFromRaw(value)), 1)) + " cm)")

  ax.set_xlabel("Actual distance (cm)")
  ax.set_ylabel("Sensor reading (0-1023)")
  volts = ax.secondary_yaxis("right", functions=(lambda r: r * vRef / adcMax,
                                                 lambda v: v * adcMax / vRef))
  volts.set_ylabel("Sensor output (V)")
  ax.set_title("IR Distance Sensor Calibration")
  ax.set_xlim(0, curveDist.max() * 1.05)
  ax.set_ylim(0, max(raw.max(), readThreshold("IR_TOO_CLOSE") or 0) * 1.1)
  ax.grid(alpha=0.3)
  ax.legend()
  fig.tight_layout()
  fig.savefig(plotFilename, dpi=200)

  # summary for the write-up
  print("\nPer-distance readings:")
  for d, m, s in zip(groupDist, groupMean, groupStd):
    print("  " + str(int(d)).rjust(3) + " cm: mean = " + str(round(m, 1)).rjust(6)
          + ", std = " + str(round(s, 1)))
  if len(noneRaw):
    print("  none  : mean = " + str(round(noneRaw.mean(), 1)) + ", std = " + str(round(noneRaw.std(), 1)))

  suggestedStop = int(round(float(rawFromDistance(stopDistanceCm))))
  suggestedClear = int(round(float(rawFromDistance(stopDistanceCm + clearMarginCm))))
  noise = groupStd[groupInFit].max()
  print("\nSuggested thresholds for stopping at " + str(int(stopDistanceCm)) + " cm:")
  print("  IR_TOO_CLOSE = " + str(suggestedStop))
  print("  IR_CLEAR     = " + str(suggestedClear) + "   (~" + str(int(stopDistanceCm + clearMarginCm)) + " cm)")
  print("  gap = " + str(suggestedStop - suggestedClear) + ", largest measured std = "
        + str(round(noise, 1)) + " -> gap is " + str(round((suggestedStop - suggestedClear) / noise, 1))
        + "x the noise")

  print("\nSaved " + plotFilename.name)
  plt.show()


if __name__ == "__main__":
  if not (len(sys.argv) > 1 and sys.argv[1] == "plot"):
    collect()
  plot()
