# Puffin - photoresistor calibration
#
# pip install pyserial numpy matplotlib
#
# usage:
#   python light_calibration.py        collect readings, then plot
#   python light_calibration.py plot   re-plot from light_calibration_data.csv


import csv
import re
import sys
import time
from pathlib import Path

import numpy as np
import matplotlib.pyplot as plt

arduinoComPort = "COM3"
baudRate = 9600

# (name, instructions), darkest first
conditions = [
  ("covered",    "cover the photoresistor completely with a finger"),
  ("shaded",     "hold your hand flat ~10 cm above it"),
  ("room",       "normal room light, nothing blocking it"),
  ("flashlight", "shine a phone flashlight at it from ~20 cm"),
]
darkConditions = ["covered", "shaded"]  # the LED should turn on for these
readingsPerCondition = 20

adcMax = 1023
vRef = 5.0

here = Path(__file__).resolve().parent
dataFilename = here / "light_calibration_data.csv"
plotFilename = here / "light_calibration_plot.png"
mainSketch = here.parent / "puffin" / "puffin.ino"


def collect():
  import serial

  serialPort = serial.Serial(arduinoComPort, baudRate, timeout=1)
  time.sleep(2)  # opening the port resets the Arduino

  rows = []  # (condition, raw_sensor_value)

  try:
    for name, instructions in conditions:
      input("\n" + name + ": " + instructions + ", then press Enter...")

      serialPort.reset_input_buffer()
      readings = []
      while len(readings) < readingsPerCondition:
        line = serialPort.readline().decode(errors="replace").strip()
        if line.isdigit():
          readings.append(int(line))

      print("  " + name + " -> mean = " + str(round(np.mean(readings), 1))
            + ", std = " + str(round(np.std(readings), 1))
            + " (" + str(len(readings)) + " readings)")
      rows += [(name, r) for r in readings]

  except KeyboardInterrupt:
    print("\nStopped early by user.")

  finally:
    serialPort.close()

  with open(dataFilename, "w", newline="") as csvFile:
    writer = csv.writer(csvFile)
    writer.writerow(["condition", "raw_sensor_value"])
    writer.writerows(rows)
  print("\nSaved " + str(len(rows)) + " readings to " + dataFilename.name)


# returns {condition: array of readings}, in the order they were recorded
def loadReadings():
  readings = {}
  with open(dataFilename, newline="") as csvFile:
    for row in csv.DictReader(csvFile):
      readings.setdefault(row["condition"], []).append(float(row["raw_sensor_value"]))
  return {name: np.array(values) for name, values in readings.items()}


# reads a "const int NAME = value;" line from puffin.ino
def readThreshold(name):
  match = re.search(r"const\s+int\s+" + name + r"\s*=\s*(\d+)", mainSketch.read_text())
  return int(match.group(1)) if match else None


def plot():
  readings = loadReadings()
  names = list(readings)
  means = np.array([readings[n].mean() for n in names])
  stds  = np.array([readings[n].std() for n in names])
  colors = ["tab:purple" if n in darkConditions else "tab:orange" for n in names]

  fig, ax = plt.subplots(figsize=(9, 6))

  # individual readings with a little sideways jitter so they don't overlap
  rng = np.random.default_rng(0)
  for i, name in enumerate(names):
    jitter = rng.uniform(-0.12, 0.12, len(readings[name]))
    ax.scatter(i + jitter, readings[name], s=10, alpha=0.3, color=colors[i])
  for i in range(len(names)):
    ax.errorbar(i, means[i], yerr=stds[i], fmt="o", color=colors[i], capsize=6, markersize=8)

  # legend entries for the two groups
  ax.scatter([], [], color="tab:purple", label="Should turn LED on")
  ax.scatter([], [], color="tab:orange", label="Should leave LED off")

  # thresholds from puffin.ino
  dark = readThreshold("LIGHT_DARK")
  bright = readThreshold("LIGHT_BRIGHT")
  if dark:
    ax.axhline(dark, color="tab:red", linestyle="--",
               label="LIGHT_DARK = " + str(dark) + " (LED on below)")
    ax.axhspan(0, dark, color="tab:red", alpha=0.06)
  if bright:
    ax.axhline(bright, color="tab:green", linestyle="--",
               label="LIGHT_BRIGHT = " + str(bright) + " (LED off above)")

  ax.set_xticks(range(len(names)), names)
  ax.set_xlabel("Lighting condition")
  ax.set_ylabel("Sensor reading (0-1023)")
  volts = ax.secondary_yaxis("right", functions=(lambda r: r * vRef / adcMax,
                                                 lambda v: v * adcMax / vRef))
  volts.set_ylabel("A1 voltage (V)")
  ax.set_title("Photoresistor Calibration")
  ax.set_ylim(0, adcMax)
  ax.grid(alpha=0.3, axis="y")
  ax.legend(loc="upper left")
  fig.tight_layout()
  fig.savefig(plotFilename, dpi=200)

  # summary for the write-up
  print("Per-condition readings:")
  for name, m, s in zip(names, means, stds):
    print("  " + name.ljust(12) + " mean = " + str(round(m, 1)).rjust(6) + ", std = " + str(round(s, 1)))

  darkNames = [n for n in names if n in darkConditions]
  litNames = [n for n in names if n not in darkConditions]
  if darkNames and litNames:
    # keep 2 std away from each group so normal noise can't cross a threshold
    darkTop = max(readings[n].mean() + 2 * readings[n].std() for n in darkNames)
    litBottom = min(readings[n].mean() - 2 * readings[n].std() for n in litNames)
    gap = litBottom - darkTop
    if gap > 0:
      print("\nSuggested thresholds (gap between dark and lit readings split in thirds):")
      print("  LIGHT_DARK   = " + str(int(round(darkTop + gap / 3))))
      print("  LIGHT_BRIGHT = " + str(int(round(darkTop + 2 * gap / 3))))
    else:
      print("\nThe dark and lit conditions overlap (within 2 std) - no thresholds can"
            " separate them reliably. Try a different resistor or different conditions.")

  print("\nSaved " + plotFilename.name)
  plt.show()


if __name__ == "__main__":
  if not (len(sys.argv) > 1 and sys.argv[1] == "plot"):
    collect()
  plot()
