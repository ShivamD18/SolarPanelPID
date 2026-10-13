import math
import subprocess
import time
import csv
import re


T = 25.0
N = 100
DT = T / N


CMDForTopic = "/model/solartracker/joint/tracking_joint/0/cmd_pos"
StateForTopic = "/world/tracker_world/model/solartracker/joint_state"


CSVFileMaker = "scenario3_WindDisturbance_data.csv"


LowerJoint = -1.57
UPPERJoint = 1.57


# disturbance parameters
WIND_AMPLITUDE = 0.15
WIND_FREQ = 0.8


ESTIMATED_ALPHA = 0.2
ESTIMATED_ACTUAL = 0.0


def clamp(x, lo, hi):
    return max(lo, min(hi, x))


def ReferenceTheta(t):
    return -math.cos(math.pi * t / T)


def WindDisturbance(t):
    return WIND_AMPLITUDE * math.sin(WIND_FREQ * t)


def CMDSending(angle, repeats=4):
    angle = clamp(angle, LowerJoint, UPPERJoint)
    for _ in range(repeats):
        subprocess.run(
            [
                "ign", "topic",
                "-t", CMDForTopic,
                "-m", "ignition.msgs.Double",
                "-p", f"data: {angle}"
            ],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL
        )
        time.sleep(0.015)


def JointPosition():
    try:
        proc = subprocess.Popen(
            ["ign", "topic", "-e", "-n", "1", "-t", StateForTopic],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True
        )
        output, _ = proc.communicate(timeout=1.2)
    except Exception:
        return None

    m = re.search(r'position:\s*([-0-9.eE]+)', output)
    if m:
        return float(m.group(1))
    return None


print("Resetting panel to 0 rad...")
CMDSending(0.0, repeats=20)
time.sleep(1)


with open(CSVFileMaker, "w", newline="") as file:

    writer = csv.writer(file)
    writer.writerow([
        "Time_Secondseconds",
        "Target_SUN",
        "SolarPanelRadians",
        "SolarPanelRadians",
        "WindDisturbance_rad"
    ])

    for k in range(N):

        t = k * DT

        ref = ReferenceTheta(t)
        wind = WindDisturbance(t)

        cmd = ref + wind

        CMDSending(cmd)

        time.sleep(max(DT - 0.06, 0))

        actual = JointPosition()

        if actual is None:
            ESTIMATED_ACTUAL = ESTIMATED_ACTUAL + ESTIMATED_ALPHA * (cmd - ESTIMATED_ACTUAL)
            actual = ESTIMATED_ACTUAL

        error = ref - actual

        writer.writerow([
            round(t, 3),
            round(ref, 5),
            round(actual, 5),
            round(error, 5),
            round(wind, 5)
        ])


print("Scenario 3 complete")
