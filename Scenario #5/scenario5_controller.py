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


CSVFileMaker = "scenario5_gearbox_data.csv"


LowerJoint = -1.57
UPPERJoint = 1.57


RatioForGears = 10.0
alphaMotor = 0.22


ESTIMATED_ALPHA = 0.2
ESTIMATED_ACTUAL = 0.0


def clamp(x, lo, hi):
    return max(lo, min(hi, x))


def ReferenceTheta(t):
    return -math.cos(math.pi * t / T)


def CMDSending(angle):

    angle = clamp(angle, LowerJoint, UPPERJoint)

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
CMDSending(0.0)
time.sleep(1)


CMDForGears = 0.0


with open(CSVFileMaker, "w", newline="") as file:

    writer = csv.writer(file)

    writer.writerow([
        "Time_Secondseconds",
        "Target_SUN",
        "CMDForGearboxRad",
        "SolarPanelRadians",
        "SolarPanelRadians"
    ])

    for k in range(N):

        t = k * DT

        ref = ReferenceTheta(t)

        CMDForGears = CMDForGears + (alphaMotor / RatioForGears) * (ref - CMDForGears)

        CMDSending(CMDForGears)

        time.sleep(max(DT - 0.06, 0))

        actual = JointPosition()

        if actual is None:
            ESTIMATED_ACTUAL = ESTIMATED_ACTUAL + ESTIMATED_ALPHA * (CMDForGears - ESTIMATED_ACTUAL)
            actual = ESTIMATED_ACTUAL

        error = ref - actual

        writer.writerow([
            round(t, 3),
            round(ref, 5),
            round(CMDForGears, 5),
            round(actual, 5),
            round(error, 5)
        ])


print("Scenario 5 complete")
