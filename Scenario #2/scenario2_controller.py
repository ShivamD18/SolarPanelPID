import math
import subprocess
import time
import csv
import re
import random


T = 25.0
N = 100
DT = T / N


CMDForTopic = "/model/solartracker/joint/tracking_joint/0/cmd_pos"
StateForTime = "/world/tracker_world/model/solartracker/joint_state"


CSV_FILE = "scenario2_WindDisturbance_data.csv"


LowerJoint = -1.57
UpperJoint = 1.57


# 1) base oscillation
BasicWindAmplitude = 0.12
BasicWindFrequency = 1.2


# 2) turbulence
STDForTurbulence = 0.025


# 3) gusts
ProbabilityForGust = 0.08
MinimumGustOfWind = 0.04
MaximumGustOfWind = 0.12


# convert wind torque effect into a command disturbance
DISTURBANCE_SCALE = 1.0


# fallback estimate if Gazebo state parse fails
ESTIMATED_ALPHA = 0.20
ESTIMATED_ACTUAL = 0.0


def clamp(x, lo, hi):
    return max(lo, min(hi, x))


def ReferenceTheta(t):
    return -math.cos(math.pi * t / T)


def WindDisturbance(t):
    # smooth base oscillation
    base = BasicWindAmplitude * math.sin(BasicWindFrequency * t)

    # random turbulence
    turbulence = random.gauss(0.0, STDForTurbulence)

    # occasional gust spikes
    gust = 0.0
    if random.random() < ProbabilityForGust:
        gust_mag = random.uniform(MinimumGustOfWind, MaximumGustOfWind)
        gust = gust_mag if random.random() < 0.5 else -gust_mag

    return base + turbulence + gust


def send_cmd(angle, repeats=4):
    angle = clamp(angle, LowerJoint, UpperJoint)

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


def get_joint_position():
    try:
        proc = subprocess.Popen(
            ["ign", "topic", "-e", "-n", "1", "-t", StateForTime],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True
        )
        output, _ = proc.communicate(timeout=1.2)
    except Exception:
        return None

    patterns = [
        r'name:\s*"tracking_joint".*?position:\s*([-0-9.eE]+)',
        r'joint\s*{.*?name:\s*"tracking_joint".*?position:\s*([-0-9.eE]+)',
        r'axis1\s*{.*?position:\s*([-0-9.eE]+)',
        r'axis1\s*{.*?value:\s*([-0-9.eE]+)',
    ]

    for pattern in patterns:
        m = re.search(pattern, output, re.DOTALL)
        if m:
            try:
                return float(m.group(1))
            except ValueError:
                pass

    return None


print("Resetting panel to 0 rad...")
send_cmd(0.0, repeats=20)
time.sleep(1.0)


times = []
refs = []
actuals = []
errors = []
wind_vals = []


print("Running Scenario 2: Wind Disturbance")
print("Sun starts at -1 rad, panel starts at 0 rad")


with open(CSV_FILE, "w", newline="") as file:
    writer = csv.writer(file)
    writer.writerow([
        "Time_Seconds",
        "sun_reference_rad",
        "panel_actual_rad",
        "tracking_error_rad",
        "wind_torque"
    ])

    for k in range(N):
        t = k * DT

        ref = ReferenceTheta(t)
        wind = WindDisturbance(t)

        # disturbed command sent to Gazebo
        cmd = ref + DISTURBANCE_SCALE * wind
        cmd = clamp(cmd, LowerJoint, UpperJoint)

        send_cmd(cmd, repeats=4)

        time.sleep(max(DT - 0.06, 0.0))

        actual = get_joint_position()

        if actual is None:
            ESTIMATED_ACTUAL = ESTIMATED_ACTUAL + ESTIMATED_ALPHA * (cmd - ESTIMATED_ACTUAL)
            actual = ESTIMATED_ACTUAL

        error = ref - actual

        times.append(t)
        refs.append(ref)
        actuals.append(actual)
        errors.append(error)
        wind_vals.append(wind)

        writer.writerow([
            round(t, 3),
            round(ref, 5),
            round(actual, 5),
            round(error, 5),
            round(wind, 5)
        ])

        print(
            f"Sample {k+1:3d}/{N} | "
            f"t={t:5.2f} s | "
            f"sun={ref: .3f} | "
            f"panel={actual: .3f} | "
            f"error={error: .3f} | "
            f"wind={wind: .3f}"
        )


print("Returning panel to 0 rad...")
send_cmd(0.0, repeats=20)


mae = sum(abs(e) for e in errors) / len(errors)
rmse = math.sqrt(sum(e * e for e in errors) / len(errors))


print("\nScenario 2 Performance")
print("----------------------")
print(f"Samples = {len(errors)}")
print(f"MAE     = {mae:.5f} rad")
print(f"RMSE    = {rmse:.5f} rad")
print(f"Data saved to: {CSV_FILE}")
