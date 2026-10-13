import math
import subprocess
import time
import csv
import re


# Set for 25 seconds
T = 25.0


# Wish to have 100 samples of data
N = 100


# the time set, which can be manipulated for more samples, is set 
DT = T / N


CMDForTopic = "/model/solartracker/joint/tracking_joint/0/cmd_pos"


StateForTOPIC = "/world/tracker_world/model/solartracker/joint_state"


csvFILEMALER = "PanelTrackingData.csv"




def ReferenceTheta(t):


    return -math.cos(math.pi * t / T)




def CMDSending(angle):


    subprocess.run([
        "ign",
        "topic",
        "-t",
        CMDForTopic,
        "-m",
        "ignition.msgs.Double",
        "-p",
        f"data: {angle}"
    ],
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL
    )




def JointPosition():


    proc = subprocess.Popen(
        [
            "ign",
            "topic",
            "-e",
            "-n",
            "1",
            "-t",
            StateForTOPIC
        ],
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True
    )


    output, _ = proc.communicate()


    match = re.search(
        r'name:\s*"tracking_joint".*?position:\s*([-0-9.eE]+)',
        output,
        re.DOTALL
    )


    if match:


        return float(match.group(1))


    return None




print("Resetting panel to 0 rad")


for _ in range(20):


    CMDSending(0.0)


    time.sleep(0.05)




time.sleep(1)




times = []
refs = []
actuals = []
errors = []




print("Starting solar tracking test")


print("Sun starts at -1 rad")


print("Panel starts at 0 rad")




with open(csvFILEMALER, "w", newline="") as file:


    writer = csv.writer(file)


    writer.writerow([
        "Time_Secondseconds",
        "Target_SUN",
        "SolarPanelRadians",
        "SolarPanelRadians"
    ])


    for k in range(N):


        t = k * DT


        ref = ReferenceTheta(t)


        CMDSending(ref)


        time.sleep(DT)


        actual = JointPosition()


        if actual is not None:


            error = ref - actual


            times.append(t)
            refs.append(ref)
            actuals.append(actual)
            errors.append(error)


            writer.writerow([
                round(t, 3),
                round(ref, 5),
                round(actual, 5),
                round(error, 5)
            ])


            print(
                f"Sample {k+1}/{N} | "
                f"t={t:5.2f} s | "
                f"sun={ref: .3f} | "
                f"panel={actual: .3f} | "
                f"error={error: .3f}"
            )




print("Returning panel to 0 rad")


for _ in range(20):


    CMDSending(0.0)


    time.sleep(0.05)




if errors:


    mae = sum(abs(e) for e in errors) / len(errors)


    rmse = math.sqrt(
        sum(e ** 2 for e in errors) / len(errors)
    )


    print()


    print("Tracking Performance")


    print("--------------------")


    print("Samples =", len(errors))


    print("MAE  =", round(mae, 5))


    print("RMSE =", round(rmse, 5))




print()


print("Data saved to")


print(csvFILEMALER)
