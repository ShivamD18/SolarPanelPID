# Single-Axis Solar Tracker: Gazebo Simulation and PD Control

![Simulator](https://img.shields.io/badge/simulator-Gazebo-orange)
![Language](https://img.shields.io/badge/controller-Python-blue)
![Controller](https://img.shields.io/badge/control-PD%20state%20feedback-green)
<!-- TODO: Add license badge -->

<p align="center">
  <img src="docs/images/hero.png" alt="Solar tracker in Gazebo" width="70%">
  <br>
  <em>PLACEHOLDER: Screenshot or GIF of the tracker in Gazebo.</em>
</p>

## Overview

This project models and simulates a single-axis solar tracking system in Gazebo and controls it with a Python-based feedback controller. The panel follows a prescribed sun trajectory over a 25-second interval, and its tracking performance is evaluated under five scenarios: a nominal baseline, wind disturbance, sensor inaccuracy, actuator saturation, and a gearbox with a weak motor.

Developed for **MTE604: Electromechanical System Modeling and Design** at Toronto Metropolitan University.

## System Description

**Mechanical model:** a fixed support pole and a rotating solar panel connected by a revolute joint, with joint limits, damping, and friction defined in the SDF model.

**Panel dynamics:** modeled as a rotational mass-damper system:

$$I\ddot{\theta} + b\dot{\theta} = \tau$$

**Reference trajectory** (sweeps from -1 rad to +1 rad, with $T = 25\ \text{s}$):

$$\theta_{ref}(t) = -\cos\left(\frac{\pi t}{T}\right)$$

**Controller:** PD-equivalent state feedback, with gains chosen by matching the closed loop to a critically damped second-order response ($\zeta = 1$). Final gains: $K_p = 12$, $K_d = 15$. The controller runs through Gazebo's joint position controller plugin, while a Python script generates the reference and logs panel motion.

<p align="center">
  <img src="docs/images/model.png" alt="Gazebo model" width="60%">
  <br>
  <em>PLACEHOLDER: Gazebo model or CAD view of the pole and panel.</em>
</p>

## Scenarios

| # | Scenario | What is modeled |
|---|---|---|
| 1 | Nominal | Ideal sensing, no disturbance |
| 2 | Wind disturbance | Oscillating torque, random turbulence, and gust spikes |
| 3 | Sensor inaccuracy | Gaussian noise, quantization, and slow drift on the measured angle |
| 4 | Actuator saturation | Capped motor torque and restricted joint speed |
| 5 | Gearbox | Weak motor paired with a 10:1 gear ratio |

## Results

Tracking error was computed from the simulation logs as Mean Absolute Error (MAE) and Root Mean Squared Error (RMSE), in radians.

| Scenario | MAE (rad) | RMSE (rad) |
|---|---|---|
| 1. Nominal | 0.0534 | 0.1019 |
| 2. Wind disturbance | 0.1009 | 0.1528 |
| 3. Sensor inaccuracy | 0.1871 | 0.2294 |
| 4. Actuator saturation | 0.1395 | 0.2518 |
| 5. Gearbox (10:1) | 0.1733 | 0.2944 |

**Key findings**

- The controller tracks best in the nominal case, with only an initial transient.
- Wind and sensor error add oscillations and deviations, but the panel still follows the overall trajectory.
- Saturation and the weak-motor gearbox case introduce lag, since available torque limits how fast the panel can respond.
- The system stayed stable in all five scenarios.

| Nominal | Wind Disturbance | Sensor Error |
|:---:|:---:|:---:|
| ![S1](docs/images/scenario1.png) | ![S2](docs/images/scenario2.png) | ![S3](docs/images/scenario3.png) |

| Actuator Saturation | Gearbox |
|:---:|:---:|
| ![S4](docs/images/scenario4.png) | ![S5](docs/images/scenario5.png) |

<!-- TODO: Add the plot images above to docs/images/ (reference vs. actual angle for each scenario) -->

## Getting Started

### Prerequisites

- Gazebo <!-- TODO: add version, e.g. Gazebo Harmonic -->
- Python 3 <!-- TODO: add required packages -->

### Run

```bash
# TODO: replace with your actual commands
git clone https://github.com/<your-username>/<your-repo>.git
cd <your-repo>
gz sim worlds/solar_tracker.sdf
python controller.py --scenario 1
```

## Repository Structure

```
.
├── worlds/        # Gazebo world and SDF model (TODO)
├── src/           # Python controller and scenario scripts (TODO)
├── data/          # Logged tracking data for each scenario (TODO)
├── docs/images/   # Plots and screenshots
└── README.md
```

<!-- TODO: Update to match the actual repository layout -->

## Authors

Zakareya Gharib, Farooq Nomani, and Shivam Dave.

## License

<!-- TODO: Choose a license and add a LICENSE file -->
Licensed under the **[LICENSE NAME]**. See `LICENSE` for details.
