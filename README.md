# Hybrid BCI (EEG + EMG) for 5-DOF Robotic Arm Control

This repository contains the software and physical modeling to control a 5-Degree-of-Freedom (DOF) laboratory robotic arm (based on the PhantomX Reactor) using a multi-modal Brain-Computer Interface (BCI). It was developed as part of my MSc in Neurotechnology, specifically the Neuroprosthetics course, to merge high-level intent from brainwaves (EEG) with low-level execution from muscle activity (EMG).

## 🧠 System Architecture

The project bridges human neuroscience with physical robotic kinematics:

1. **Python Client (EEG Processing):** - Receives EEG data via Lab Streaming Layer (LSL) from a Bitbrain headset.
   - Extracts Mu-band (8-13 Hz) features from the motor cortex channels (C3, Cz, C4).
   - Classifies user intent (e.g., Hand vs. Feet motor imagery) to select spatial target routes.
2. **Arduino Firmware (Hardware & EMG):**
   - Drives 5 Dynamixel AX-12 servos.
   - Samples an analog EMG sensor at 200 Hz via hardware interrupts to control the end-effector (gripper).
   - Manages dynamic "Hybrid" (EEG starts the route, EMG controls grasp) and "Direct" (EEG-only validation) control paths.

## 📐 Robotic Kinematics

The physical execution relies on custom Forward and Inverse Kinematics models, utilizing the kinematic decoupling technique. The robot consists of 5 revolute joints ($q_1$ through $q_5$) mapping to the Base, Shoulder, Elbow, Wrist, and Wrist Rotation. 

### Robot Dimensions
| Segment | Length (mm) |
| :--- | :--- |
| $l_0$ | 86.8 |
| $l_1$ | 31.0 |
| $l_2$ | 150.2 |
| $l_3$ | 146.3 |
| $l_4$ | 70.0 |
| $l_5$ | 66.3 |

### Mechanical Constraints
To ensure safe operation and prevent self-collision or servo damage, the system strictly enforces the following joint limits (in radians):

* **$q_1$ (Base):** -2.62 to 2.62
* **$q_2$ (Shoulder):** -0.33 to 2.97
* **$q_3$ (Elbow):** -2.89 to 0.26
* **$q_4$ (Wrist):** -1.83 to 1.86
* **$q_5$ (Wrist Rot):** -1.05 to 4.19


## ⚙️ Core Features

* **Real-time EEG Classification:** Uses logarithmic power ratios and Euclidean distance to classify motor imagery against a dynamically calibrated resting baseline.
* **Dynamic EMG Calibration:** Calculates resting and flexing thresholds on the fly to account for sensor placement variations and muscle fatigue.
* **Hybrid Control Paradigm:** * **Intent to Move:** EEG selects the trajectory (e.g., navigating to a grasp point).
    * **Intent to Act:** EMG thresholding allows the user to manually trigger the gripper during the transport phase.
* **Safety Protocols:** Includes a 700ms gripper lockout to prevent rapid oscillation, and hardware timeout routines.

## 🛠️ Hardware Requirements

* **Robot:** PhantomX Reactor.
* **Microcontroller:** Arduino-compatible board.
* **Sensors:** Analog EMG sensor (connected to `A0`), Bitbrain EEG amplifier.

## 💻 Software Dependencies

**Arduino:**
* `ax12.h` (Dynamixel Servo Control)
* `TimerOne.h` (Hardware timer for EMG sampling)

**Python 3.x:**
* `pylsl` (Lab Streaming Layer)
* `pyserial` (Arduino communication)
* `numpy`, `scipy` (Signal processing and filtering)

## 🚀 Setup and Usage

1. **Upload Firmware:** Flash `GR12_D3a.ino` to the Arduino. Ensure servos are powered and connected via the Dynamixel bus.
2. **Start EEG Stream:** Connect the Bitbrain headset and begin broadcasting data over your local LSL network.
3. **Launch Python Client:** Execute `GR12_D3b.py` (verify your `SERIAL_PORT` matches the Arduino).
4. **Calibrate & Run (Terminal UI):**
   * Press `R` to calibrate the resting EEG baseline (requires 15 seconds of relaxation).
   * Press `1` and `2` to train the classification prototypes for different motor imagery classes.
   * Press `C` to recalibrate the physical EMG thresholds on the Arduino.
   * Press `E` to arm the automatic BCI route selection, or use manual overrides (`D`, `F`, `S`, `X`) for debugging trajectories.
