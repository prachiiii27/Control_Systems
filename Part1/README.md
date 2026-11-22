# Part 1: Time Domain Analysis and PID Control

Final report: [`report/ES245_Part1_Report.pdf`](report/ES245_Part1_Report.pdf) · Assignment: [`task/ES245_Project_Part1_Task.pdf`](task/ES245_Project_Part1_Task.pdf) · Video: https://youtu.be/amDnMqpV4Dc

## Tasks and what we did

| Task | Asked | What we did |
|---|---|---|
| 1. Define the system | Dynamics, transfer function, state space | Lagrangian derivation, linkage relation α ≈ (d/L)θ, linearised about a level beam: **G(s) = K/s², K = gd/L = 1.962 m/(rad s²)** |
| 2. Analysis | Poles/zeros, open loop step, system type, Simulink model | Double pole at s = 0: Type 2, unstable in open loop, the car runs off the beam |
| 3. PID control | P, PD, PID in Simulink, settling < 5 s, overshoot < 5 %, PID Tuner | P can never settle (poles stay on the imaginary axis). PD and PID meet both specs with a 2DOF block (derivative on measurement) |
| 4. Demonstration | Run the controllers on the hardware | One Arduino control law for all three, only the gains change, with anti windup and a friction kick |

## Results

**Simulink (Task 3)**

| Controller | Gains | Overshoot | Settling (5 %) |
|---|---|---|---|
| P | any Kp | never settles | never |
| PD | Kp = 1.15, Kd = 1.20 | 1.67 % | 2.44 s |
| PID | Kp = 2.40, Ki = 0.80, Kd = 1.70 | 0.68 % | 2.12 s |

The same PID gains in a standard (1DOF) PID block give about 20 % overshoot, so putting the derivative on the measurement mattered as much as the gains.

**Hardware (Task 4)**, from [`data/hw_metrics.csv`](data/hw_metrics.csv)

| Controller | Gains Kp/Kd/Ki/kick | Settling into ±2 cm | Final error |
|---|---|---|---|
| P | 2.5 / 0 / 0 / 4 | never, swings 7 to 38 cm | still swinging |
| PD | 4 / 2.1 / 0 / 3 | 12.6 s | 0.19 cm |
| PID | 4 / 2.1 / 0.8 / 3 | 75.1 s | −0.13 cm |

**Why the hardware is slower than the simulation**
- **Friction:** the car stays still until the servo reaches about 7° to 9°, so the gains had to be raised about 6 times above the design.
- **Delay:** the higher gain moved the crossover from 2.5 to 7.4 rad/s, where about 70 ms of servo lag uses up almost all the phase margin (PD margin 75.6° ideal, about 0° with lags).
- **PD vs PID resting point:** when the car stops, PD only commands Kp·e, too weak to beat friction within about ±2 cm, so it can rest anywhere in that band. The PID integral keeps growing until the car reaches the centre.

## Files

| File | What it does |
|---|---|
| `arduino/ES245_CarBeam.ino` | P, PD and PID on one control law. Serial commands: `P`, `PD` or `PID` to choose, `START`, `STOP`, `KP 4 KD 2.1 KI 0.8 KICK 3` to change gains |
| `matlab/ES245_Analysis_v2.m` | Tasks 2 and 3: plant gain, poles/zeros, step responses, controller gains |
| `matlab/ES245_Log_Serial.m` | Records one hardware run from the Arduino to a CSV file |
| `matlab/ES245_Analyse_Log.m` | Measures rise time, overshoot and settling from a log and compares with the model |
| `data/hardware_P_PD_PID_log.txt` | Serial log of the three hardware runs (40 ms samples) |
| `data/hw_metrics.csv` | Metrics measured from that log |
| `figures/` | Hardware responses for P, PD, PID, comparison and hardware against model |

Note on the log: the controller label printed in the header of trials 2 and 3 is swapped. Go by the gains: Ki = 0 is the PD run, Ki = 0.8 is the PID run.

The Simulink `.slx` models are not included; they are fully described in Figures 8 and 10 and Section 4.5 of the report.

## How to run
1. Upload `arduino/ES245_CarBeam.ino` to the Mega and open Serial Monitor at 115200 baud, line ending "Newline".
2. Put the car near the sensor end, type `PD` (or `P`, `PID`), then `START`. Type `STOP` to end; a `SUMMARY` line prints overshoot, settling time and final error.
3. In MATLAB run `ES245_Analysis_v2.m` for Tasks 2 and 3.
