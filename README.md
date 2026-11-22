# ES 245: Control Systems, Course Project (Fall 2026)

Car on beam controlled by a servo driven four bar linkage, Lab Setup 5, IIT Gandhinagar.

**Team:** Dhvani Shah (24110325), Sejal Sharbidre (24110322), Prachi Jindal (24110264)

**Hardware demo video (Part 2):** https://youtu.be/xoJ8jDUoAdQ

## Hardware
- Arduino Mega, MG996R servo (pin 11), HC-SR04 ultrasonic sensor (TRIG 9, ECHO A2)
- Servo horn d = 5 cm, link l = 11 cm, beam attachment L = 25 cm, beam level at servo 90 deg

## Part 1: Time domain analysis and PID control
Linearised model G(s) = (g d / L) / s^2, P, PD and PID design in MATLAB and Simulink, then tested on the hardware.

| Folder | Contents |
|---|---|
| `Part1/report` | Submitted report (PDF) |
| `Part1/arduino` | `ES245_CarBeam_Final.ino` (final controller), `ES245_PID_Logger.ino` (Task 4 logging sketch) |
| `Part1/matlab` | `ES245_Part1_Analysis.m` (Tasks 2 and 3), `ES245_Log_Serial.m` (records a run to CSV), `ES245_Analyse_Log.m` (measures runs, compares with model) |
| `Part1/figures` | Hardware P, PD, PID plots and the metrics table |

## Part 2: System identification, frequency domain analysis and compensation
- **Task 5:** four chirp tests, first 10 s removed, structured `tfest` fit:
  G(s) = 2.591 / (s^2 + 0.4789 s + 0.002551) cm/deg, 60.6 % fit, about 0.09 s delay
- **Task 6:** root locus, Nyquist and Bode. Phase margin 16.9 deg (8.8 deg with delay)
- **Task 7:** lead C(s) = 1.966 (0.791 s + 1)/(0.0791 s + 1) with prefilter, phase margin 61.7 deg, 0 % overshoot, 2.1 s settling. Lag and lead-lag designed for comparison
- **Task 8:** Tustin discretisation at Ts = 0.04 s. The lead reaches the target in about 2 s but friction leaves a 1.7 to 4.8 cm offset; the lead-lag removes it (0.09 to 0.43 cm)

| Folder | Contents |
|---|---|
| `Part2/report` | Submitted report (PDF) |
| `Part2/latex` | LaTeX source and figures (compile with pdfLaTeX) |
| `Part2/arduino` | `ES245_Chirp_Setup5.ino` (chirp test), `ES245_LeadLag_Target.ino` (lead, lag, lead-lag with settable target), simple lead and lead-lag versions |
| `Part2/matlab` | `ES245_Part2_Solve.m` (Tasks 5 to 7), `ES245_Chirp_Logger.m` |
| `Part2/data/chirp` | The four chirp logs (G21 = group, A = amplitude in deg, f = end frequency setting) |
| `Part2/data/hardware` | Lead and lead-lag hardware logs for targets 10, 22.12 and 35 cm |

## How to run
1. **Arduino:** open the `.ino` in the Arduino IDE, upload to the Mega, open Serial Monitor at 115200 baud with "Newline". For `ES245_LeadLag_Target.ino`: `1`/`2`/`3` choose lead/lag/lead-lag, `T 18` sets an 18 cm target, `S` starts, `X` stops.
2. **MATLAB:** copy `Part2/data/chirp/*.txt` next to `ES245_Part2_Solve.m` and press Run (needs Control System and System Identification toolboxes).
