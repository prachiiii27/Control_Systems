# ES 245: Control Systems, Course Project (Fall 2026)

**Controlling a car on a tilting beam**: a servo driven four bar linkage tilts a beam, an ultrasonic sensor measures where a toy car is, and an Arduino Mega closes the loop.
Lab Setup 5, Department of Mechanical Engineering, IIT Gandhinagar.

| | Part 1: Time domain | Part 2: Frequency domain |
|---|---|---|
| Model | First principles: G(s) = 1.962/s² (m/rad) | Identified from chirp tests: G(s) = 2.591/(s² + 0.4789 s + 0.002551) (cm/deg) |
| Controllers | P, PD, PID (2DOF, derivative on measurement) | Lead, lag, lead-lag (Tustin, Ts = 40 ms) |
| Best hardware result | PID, 0.13 cm final error | Lead-lag, 0.09 to 0.43 cm final error |
| Report | [Part1/report](Part1/report/ES245_Part1_Report.pdf) | [Part2/report](Part2/report/ES245_Part2_final.pdf) |
| Video | [youtu.be/amDnMqpV4Dc](https://youtu.be/amDnMqpV4Dc) | [youtu.be/xoJ8jDUoAdQ](https://youtu.be/xoJ8jDUoAdQ) |

## Team

| Member | Roll number |
|---|---|
| Prachi Jindal | 24110264 |
| Sejal Sharbidre | 24110322 |
| Dhvani Shah | 24110325 |

## The rig

| Item | Value |
|---|---|
| Controller | Arduino Mega |
| Actuator | MG996R servo on pin 11, beam level at 90°, limited to ±30° |
| Sensor | HC-SR04 ultrasonic, TRIG pin 9, ECHO pin A2, 30 ms echo timeout |
| Linkage | horn d = 5 cm, coupler l = 11 cm, beam pin L = 25 cm from pivot |
| Car | 36.34 g toy car, beam centre at a sensor reading of 22.12 cm |
| Main nonlinearity | wheel friction: the car only moves once the servo tilts about 7° to 9° |

## Repository layout

```
Control_Systems/
├── Part1/                 Tasks 1 to 4: modelling, MATLAB/Simulink analysis, PID on hardware
│   ├── task/              Assignment sheet
│   ├── report/            Final submitted report (PDF)
│   ├── arduino/           P/PD/PID sketch used for all hardware runs
│   ├── matlab/            Analysis script and log tools
│   ├── data/              Hardware log of the P, PD and PID runs, measured metrics
│   └── figures/           Hardware plots
└── Part2/                 Tasks 5 to 8: system ID, root locus/Nyquist/Bode, lead and lag compensation
    ├── task/              Assignment sheet and the TA's chirp sketch
    ├── report/            Final submitted report (PDF)
    ├── latex/             LaTeX source of the report with all figures
    ├── arduino/           Chirp test and compensator sketches
    ├── matlab/            Identification and design script, serial logger
    └── data/              Four chirp logs and six hardware logs
```

Each part has its own README with the task list, results and how to run the code.

## Software needed
- Arduino IDE (Servo library is built in)
- MATLAB with Control System Toolbox, System Identification Toolbox and Simulink
- Any LaTeX editor (Overleaf works) to rebuild the Part 2 report
