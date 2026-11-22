# Part 2: Frequency Domain Analysis and Compensator Design

Final report: [`report/ES245_Part2_Report.pdf`](report/ES245_Part2_Report.pdf) · Assignment: [`task/ES245_Project_Part2_Task.pdf`](task/ES245_Project_Part2_Task.pdf) · Video: https://youtu.be/xoJ8jDUoAdQ

## Tasks and what we did

| Task | Asked | What we did |
|---|---|---|
| 5. System characterisation | Chirp test, transfer function with n4sid, Bode plot | 4 chirp runs, first 10 s removed, spikes fixed. Unconstrained n4sid gave an unphysical model, so a structured 2 pole `tfest` fit was used |
| 6. Root locus, Nyquist, Bode | Stability, margins, check specs (overshoot < 5 %, settling < 5 s) | Stable but poorly damped: phase margin 16.9° (8.8° with the 90 ms delay), 62 % overshoot, 12 s settling. Specs fail |
| 7. Lead or lag compensator | Design and verify with Bode and step plots | Lead chosen (needs about 50° of extra phase). Lag and lead-lag designed for comparison |
| 8. Hardware implementation | Run on the rig, show stability and response to perturbations | Tustin discretisation at 40 ms, settable target, friction kick. Hand pushes freed the car where friction held it and served as perturbations |

## Key results

**Identified plant (Task 5)**

G(s) = 2.591 / (s² + 0.4789 s + 0.002551) cm/deg, 60.6 % fit, about 0.09 s delay

**Compensators (Task 7)**

| Compensator | Transfer function |
|---|---|
| Lead | C(s) = 1.966 (0.791 s + 1)/(0.0791 s + 1), prefilter F(s) = 1/(0.791 s + 1) |
| Lag | C(s) = 0.0774 (s + 0.0343)/(s + 0.00265) |
| Lead-lag | Lead × (s + 0.2)/(s + 0.02), same prefilter |

| Compensator | Phase margin | With 90 ms delay | Overshoot | Settling | Specs met |
|---|---|---|---|---|---|
| None | 16.9° | 8.8° | 78.7 % | 24.0 s | No |
| Lag only | 49.6° | 47.9° | 23.5 % | 13.7 s | No |
| Lead + prefilter | 61.7° | 41.1° | 0 % | 2.10 s | Yes |
| Lead-lag + prefilter | 59.2° | 38.5° | 0.4 % | 1.94 s | Yes |

In simulation the lead is slightly better: the lag stage costs about 2.5° of phase at the crossover, and the model has no friction for it to fix.

**Hardware (Task 8)**

| Run | Final error |
|---|---|
| Lead, targets 10 / 22.12 / 35 cm | 4.82 / 1.65 / 0.33 cm (friction stops the car short) |
| Lead-lag, targets 10 / 22.12 / 35 cm | 0.15 / 0.43 / 0.09 cm |

On the hardware the lead-lag wins: its extra low frequency gain keeps raising the servo angle until friction breaks, so the offset disappears. After a hand push it brought the car back within 1 to 2 s.

## Files

| File | What it does |
|---|---|
| `task/Chirp_Signal_Ultrasonic_TA.ino` | Chirp sketch provided by the TA |
| `arduino/ES245_Chirp_Setup5.ino` | Our version of the chirp test for Setup 5 (changes marked CHANGED) |
| `arduino/ES245_LeadLag_Target.ino` | Final controller: lead, lag and lead-lag with a settable target |
| `arduino/ES245_Lead_Simple.ino`, `ES245_LeadLag_Simple.ino` | Short versions with a fixed centre target, easier to read |
| `matlab/ES245_Part2_Solve.m` | Tasks 5 to 7: cleaning, identification, Bode, root locus, Nyquist, margins, compensator design, Tustin coefficients |
| `matlab/ES245_Chirp_Logger.m` | Saves the Arduino serial output to a text file |
| `data/chirp/` | `G21A20f11.txt` etc.: G21 = group, A = chirp amplitude (deg), f = frequency setting |
| `data/hardware/` | Lead and lead-lag logs for targets 10, 22.12 and 35 cm (time, target, filtered target, position, servo angle) |
| `latex/` | Report source (`ES245_Part2_Report.tex`) and its 18 figures |

## How to run
1. **Chirp test:** upload `ES245_Chirp_Setup5.ino`, log the serial output with `ES245_Chirp_Logger.m`.
2. **Identification and design:** copy `data/chirp/*.txt` next to `matlab/ES245_Part2_Solve.m` and press Run.
3. **Controller:** upload `ES245_LeadLag_Target.ino`, Serial Monitor at 115200 baud with "Newline", then:
   - `1`, `2` or `3` chooses lead, lag or lead-lag
   - `T 18` sets the target to 18 cm (8 to 36 cm allowed, works while running)
   - `S` starts, `X` stops
4. **Report:** upload `latex/` to Overleaf (or zip it) and compile with pdfLaTeX.
