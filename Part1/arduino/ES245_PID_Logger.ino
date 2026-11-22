/*
  ES 245 Course Project, Task 4: P, PD and PID control on Setup 5
  Car on beam, HC-SR04 ultrasonic sensor, MG996R servo, Arduino Mega

  AUTO LOGGING VERSION
    1. Car in the middle. Type  m1  then  s1   -> P test starts, recording starts
    2. Watch it for about 40 s (recording holds up to 48 s)
    3. Type  m0  -> controller stops, beam goes level, and the Arduino prints
                    the whole recording as CSV plus a results summary
    4. Copy everything from the Serial Monitor, paste into Notepad, save (P.txt)
    5. Click the "clear output" button, recentre the car, then  m2  s1  (PD)
    6. Same again with  m3  s1  (PID)
    Type  p  at any time to print the last recording again.

  Values already filled in: level 92.5 deg, DIR +1, centre 21.88 cm
    P   : Kp 2.5                     -> keeps oscillating (as theory predicts)
    PD  : Kp 4, Kd 2.1               -> settles in about 2 to 4 s
    PID : Kp 6, Ki 3, Kd 2.8, b 0.85 -> settles in about 2 to 3 s, no steady-state error
    Each mode adds a small friction kick (Ff) so the car does not stick.

  OTHER COMMANDS (Serial Monitor, 115200 baud, "Newline")
    l92.5  set level    c21.88  set centre    P I D B F + value  change a gain live (after m)
    ?      print settings
*/

#include <Servo.h>

// ---------------- Wiring ----------------
const int TRIG_PIN  = 9;
const int ECHO_PIN  = A2;
const int SERVO_PIN = 11;

// ---------------- Calibration (measured in the lab) ----------------
float servoLevel = 92.5;    // servo angle (deg) that makes the beam level
int   dir        = +1;      // +1: bigger servo angle sends the car away from the sensor
float centreCm   = 21.88;   // sensor reading with the car over the pivot (cm)

// ---------------- Limits ----------------
const float TH_MAX  = 30.0;   // max servo swing either side of level (deg)
const float POS_MIN = 2.0;    // readings outside this window are ignored
const float POS_MAX = 40.0;

// ---------------- Controller gains (deg/cm) ----------------
//                    Kp     Ki     Kd     b     Ff
const float GAINS[4][5] = {
  {0.00, 0.00, 0.00, 1.00, 0.0},   // m0 off
  {2.50, 0.00, 0.00, 1.00, 4.0},   // m1 P
  {4.00, 0.00, 2.10, 1.00, 3.0},   // m2 PD
  {6.00, 3.00, 2.80, 0.85, 4.0}    // m3 PID
};
const char* NAMES[4] = {"OFF", "P", "PD", "PID"};
const float I_ZONE = 6.0;     // integrate only when within 6 cm of the setpoint

// ---------------- Step test ----------------
const float STEP_HALF = 5.0;                 // setpoints: centre - 5 and centre + 5 cm
const unsigned long STEP_PERIOD = 10000;     // ms between setpoint jumps

// ---------------- Timing ----------------
const unsigned long TS_MS = 40;              // 25 Hz control loop
const float TS = TS_MS / 1000.0;

// ---------------- Recording (stored in the Arduino's memory) ----------------
const int NREC = 1200;                       // 1200 samples x 40 ms = 48 s
int16_t recY[NREC];                          // position x 100 (cm)
int16_t recU[NREC];                          // servo deviation x 100 (deg)
uint8_t recHigh[(NREC + 7) / 8];             // 1 bit per sample: setpoint high or low
int   nRec = 0;
bool  recording = false;
bool  unprinted = false;                     // a finished test waiting to be printed
int   recMode = 0;
float recCentre = 0;
float recGains[5];

// ---------------- State ----------------
Servo servo;
int   mode = 0;
bool  stepTest = false;
float Kp, Ki, Kd, b, Ff;
float integral = 0, yPrev = 0, dFilt = 0;
float buf[3] = {0, 0, 0};
unsigned long tLast = 0, tStepStart = 0;
char  cmd[16];
byte  cmdLen = 0;

// ---------------- Servo ----------------
void writeServo(float angleDeg) {
  servo.writeMicroseconds((int)(544 + angleDeg * (2400.0 - 544.0) / 180.0));
}

// ---------------- Sensor ----------------
float readOnce() {
  digitalWrite(TRIG_PIN, LOW);  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH); delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  unsigned long us = pulseIn(ECHO_PIN, HIGH, 30000UL);   // 30 ms timeout
  if (us == 0) return -1;
  return us * 0.0343 / 2.0;
}

float median3(float a, float b2, float c) {
  return max(min(a, b2), min(max(a, b2), c));
}

float readFiltered() {
  float r = readOnce();
  if (r < POS_MIN || r > POS_MAX) r = buf[2];
  buf[0] = buf[1]; buf[1] = buf[2]; buf[2] = r;
  return median3(buf[0], buf[1], buf[2]);
}

// ---------------- Recording helpers ----------------
bool  isHigh(int k)   { return (recHigh[k >> 3] >> (k & 7)) & 1; }
float recR(int k)     { return isHigh(k) ? recCentre + STEP_HALF : recCentre - STEP_HALF; }
float recYcm(int k)   { return recY[k] / 100.0; }
float smoothY(int k, int lo, int hi) {          // 5-sample average inside [lo, hi)
  int a = max(lo, k - 2), z = min(hi - 1, k + 2);
  float s = 0; for (int i = a; i <= z; i++) s += recYcm(i);
  return s / (z - a + 1);
}

void startRecording() {
  nRec = 0;
  recording = true;
  unprinted = true;
  recMode = mode;
  recCentre = centreCm;
  recGains[0] = Kp; recGains[1] = Ki; recGains[2] = Kd; recGains[3] = b; recGains[4] = Ff;
  memset(recHigh, 0, sizeof(recHigh));
  Serial.print(F("# RECORDING STARTED: ")); Serial.println(NAMES[mode]);
}

void saveSample(bool high, float y, float u) {
  if (!recording) return;
  if (nRec >= NREC) { recording = false; Serial.println(F("# recording full (48 s). Type m0 to print it.")); return; }
  recY[nRec] = (int16_t)(y * 100);
  recU[nRec] = (int16_t)(u * 100);
  if (high) recHigh[nRec >> 3] |= (1 << (nRec & 7));
  nRec++;
}

// Results for every set point step in the recording
void printSummary() {
  Serial.println(F("# ===== RESULTS ====="));
  Serial.println(F("# step, from_cm, to_cm, rise_s, overshoot_pct, settling_s, ss_error_cm, max_servo_deg"));
  int start = 0;
  float rPrev = recCentre;                     // the car starts at the centre
  int stepNo = 1;
  while (start < nRec) {
    int end = start + 1;
    while (end < nRec && isHigh(end) == isHigh(start)) end++;
    float r1 = recR(start), S = r1 - rPrev;
    int len = end - start;
    if (len >= 75 && fabs(S) > 1) {            // need at least 3 s of data
      float peak = 0, umax = 0; int i10 = -1, i90 = -1, lastOut = -1;
      for (int k = start; k < end; k++) {
        float n = (smoothY(k, start, end) - rPrev) / S;   // 0 = old set point, 1 = new
        if (n > peak) peak = n;
        if (i10 < 0 && n >= 0.1) i10 = k;
        if (i90 < 0 && n >= 0.9) i90 = k;
        if (fabs(smoothY(k, start, end) - r1) > 0.5) lastOut = k;
        float ua = fabs(recU[k] / 100.0); if (ua > umax) umax = ua;
      }
      int nTail = min(len, 50);                // last 2 s
      float ess = 0; for (int k = end - nTail; k < end; k++) ess += r1 - recYcm(k); ess /= nTail;
      Serial.print(F("# ")); Serial.print(stepNo); Serial.print(F(", "));
      Serial.print(rPrev, 2); Serial.print(F(", ")); Serial.print(r1, 2); Serial.print(F(", "));
      if (i10 >= 0 && i90 >= 0) Serial.print((i90 - i10) * TS, 2); else Serial.print(F("NaN"));
      Serial.print(F(", ")); Serial.print(max(0.0, (peak - 1) * 100), 1); Serial.print(F(", "));
      if (lastOut < 0) Serial.print(0.0, 2);
      else if (lastOut < end - 1) Serial.print((lastOut + 1 - start) * TS, 2);
      else Serial.print(F("not settled"));
      Serial.print(F(", ")); Serial.print(ess, 2); Serial.print(F(", ")); Serial.println(umax, 1);
    }
    rPrev = r1; start = end; stepNo++;
  }
}

void printRecording() {
  if (nRec == 0) { Serial.println(F("# nothing recorded yet")); return; }
  Serial.println();
  Serial.print(F("# ===== DATA: ")); Serial.print(NAMES[recMode]);
  Serial.print(F("  Kp ")); Serial.print(recGains[0]);
  Serial.print(F(" Ki "));  Serial.print(recGains[1]);
  Serial.print(F(" Kd "));  Serial.print(recGains[2]);
  Serial.print(F(" b "));   Serial.print(recGains[3]);
  Serial.print(F(" F "));   Serial.print(recGains[4]);
  Serial.print(F("  level ")); Serial.print(servoLevel, 1);
  Serial.print(F("  centre ")); Serial.print(recCentre, 2);
  Serial.print(F("  samples ")); Serial.println(nRec);
  Serial.println(F("t_s,setpoint_cm,position_cm,servo_deg"));
  for (int k = 0; k < nRec; k++) {
    Serial.print(k * TS, 2); Serial.print(',');
    Serial.print(recR(k), 2); Serial.print(',');
    Serial.print(recYcm(k), 2); Serial.print(',');
    Serial.println(recU[k] / 100.0, 2);
  }
  printSummary();
  Serial.println(F("# ===== END ====="));
  Serial.println(F("# Copy all of this into Notepad and save it. Then clear the output, recentre the car, next test."));
}

// ---------------- Modes and commands ----------------
void setMode(int m) {
  m = constrain(m, 0, 3);
  if (m == 0) {
    stepTest = false;
    if (unprinted) { recording = false; unprinted = false; mode = 0; writeServo(servoLevel); printRecording(); }
  }
  mode = m;
  Kp = GAINS[mode][0]; Ki = GAINS[mode][1]; Kd = GAINS[mode][2]; b = GAINS[mode][3]; Ff = GAINS[mode][4];
  integral = 0;
  tStepStart = millis();
}

void printSettings() {
  Serial.print(F("# mode ")); Serial.print(NAMES[mode]);
  Serial.print(F("  Kp ")); Serial.print(Kp);
  Serial.print(F("  Ki ")); Serial.print(Ki);
  Serial.print(F("  Kd ")); Serial.print(Kd);
  Serial.print(F("  b "));  Serial.print(b);
  Serial.print(F("  F "));  Serial.print(Ff);
  Serial.print(F("  level ")); Serial.print(servoLevel, 1);
  Serial.print(F("  centre ")); Serial.print(centreCm, 2);
  Serial.print(F("  step ")); Serial.println(stepTest ? F("ON") : F("OFF"));
}

void handleCommand(const char *c) {
  if (c[0] == '\0') return;
  if      (c[0] == 'm')   { setMode(atoi(c + 1)); }
  else if (c[0] == 's')   { stepTest = (c[1] != '0');
                            tStepStart = millis(); integral = 0;
                            if (stepTest && mode > 0) startRecording(); }
  else if (c[0] == 'p')   { printRecording(); return; }
  else if (c[0] == 'l')   servoLevel = constrain(atof(c + 1), 30.0, 150.0);
  else if (c[0] == 'c')   centreCm = atof(c + 1);
  else if (c[0] == 'P')   Kp = atof(c + 1);
  else if (c[0] == 'I')   { Ki = atof(c + 1); integral = 0; }
  else if (c[0] == 'D')   Kd = atof(c + 1);
  else if (c[0] == 'B')   b  = atof(c + 1);
  else if (c[0] == 'F')   Ff = atof(c + 1);
  else if (c[0] != '?')   { Serial.print(F("# unknown command: ")); Serial.println(c); return; }
  printSettings();
}

void readSerial() {
  while (Serial.available()) {
    char ch = Serial.read();
    if (ch == '\n' || ch == '\r') {
      cmd[cmdLen] = '\0';
      handleCommand(cmd);
      cmdLen = 0;
    } else if (cmdLen < sizeof(cmd) - 1) {
      cmd[cmdLen++] = ch;
    }
  }
}

// ---------------- Setup ----------------
void setup() {
  Serial.begin(115200);
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  servo.attach(SERVO_PIN);
  writeServo(servoLevel);
  setMode(0);
  delay(1000);
  for (int i = 0; i < 3; i++) { float r = readOnce(); buf[i] = (r > 0) ? r : centreCm; delay(30); }
  yPrev = readFiltered();
  tLast = millis();
  Serial.println(F("# ES 245 ball and beam ready. Car in the middle, then m1 and s1."));
  printSettings();
}

// ---------------- Control loop ----------------
void loop() {
  readSerial();
  if (millis() - tLast < TS_MS) return;
  tLast += TS_MS;

  // 1) Set point
  float r = centreCm;
  bool high = false;
  if (stepTest) {
    high = ((millis() - tStepStart) / STEP_PERIOD) % 2;
    r = high ? centreCm + STEP_HALF : centreCm - STEP_HALF;
  }

  // 2) Measurement
  float y = readFiltered();
  float e = r - y;

  // 3) Filtered derivative of the measurement (no kick on set point jumps)
  float dy = (y - yPrev) / TS;
  yPrev = y;
  dFilt = 0.6 * dFilt + 0.4 * dy;

  // 4) Integral, only near the set point, with a clamp (anti wind-up)
  if (Ki > 0) { if (fabs(e) < I_ZONE) integral = constrain(integral + Ki * e * TS, -15.0, 15.0); }
  else        integral = 0;

  // 5) Control law: u = Kp(b r - y) + I - Kd dy/dt + friction kick, measured from the centre
  float rc = r - centreCm, yc = y - centreCm;
  float u = 0;
  if (mode > 0) {
    u = Kp * (b * rc - yc) + integral - Kd * dFilt;
    if (e > 0.5)  u += Ff;
    if (e < -0.5) u -= Ff;
    u = constrain(u, -TH_MAX, TH_MAX);
  }

  // 6) Servo
  writeServo(servoLevel + dir * u);

  // 7) Save and show
  saveSample(high, y, u);
  Serial.print(F("setpoint:")); Serial.print(r, 2);
  Serial.print(F(",position:")); Serial.print(y, 2);
  Serial.print(F(",servo_deg:")); Serial.println(u, 2);
}
