#include <Servo.h>
#include <math.h>
#include <ctype.h>
#include <string.h>
#include <stdlib.h>

// =====================================================
// ES245 PART 1, TASK 4: CAR-ON-BEAM P / PD / PID (Setup 5)
// Ultrasonic HC-SR04 sensor, MG996R servo, Arduino Mega
// =====================================================
//
// HOW TO RUN (Serial Monitor, 115200 baud, "Newline")
//   1. Put the car near the sensor end (about 10 to 12 cm reading).
//   2. Type  P   or  PD   or  PID   (selects the controller)
//   3. Type  START
//   4. The car is driven to the centre. When it stays within +/-2 cm
//      of the centre and almost still for 3 s, the Arduino prints STABLE,
//      levels the beam and ends the trial. If it never settles, the trial
//      ends after 40 s with TIMEOUT (this is what P will do).
//   5. A SUMMARY line is printed at the end of every trial.
//   Type STOP at any time to stop.
//
// Other commands (optional):
//   LEVEL 92.5   servo angle that makes the beam level
//   CENTRE 21.88 sensor reading with the car over the pivot (cm)
//   KP 4  KD 2.1  KI 0.8  KICK 7   change the gains of the selected controller
//   SHOW         print the current settings

// ---------------- Hardware ----------------
const byte TRIG_PIN  = 9;
const byte ECHO_PIN  = A2;
const byte SERVO_PIN = 11;
Servo beamServo;

// ---------------- Calibration (measured in the lab) ----------------
float servoLevel = 90.0;      // servo angle (deg) at which the beam is level
const int DIR    = +1;        // +1: bigger servo angle sends the car AWAY from the sensor
float centreCm   = 22.12;     // centre reading: midpoint of the measured 21.88 to 22.35 cm

// Servo limits: level +/- 30 deg
const float MAX_OFFSET = 30.0;

// Usable sensor range (cm). Anything outside is a bad echo and is ignored.
const float POS_MIN = 2.0;
const float POS_MAX = 40.0;

// ---------------- Stability requirement ----------------
const float STABLE_BAND_CM   = 2.0;     // within +/-2 cm of the centre
const float STABLE_SPEED_CMS = 2.0;     // and moving slower than 2 cm/s (smoothed speed)
const unsigned long STABLE_HOLD_MS = 3000;   // for 3 s continuously
const unsigned long MAX_RUN_MS     = 40000;  // give up after 40 s (TIMEOUT)

// ---------------- Sampling and filtering ----------------
const unsigned long SAMPLE_MS = 40;     // 25 Hz
const float POSITION_FILTER = 0.5;      // 0..1, higher = less smoothing
const float VELOCITY_FILTER = 0.35;

// ---------------- Controller gains (servo deg per cm, per cm/s, per cm*s) --------
//                     Kp     Kd     Ki    kick
float GAIN_P[4]   = {2.5,   0.0,   0.0,  7.0};
float GAIN_PD[4]  = {4.0,   2.1,   0.0,  7.0};
float GAIN_PID[4] = {4.0,   2.1,   0.8,  7.0};

// ---------------- Friction handling ----------------
const float STALL_ERROR_CM  = 1.0;      // car is away from the centre...
const float STALL_SPEED_CMS = 1.0;      // ...and not moving -> apply at least "kick" degrees
const float HOLD_ERROR_CM   = 0.7;      // close enough and still -> hold the beam steady
const float HOLD_SPEED_CMS  = 1.0;
const float I_ZONE_CM       = 6.0;      // integrate only within 6 cm of the centre

// ---------------- End recovery ----------------
const float NEAR_END_CM   = 6.0;        // car almost at the sensor end
const float FAR_END_CM    = 34.0;       // car almost at the far end
const float RECOVERY_DEG  = 15.0;

// ---------------- Modes and state ----------------
enum ControllerMode { P_MODE, PD_MODE, PID_MODE };
ControllerMode controller = PD_MODE;

bool running = false;
bool initialized = false;

float buf[3] = {0, 0, 0};
float positionCm = 0, previousPositionCm = 0, velocityCms = 0, integralError = 0;
float slowSpeedCms = 0;           // heavily smoothed speed, only for the stability check
float servoOffset = 0;

unsigned long runStartMs = 0, lastSampleMs = 0, stableSinceMs = 0;
unsigned long trialNumber = 0;

// trial statistics for the summary line
float startCm = 0, peakOvershootCm = 0, maxOffsetUsed = 0;
float settleTimeS = -1;

char inputBuffer[24];
byte inputLength = 0;

// =====================================================
// HELPERS
// =====================================================

float* gains() {
  if (controller == P_MODE) return GAIN_P;
  if (controller == PD_MODE) return GAIN_PD;
  return GAIN_PID;
}

const char* controllerName() {
  if (controller == P_MODE) return "P";
  if (controller == PD_MODE) return "PD";
  return "PID";
}

float readOnceCm() {
  digitalWrite(TRIG_PIN, LOW);  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH); delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  unsigned long us = pulseIn(ECHO_PIN, HIGH, 30000UL);
  if (us == 0) return -1;
  return us * 0.0343 / 2.0;
}

float median3(float a, float b, float c) {
  return max(min(a, b), min(max(a, b), c));
}

// Median of the last three good readings (bad echoes are replaced by the last good one)
float readSensorCm() {
  float r = readOnceCm();
  if (r < POS_MIN || r > POS_MAX) r = buf[2];
  buf[0] = buf[1]; buf[1] = buf[2]; buf[2] = r;
  return median3(buf[0], buf[1], buf[2]);
}

void setServoOffset(float offsetDeg) {
  servoOffset = constrain(offsetDeg, -MAX_OFFSET, MAX_OFFSET);
  float angle = servoLevel + DIR * servoOffset;
  beamServo.writeMicroseconds((int)(544 + angle * (2400.0 - 544.0) / 180.0));
}

void logCSV(unsigned long now, float rawCm, const char* status) {
  Serial.print(trialNumber);                       Serial.print(',');
  Serial.print((now - runStartMs) / 1000.0, 3);    Serial.print(',');
  Serial.print(controllerName());                  Serial.print(',');
  Serial.print(centreCm, 2);                       Serial.print(',');
  Serial.print(rawCm, 2);                          Serial.print(',');
  Serial.print(positionCm, 2);                     Serial.print(',');
  Serial.print(centreCm - positionCm, 2);          Serial.print(',');
  Serial.print(velocityCms, 2);                    Serial.print(',');
  Serial.print(servoOffset, 2);                    Serial.print(',');
  Serial.print(servoOffset * 0.2, 2);              Serial.print(',');   // beam angle = 0.2 x servo angle (linkage d/L)
  Serial.println(status);
}

void printSummary(const char* result, unsigned long now) {
  float startErr = fabs(centreCm - startCm);
  float osPct = (startErr > 0.5) ? 100.0 * peakOvershootCm / startErr : 0;
  Serial.print("# SUMMARY trial ");      Serial.print(trialNumber);
  Serial.print(", controller=");         Serial.print(controllerName());
  Serial.print(", result=");             Serial.print(result);
  Serial.print(", start_cm=");           Serial.print(startCm, 2);
  Serial.print(", overshoot_cm=");       Serial.print(peakOvershootCm, 2);
  Serial.print(", overshoot_pct=");      Serial.print(osPct, 1);
  Serial.print(", settling_s=");
  if (settleTimeS >= 0) Serial.print(settleTimeS, 2); else Serial.print("none");
  Serial.print(", final_error_cm=");     Serial.print(centreCm - positionCm, 2);
  Serial.print(", max_servo_deg=");      Serial.print(maxOffsetUsed, 1);
  Serial.print(", run_time_s=");         Serial.println((now - runStartMs) / 1000.0, 2);
}

void stopController(const char* reason) {
  if (!running) return;
  running = false;
  integralError = 0;
  setServoOffset(0);
  Serial.print("# STOP: "); Serial.println(reason);
  printSummary(reason, millis());
}

void startController() {
  if (running) return;
  integralError = 0;
  velocityCms = 0;
  initialized = false;
  stableSinceMs = 0;
  peakOvershootCm = 0;
  maxOffsetUsed = 0;
  settleTimeS = -1;
  setServoOffset(0);
  runStartMs = millis();
  lastSampleMs = runStartMs;
  trialNumber++;
  running = true;
  Serial.print("# START trial "); Serial.print(trialNumber);
  Serial.print(", controller=");  Serial.print(controllerName());
  float* g = gains();
  Serial.print(", Kp=");   Serial.print(g[0]);
  Serial.print(", Kd=");   Serial.print(g[1]);
  Serial.print(", Ki=");   Serial.print(g[2]);
  Serial.print(", kick="); Serial.println(g[3]);
  Serial.println("trial,time_s,controller,setpoint_cm,raw_cm,position_cm,error_cm,velocity_cms,servo_offset_deg,beam_angle_deg,status");
}

void showSettings() {
  float* g = gains();
  Serial.print("# controller="); Serial.print(controllerName());
  Serial.print(" Kp="); Serial.print(g[0]);
  Serial.print(" Kd="); Serial.print(g[1]);
  Serial.print(" Ki="); Serial.print(g[2]);
  Serial.print(" kick="); Serial.print(g[3]);
  Serial.print("  level="); Serial.print(servoLevel, 1);
  Serial.print("  centre="); Serial.print(centreCm, 2);
  Serial.print("  sensor now="); Serial.println(readSensorCm(), 2);
}

void processCommand(char* cmd) {
  for (byte i = 0; cmd[i] != '\0'; i++) cmd[i] = toupper((unsigned char)cmd[i]);
  char* arg = strchr(cmd, ' ');
  float val = 0;
  if (arg) { *arg = '\0'; val = atof(arg + 1); }

  if      (strcmp(cmd, "START") == 0) startController();
  else if (strcmp(cmd, "STOP") == 0)  stopController("MANUAL_STOP");
  else if (strcmp(cmd, "P") == 0)     { stopController("SWITCH"); controller = P_MODE;   showSettings(); }
  else if (strcmp(cmd, "PD") == 0)    { stopController("SWITCH"); controller = PD_MODE;  showSettings(); }
  else if (strcmp(cmd, "PID") == 0)   { stopController("SWITCH"); controller = PID_MODE; showSettings(); }
  else if (strcmp(cmd, "LEVEL") == 0 && arg)  { servoLevel = val; if (!running) setServoOffset(0); showSettings(); }
  else if (strcmp(cmd, "CENTRE") == 0 && arg) { centreCm = val; showSettings(); }
  else if (strcmp(cmd, "KP") == 0 && arg)     { gains()[0] = val; showSettings(); }
  else if (strcmp(cmd, "KD") == 0 && arg)     { gains()[1] = val; showSettings(); }
  else if (strcmp(cmd, "KI") == 0 && arg)     { gains()[2] = val; showSettings(); }
  else if (strcmp(cmd, "KICK") == 0 && arg)   { gains()[3] = val; showSettings(); }
  else if (strcmp(cmd, "SHOW") == 0)          showSettings();
  else Serial.println("# Commands: P, PD, PID, START, STOP, SHOW, LEVEL x, CENTRE x, KP x, KD x, KI x, KICK x");
}

void readSerialCommands() {
  while (Serial.available() > 0) {
    char c = Serial.read();
    if (c == '\r') continue;
    if (c == '\n') {
      inputBuffer[inputLength] = '\0';
      if (inputLength > 0) processCommand(inputBuffer);
      inputLength = 0;
    } else if (inputLength < sizeof(inputBuffer) - 1) {
      inputBuffer[inputLength++] = c;
    } else {
      inputLength = 0;
    }
  }
}

// =====================================================
// SETUP
// =====================================================

void setup() {
  Serial.begin(115200);
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  beamServo.attach(SERVO_PIN);
  setServoOffset(0);
  delay(800);
  for (byte i = 0; i < 3; i++) { float r = readOnceCm(); buf[i] = (r > 0) ? r : centreCm; delay(30); }

  Serial.println("# ES245 Task 4: car-on-beam stability test (ultrasonic, Setup 5)");
  Serial.println("# Target: centre of the beam. Stable = within +/-2 cm and slower than 2 cm/s for 3 s.");
  Serial.println("# Put the car near the sensor end, type P, PD or PID, then START.");
  showSettings();
}

// =====================================================
// MAIN LOOP
// =====================================================

void loop() {
  readSerialCommands();
  if (!running) return;

  unsigned long now = millis();
  if (now - lastSampleMs < SAMPLE_MS) return;
  float dt = (now - lastSampleMs) / 1000.0;
  lastSampleMs = now;

  // 1) Measure and filter
  float rawCm = readSensorCm();
  if (!initialized) {
    positionCm = previousPositionCm = rawCm;
    velocityCms = 0;
    slowSpeedCms = 0;
    startCm = rawCm;
    initialized = true;
  } else {
    positionCm += POSITION_FILTER * (rawCm - positionCm);
    float measuredVelocity = (positionCm - previousPositionCm) / dt;
    velocityCms += VELOCITY_FILTER * (measuredVelocity - velocityCms);
    slowSpeedCms += 0.1 * (measuredVelocity - slowSpeedCms);
    previousPositionCm = positionCm;
  }

  float error = centreCm - positionCm;   // + means the car must move away from the sensor

  // 2) Statistics: overshoot = how far it went past the centre on the other side
  float startSide = (startCm < centreCm) ? 1.0 : -1.0;
  float past = -startSide * error;       // > 0 when the car is beyond the centre
  if (past > peakOvershootCm) peakOvershootCm = past;

  // 3) Stability check: inside the band AND nearly still, continuously for 3 s
  bool inStableBand = fabs(error) <= STABLE_BAND_CM && fabs(slowSpeedCms) < STABLE_SPEED_CMS;
  if (inStableBand) {
    if (stableSinceMs == 0) stableSinceMs = now;
    if (now - stableSinceMs >= STABLE_HOLD_MS) {
      settleTimeS = (stableSinceMs - runStartMs) / 1000.0;
      setServoOffset(0);
      logCSV(now, rawCm, "STABLE");
      running = false;
      integralError = 0;
      Serial.print("# STABLE: position="); Serial.print(positionCm, 2);
      Serial.print(" cm, error="); Serial.print(error, 2);
      Serial.println(" cm. Beam levelled.");
      printSummary("STABLE", now);
      return;
    }
  } else {
    stableSinceMs = 0;
  }

  if (now - runStartMs > MAX_RUN_MS) {
    logCSV(now, rawCm, "TIMEOUT");
    stopController("TIMEOUT");
    return;
  }

  // 4) Control law: u = Kp e - Kd v + Ki integral(e)  (derivative on the measurement)
  float* g = gains();
  float kp = g[0], kd = g[1], ki = g[2], kick = g[3];

  // Integral only near the centre, and only if it would not push a saturated output further (anti-windup)
  float candidateIntegral = integralError;
  if (ki > 0 && fabs(error) < I_ZONE_CM) candidateIntegral = constrain(integralError + error * dt, -40.0, 40.0);
  float candidateOutput = kp * error - kd * velocityCms + ki * candidateIntegral;
  if (fabs(candidateOutput) <= MAX_OFFSET ||
      (candidateOutput > MAX_OFFSET && error < 0) ||
      (candidateOutput < -MAX_OFFSET && error > 0)) {
    integralError = candidateIntegral;
  }

  float output = kp * error - kd * velocityCms + ki * integralError;
  output = constrain(output, -MAX_OFFSET, MAX_OFFSET);

  // 5) Friction: if the car is stuck away from the centre, tilt at least "kick" degrees toward it
  if (fabs(error) > STALL_ERROR_CM && fabs(velocityCms) < STALL_SPEED_CMS) {
    if (error > 0 && output <  kick) output =  kick;
    if (error < 0 && output > -kick) output = -kick;
  }

  // 6) Close to the centre and nearly still: hold the beam (only the integral's trim stays)
  if (fabs(error) <= HOLD_ERROR_CM && fabs(velocityCms) < HOLD_SPEED_CMS) {
    output = ki * integralError;
  }

  // 7) End recovery: if the car reaches an end, tilt firmly back inward
  if (rawCm <= NEAR_END_CM)      { output =  RECOVERY_DEG; integralError = 0; }
  else if (rawCm >= FAR_END_CM)  { output = -RECOVERY_DEG; integralError = 0; }

  setServoOffset(output);
  if (fabs(servoOffset) > maxOffsetUsed) maxOffsetUsed = fabs(servoOffset);
  logCSV(now, rawCm, "RUNNING");
}
