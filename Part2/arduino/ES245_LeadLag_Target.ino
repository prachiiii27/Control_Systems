// ES245 Part 2, Task 8: LEAD, LAG and LEAD-LAG with a settable target, Setup 5
// Plant (MATLAB tfest): G(s) = 2.591/(s^2 + 0.4789 s + 0.002551)  [cm/deg]
//
//   LEAD     : C(s) = 1.966 (0.791 s + 1)/(0.0791 s + 1)     + prefilter 1/(0.791 s + 1)
//   LAG      : C(s) = 0.0774 (s + 0.0343)/(s + 0.00265)      (no prefilter)
//   LEAD-LAG : C(s) = LEAD x (s + 0.2)/(s + 0.02)             + prefilter 1/(0.791 s + 1)
// All discretised with Tustin for a 40 ms loop.
//
// Serial Monitor: 115200 baud, line ending "Newline". Commands:
//   1 / 2 / 3   choose LEAD / LAG / LEAD-LAG
//   T 18        set the target to 18 cm (works before START and while running)
//   S           start
//   X           stop

#include <Servo.h>

const int TRIG = 9, ECHO = A2, SERVO_PIN = 11;
const float LEVEL  = 90.0;      // servo angle with the beam level
const float CENTRE = 22.12;     // sensor reading with the car at the centre (cm)
const float T_MIN = 8.0, T_MAX = 36.0;   // allowed target range on the track (cm)
const float KICK   = 3.0;       // small push to beat wheel friction (deg)
const unsigned long DT_MS = 40;

// One first order section: y[k] = b0*x[k] + b1*x[k-1] + a1*y[k-1]
struct Stage {
  float b0, b1, a1, xPrev, yPrev;
  float run(float x) {
    float y = b0 * x + b1 * xPrev + a1 * yPrev;
    y = constrain(y, -30, 30);          // keeps the memory inside the servo range
    xPrev = x; yPrev = y;
    return y;
  }
  void reset() { xPrev = 0; yPrev = 0; }
};

Stage lead = {16.0876, -15.2938, 0.596204, 0, 0};
Stage lag  = {0.077416, -0.077310, 0.999894, 0, 0};   // lag on its own
Stage lag2 = {1.003599, -0.995602, 0.999200, 0, 0};   // lag stage after the lead

// Prefilter: rf[k] = F0*(r[k] + r[k-1]) + F1*rf[k-1]
const float F0 = 0.02467, F1 = 0.95065;

Servo servo;
int mode = 1;                   // 1 = LEAD, 2 = LAG, 3 = LEAD-LAG
bool running = false;
float target = CENTRE;          // r[k], changed with the T command
float rPrev = CENTRE;           // r[k-1]
float rf = CENTRE, pos = 0;
unsigned long tStart = 0, tLast = 0;

const char* modeName() { return mode == 1 ? "LEAD" : (mode == 2 ? "LAG" : "LEAD-LAG"); }

float readCm() {
  digitalWrite(TRIG, LOW);  delayMicroseconds(2);
  digitalWrite(TRIG, HIGH); delayMicroseconds(10);
  digitalWrite(TRIG, LOW);
  unsigned long us = pulseIn(ECHO, HIGH, 30000UL);
  float d = us * 0.0343 / 2.0;
  if (us == 0 || d < 2 || d > 40) return pos;   // bad echo: keep last value
  return d;
}

void setAngle(float offset) {
  offset = constrain(offset, -30, 30);
  servo.write(LEVEL + offset);
}

void handleCommand(String cmd) {
  cmd.trim();
  cmd.toUpperCase();
  if (cmd.length() == 0) return;

  if (cmd == "1" || cmd == "2" || cmd == "3") {
    running = false; setAngle(0);
    mode = cmd.toInt();
    Serial.print("Mode: "); Serial.println(modeName());
  }
  else if (cmd.startsWith("T")) {
    float t = cmd.substring(1).toFloat();
    if (t < T_MIN || t > T_MAX) {
      Serial.print("Target must be between "); Serial.print(T_MIN); Serial.print(" and "); Serial.println(T_MAX);
      return;
    }
    target = t;
    Serial.print("# Target = "); Serial.print(target, 2); Serial.println(" cm");
  }
  else if (cmd == "S") {
    pos = readCm();
    rPrev = target;
    rf = (mode == 2) ? target : pos;   // prefilter starts where the car is (lag has none)
    lead.reset(); lag.reset(); lag2.reset();
    tStart = tLast = millis();
    running = true;
    Serial.print("START "); Serial.print(modeName());
    Serial.print(", target "); Serial.print(target, 2); Serial.println(" cm");
    Serial.println("time_s,target_cm,filtered_target_cm,position_cm,servo_deg");
  }
  else if (cmd == "X") {
    running = false; setAngle(0);
    Serial.println("STOPPED");
  }
  else {
    Serial.println("Commands: 1, 2, 3, T <cm>, S, X");
  }
}

void setup() {
  Serial.begin(115200);
  pinMode(TRIG, OUTPUT);
  pinMode(ECHO, INPUT);
  servo.attach(SERVO_PIN);
  setAngle(0);
  delay(500);
  pos = readCm();
  Serial.println("Commands: 1 = LEAD, 2 = LAG, 3 = LEAD-LAG, T <cm> = target, S = start, X = stop");
  Serial.print("Target now "); Serial.print(target, 2); Serial.println(" cm");
}

void loop() {
  // 1) Commands (one per line)
  if (Serial.available()) handleCommand(Serial.readStringUntil('\n'));
  if (!running) return;

  // 2) Fixed 40 ms loop
  unsigned long now = millis();
  if (now - tLast < DT_MS) return;
  tLast = now;

  // 3) Measure (light smoothing of the sensor)
  pos = 0.7 * readCm() + 0.3 * pos;

  // 4) Prefilter the target (LEAD and LEAD-LAG). A new target glides in smoothly.
  if (mode == 2) rf = target;
  else           rf = F0 * (target + rPrev) + F1 * rf;
  rPrev = target;
  float e = rf - pos;

  // 5) Compensator
  float u;
  if (mode == 1)      u = lead.run(e);
  else if (mode == 2) u = lag.run(e);
  else                u = lag2.run(lead.run(e));   // lead first, then lag

  // 6) Friction kick (not for lag: its gain is too small, the kick would swamp it)
  float err = target - pos;
  float out = u;
  if (mode != 2) {
    if (err >  0.5) out += KICK;
    if (err < -0.5) out -= KICK;
  }
  setAngle(out);

  // 7) Log
  Serial.print((now - tStart) / 1000.0, 2); Serial.print(',');
  Serial.print(target, 2);  Serial.print(',');
  Serial.print(rf, 2);      Serial.print(',');
  Serial.print(pos, 2);     Serial.print(',');
  Serial.println(out, 2);
}
