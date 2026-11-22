// ES245 Part 2, Task 8: LEAD, LAG and LEAD-LAG compensators (simple), Setup 5
// Plant (MATLAB tfest): G(s) = 2.591/(s^2 + 0.4789 s + 0.002551)  [cm/deg]
//
//   LEAD     : C(s) = 1.966 (0.791 s + 1)/(0.0791 s + 1)          + prefilter 1/(0.791 s + 1)
//   LAG      : C(s) = 0.0774 (s + 0.0343)/(s + 0.00265)           (no prefilter)
//   LEAD-LAG : C(s) = LEAD x (s + 0.2)/(s + 0.02)                  + prefilter 1/(0.791 s + 1)
//
// All discretised with Tustin for a 40 ms loop.
// Serial Monitor: 115200 baud, Newline.
//   Type 1 = LEAD, 2 = LAG, 3 = LEAD-LAG, then S to start, X to stop.

#include <Servo.h>

const int TRIG = 9, ECHO = A2, SERVO_PIN = 11;
const float LEVEL  = 90.0;    // servo angle with the beam level
const float TARGET = 22.12;   // sensor reading with the car at the centre (cm)
const float KICK   = 3.0;     // small push to beat wheel friction (deg)
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
Stage lag2 = {1.003599, -0.995602, 0.999200, 0, 0};   // lag stage used after the lead

// Prefilter: rf[k] = F0*(r[k] + r[k-1]) + F1*rf[k-1]
const float F0 = 0.02467, F1 = 0.95065;

Servo servo;
int mode = 1;                 // 1 = LEAD, 2 = LAG, 3 = LEAD-LAG
bool running = false;
float rf = 0, pos = 0;
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

void setup() {
  Serial.begin(115200);
  pinMode(TRIG, OUTPUT);
  pinMode(ECHO, INPUT);
  servo.attach(SERVO_PIN);
  setAngle(0);
  delay(500);
  pos = readCm();
  Serial.println("Type 1 = LEAD, 2 = LAG, 3 = LEAD-LAG, then S to start, X to stop");
}

void loop() {
  // 1) Commands
  if (Serial.available()) {
    char c = toupper(Serial.read());
    if (c == '1' || c == '2' || c == '3') {
      running = false; setAngle(0);
      mode = c - '0';
      Serial.print("Mode: "); Serial.println(modeName());
    }
    if (c == 'S') {
      pos = readCm();
      rf = (mode == 2) ? TARGET : pos;   // lag has no prefilter
      lead.reset(); lag.reset(); lag2.reset();
      tStart = tLast = millis();
      running = true;
      Serial.print("START "); Serial.println(modeName());
      Serial.println("time_s,target_cm,filtered_target_cm,position_cm,servo_deg");
    }
    if (c == 'X') { running = false; setAngle(0); Serial.println("STOPPED"); }
  }
  if (!running) return;

  // 2) Fixed 40 ms loop
  unsigned long now = millis();
  if (now - tLast < DT_MS) return;
  tLast = now;

  // 3) Measure (light smoothing of the sensor)
  pos = 0.7 * readCm() + 0.3 * pos;

  // 4) Prefilter the target (LEAD and LEAD-LAG only)
  if (mode != 2) rf = F0 * (TARGET + TARGET) + F1 * rf;
  float e = rf - pos;

  // 5) Compensator
  float u;
  if (mode == 1)      u = lead.run(e);
  else if (mode == 2) u = lag.run(e);
  else                u = lag2.run(lead.run(e));   // lead first, then lag

  // 6) Friction kick (LEAD and LEAD-LAG only), then drive the servo
  //    The lag alone has a tiny gain (0.08 deg/cm), so a 3 deg kick would
  //    swamp it and make the car hunt. Lag runs without the kick.
  float err = TARGET - pos;
  float out = u;
  if (mode != 2) {
    if (err >  0.5) out += KICK;
    if (err < -0.5) out -= KICK;
  }
  setAngle(out);

  // 7) Log
  Serial.print((now - tStart) / 1000.0, 2); Serial.print(',');
  Serial.print(TARGET, 2);  Serial.print(',');
  Serial.print(rf, 2);      Serial.print(',');
  Serial.print(pos, 2);     Serial.print(',');
  Serial.println(out, 2);
}
