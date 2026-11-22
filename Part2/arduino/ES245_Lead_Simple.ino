// ES245 Part 2, Task 8: LEAD compensator (simple version), Setup 5
// C(s) = 1.966 (0.791 s + 1)/(0.0791 s + 1), prefilter F(s) = 1/(0.791 s + 1)
// Discretised with Tustin for a 40 ms loop (numbers from MATLAB).
// Serial Monitor: 115200 baud, Newline.  Type S to start, X to stop.

#include <Servo.h>

const int TRIG = 9, ECHO = A2, SERVO_PIN = 11;
const float LEVEL  = 90.0;    // servo angle with the beam level
const float TARGET = 22.12;   // sensor reading with the car at the centre (cm)
const float KICK   = 3.0;     // small push to beat wheel friction (deg)
const unsigned long DT_MS = 40;

// Lead:      u[k]  = B0*e[k] + B1*e[k-1] + A1*u[k-1]
const float B0 = 16.0886, B1 = -15.2947, A1 = 0.5962;
// Prefilter: rf[k] = F0*(r[k] + r[k-1]) + F1*rf[k-1]
const float F0 = 0.02467, F1 = 0.95065;

Servo servo;
bool running = false;
float ePrev = 0, uPrev = 0, rf = 0, pos = 0;
unsigned long tStart = 0, tLast = 0;

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
  Serial.println("Type S to start, X to stop");
}

void loop() {
  // 1) Commands
  if (Serial.available()) {
    char c = toupper(Serial.read());
    if (c == 'S') {
      pos = readCm();
      rf = pos;                       // prefilter starts where the car is
      ePrev = 0; uPrev = 0;
      tStart = tLast = millis();
      running = true;
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

  // 4) Prefilter the target, then the lead compensator
  rf = F0 * (TARGET + TARGET) + F1 * rf;
  float e = rf - pos;
  float u = B0 * e + B1 * ePrev + A1 * uPrev;
  u = constrain(u, -30, 30);
  ePrev = e; uPrev = u;

  // 5) Friction kick, then drive the servo
  float err = TARGET - pos;
  float out = u;
  if (err >  0.5) out += KICK;
  if (err < -0.5) out -= KICK;
  setAngle(out);

  // 6) Log
  Serial.print((now - tStart) / 1000.0, 2); Serial.print(',');
  Serial.print(TARGET, 2);  Serial.print(',');
  Serial.print(rf, 2);      Serial.print(',');
  Serial.print(pos, 2);     Serial.print(',');
  Serial.println(out, 2);
}
