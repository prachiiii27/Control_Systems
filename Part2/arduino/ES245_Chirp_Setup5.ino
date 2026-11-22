// ES 245 Part 2, Task 5: chirp test on Setup 5
// Based on the TA's Chirp_Signal_Ultrasonic.ino. Changes are marked CHANGED.

#include <Servo.h>
#include <math.h>

// Pin locations
const int trigPin  = 9;
const int echoPin  = A2;    // CHANGED: our ECHO wire is on A2 (TA file had A1)
const int servoPin = 11;    // CHANGED: our servo is on pin 11 (TA file had 3)

Servo servo;
long duration;

// Chirp signal parameters
const float f0 = 0.1;          // start frequency (Hz)
const float f1 = 5.0;          // end frequency (Hz)
const float T_chirp = 20.0;    // chirp duration (s)

const float amplitude   = 20;  // CHANGED: TA default 45 deg. Start with 20, see notes
const float centerAngle = 90;  // our level angle

const float servoMin = 40.0;
const float servoMax = 180.0;

const unsigned long samplePeriod = 20;   // 50 Hz target
const unsigned long waitMs = 3000;       // CHANGED: 3 s at level to place the car

unsigned long startMillis;
unsigned long lastMillis;
bool finished = false;

float getDistance() {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  duration = pulseIn(echoPin, HIGH, 30000UL);
  if (duration == 0) return -1.0;          // no echo

  float d = duration * 0.0343 / 2.0;
  if (d < 2.0 || d > 50.0) return -1.0;    // out of range
  return d;
}

void setup() {
  Serial.begin(115200);       // CHANGED: 9600 is too slow for 50 lines per second
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  servo.attach(servoPin);
  servo.write(centerAngle);

  delay(waitMs);              // beam level, put the car at the centre now
  Serial.println("time_ms,servo_deg,dist_cm");
  startMillis = millis();
  lastMillis  = startMillis;
}

void loop() {
  if (finished) return;

  unsigned long now = millis();
  if (now - lastMillis < samplePeriod) return;
  lastMillis = now;

  float t = (now - startMillis) / 1000.0;
  float k = (f1 - f0) / T_chirp;

  if (t >= T_chirp) {
    servo.write(centerAngle);
    Serial.println("END");    // CHANGED: tells the MATLAB logger to stop
    finished = true;
    return;
  }

  float phi = 2.0 * PI * (f0 * t + 0.5 * k * t * t);
  float servoCmd = centerAngle + amplitude * sin(phi);
  servoCmd = constrain(servoCmd, servoMin, servoMax);
  servo.write(servoCmd);

  float dist = getDistance();

  Serial.print(now - startMillis);
  Serial.print(",");
  Serial.print(servoCmd, 2);
  Serial.print(",");
  Serial.println(dist, 2);
}
