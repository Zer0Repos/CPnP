#include <ESP32Servo.h>

#define X_JOY 34
#define Y_JOY 35
#define Z_POT 4
#define SW    32
#define SERVO 14

#define CENTER 2048
#define DEAD   200
#define POT_LO 100
#define POT_HI 3995
#define STEP_US 10

struct Motor {
  int step, dir;
  long *pos;
};

long x = 0, y = 0, z = 0;

Motor mx = {33, 25, &x};
Motor my = {17, 26, &y};
Motor mz = {16, 27, &z};

Servo claw;
bool closed = false;
bool lastSW = HIGH;

void step(Motor &m, bool dir)
{
  digitalWrite(m.dir, dir);
  digitalWrite(m.step, HIGH);
  delayMicroseconds(STEP_US);
  digitalWrite(m.step, LOW);
  delayMicroseconds(STEP_US);

  *m.pos += dir ? 1 : -1;
}

void setup()
{
  Serial.begin(115200);

  pinMode(SW, INPUT_PULLUP);

  for (Motor *m : {&mx, &my, &mz}) {
    pinMode(m->step, OUTPUT);
    pinMode(m->dir, OUTPUT);
  }

  claw.setPeriodHertz(50);
  claw.attach(SERVO, 500, 2400);
  claw.write(30);
}

void loop()
{
  int jx = analogRead(X_JOY);
  int jy = analogRead(Y_JOY);
  int pot = analogRead(Z_POT);

  if (jx > CENTER + DEAD)      step(mx, true);
  else if (jx < CENTER - DEAD) step(mx, false);

  if (jy > CENTER + DEAD)      step(my, true);
  else if (jy < CENTER - DEAD) step(my, false);

  if (pot < POT_LO)            step(mz, true);
  else if (pot > POT_HI)       step(mz, false);

  bool sw = digitalRead(SW);

  if (lastSW && !sw) {
    closed = !closed;
    claw.write(closed ? 100 : 30);
  }

  lastSW = sw;

  Serial.printf("X:%ld Y:%ld Z:%ld Claw:%d\n",
                x, y, z, closed);
}
