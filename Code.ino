#include <Servo.h>

constexpr uint8_t S0  = 30, S1  = 32, S2  = 34, S3  = 36, OUT = 38;
constexpr uint8_t relayPin = 42, DETECTION_COUNT = 5;
constexpr uint8_t pinPinza = 22, pinCintura = 24, pinCodo = 26, pinHombro = 28;
constexpr uint8_t joy1X = A1, joy1Y = A2, joy2X = A3, joy2Y = A4;
constexpr int deadZone = 40;
constexpr uint8_t maxStep = 2, stepDelay = 20;
constexpr uint8_t MIN_ANG = 0, MAX_ANG = 180;

Servo servoPinza, servoCintura, servoCodo, servoHombro;
uint8_t angPinza = 90, angCintura = 90, angCodo = 90, angHombro = 90;
uint8_t redCount = 0, blueCount = 0, greenCount = 0;
uint8_t lastAngPinza = 90, lastAngCintura = 90, lastAngCodo = 90, lastAngHombro = 90;

unsigned long readColor(uint8_t s2, uint8_t s3, uint8_t samples = 4) {
  digitalWrite(S2, s2);
  digitalWrite(S3, s3);
  unsigned long total = 0;
  for (uint8_t i = 0; i < samples; i++) {
    total += pulseIn(OUT, LOW, 12000);
  }
  return (samples > 0) ? total / samples : 0;
}

uint8_t smoothMove(uint8_t actual, uint8_t target, uint8_t step) {
  if (target > actual) return (actual + step > target) ? target : actual + step;
  if (target < actual) return (actual < step || actual - step < target) ? target : actual - step;
  return actual;
}

inline int applyDeadZone(int val) {
  return (abs(val - 512) < deadZone) ? 512 : val;
}

inline uint8_t clampAngle(int angle) {
  return static_cast<uint8_t>(constrain(angle, MIN_ANG, MAX_ANG));
}

void setup() {
  Serial.begin(115200);
  pinMode(S0, OUTPUT); pinMode(S1, OUTPUT);
  pinMode(S2, OUTPUT); pinMode(S3, OUTPUT);
  pinMode(OUT, INPUT);  
  pinMode(relayPin, OUTPUT);
  digitalWrite(S0, HIGH); digitalWrite(S1, HIGH);
  digitalWrite(relayPin, LOW);
  servoPinza.attach(pinPinza);
  servoCintura.attach(pinCintura);
  servoCodo.attach(pinCodo);
  servoHombro.attach(pinHombro);
  servoPinza.write(angPinza);
  servoCintura.write(angCintura);
  servoCodo.write(angCodo);
  servoHombro.write(angHombro);
  Serial.println(F("Brazo robótico listo (joysticks reasignados)"));
}

void loop() {
  unsigned long r = readColor(LOW, LOW);
  unsigned long b = readColor(LOW, HIGH);
  unsigned long g = readColor(HIGH, HIGH);

  Serial.print(F("R:")); Serial.print(r);
  Serial.print(F(" G:")); Serial.print(g);
  Serial.print(F(" B:")); Serial.println(b);

  redCount   = (r < 1) ? redCount + 1 : 0;
  blueCount  = (b < 1) ? blueCount + 1 : 0;
  greenCount = (g < 1) ? greenCount + 1 : 0;

  if (redCount >= DETECTION_COUNT || blueCount >= DETECTION_COUNT || greenCount >= DETECTION_COUNT) {
    digitalWrite(relayPin, HIGH);
  } else {
    digitalWrite(relayPin, LOW);
  }

  // --- NUEVA ASIGNACION DE JOYSTICKS ---
  int v1x = applyDeadZone(analogRead(joy1X)); // A1 → cintura
  int v1y = applyDeadZone(analogRead(joy1Y)); // A2 → pinza
  int v2x = applyDeadZone(analogRead(joy2X)); // A3 → codo
  int v2y = applyDeadZone(analogRead(joy2Y)); // A4 → hombro

  uint8_t tgtCint = clampAngle(map(v1x, 0, 1023, MIN_ANG, MAX_ANG));
  uint8_t tgtPinz = clampAngle(map(v1y, 0, 1023, MIN_ANG, MAX_ANG));
  uint8_t tgtCodo = clampAngle(map(v2x, 0, 1023, MIN_ANG, MAX_ANG));
  uint8_t tgtHomb = clampAngle(map(v2y, 0, 1023, MIN_ANG, MAX_ANG));

  angCintura = smoothMove(angCintura, tgtCint, maxStep);
  angPinza   = smoothMove(angPinza,   tgtPinz, maxStep);
  angCodo    = smoothMove(angCodo,    tgtCodo, maxStep);
  angHombro  = smoothMove(angHombro,  tgtHomb, maxStep);

  if (angPinza != lastAngPinza)    { servoPinza.write(angPinza);    lastAngPinza = angPinza; }
  if (angCintura != lastAngCintura){ servoCintura.write(angCintura);lastAngCintura = angCintura; }
  if (angCodo != lastAngCodo)      { servoCodo.write(angCodo);      lastAngCodo = angCodo; }
  if (angHombro != lastAngHombro)  { servoHombro.write(angHombro);  lastAngHombro = angHombro; }

  delay(stepDelay);
}