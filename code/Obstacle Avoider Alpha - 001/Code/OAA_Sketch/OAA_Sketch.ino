#include <AFMotor.h>
#define TRIG_PIN A0
#define ECHO_PIN A1

long duration = 0;
int distance = 0;
int maxSpeed = 255;
int turningDelay = 300;

AF_DCMotor motorLeft(4, MOTOR12_1KHZ), motorRight(3,  MOTOR12_1KHZ);

void setup() {

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  Serial.begin(9600);
  readSoner();
  delay(50);
  readSoner();
  delay(50);
  readSoner();
  delay(50);

}

void loop() {
  delay(40);
  Serial.print("Distance: ");
  Serial.print(distance);
  Serial.println(" cm");

  if (distance < 30) {
    stopRuning();
    delay(turningDelay);
    turnLeft();
    delay(turningDelay);
    stopRuning();
  }else
  {
      runForward();
  }

  readSoner();
}

void readSoner() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  duration = pulseIn(ECHO_PIN, HIGH);
  distance = duration * 0.034 / 2;
}
void runForward() {
  motorLeft.setSpeed(maxSpeed);
  motorLeft.run(FORWARD);
  motorRight.setSpeed(maxSpeed);
  motorRight.run(FORWARD);
}
void turnLeft() {
  delay(turningDelay);
  motorLeft.setSpeed(maxSpeed);
  motorLeft.run(BACKWARD);
  motorRight.setSpeed(maxSpeed);
  motorRight.run(FORWARD);
}
void stopRuning() {
  motorLeft.run(RELEASE);
  motorRight.run(RELEASE);
}
