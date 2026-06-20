#pragma once

#include <Arduino.h>
#include <Servo.h>

class ServoOutputs {
public:
  void begin();
  void write(float aileronCommand, float elevatorCommand, float throttleCommand, float dt);
  float leftAileronDeg() const;
  float rightAileronDeg() const;
  float elevatorDeg() const;

private:
  Servo leftAileron_;
  Servo rightAileron_;
  Servo elevator_;
  Servo throttle_;
  float leftDeg_;
  float rightDeg_;
  float elevatorDeg_;

  float applySlew(float targetDeg, float currentDeg, float dt) const;
  int surfaceMicroseconds(float degrees) const;
};
