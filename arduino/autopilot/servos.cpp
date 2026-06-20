#include "servos.h"

#include <math.h>

#include "config.h"
#include "pid.h"



void ServoOutputs::begin() {
  leftAileron_.attach(LEFT_AILERON_PIN);
  rightAileron_.attach(RIGHT_AILERON_PIN);
  elevator_.attach(ELEVATOR_PIN);
  throttle_.attach(THROTTLE_PIN);
  leftDeg_ = 0.0f;
  rightDeg_ = 0.0f;
  elevatorDeg_ = 0.0f;
  leftAileron_.writeMicroseconds(SERVO_NEUTRAL_US);
  rightAileron_.writeMicroseconds(SERVO_NEUTRAL_US);
  elevator_.writeMicroseconds(SERVO_NEUTRAL_US);
  throttle_.writeMicroseconds(THROTTLE_MIN_US);
}

void ServoOutputs::write(float aileronCommand, float elevatorCommand,
                         float throttleCommand, float dt) {
  float leftTargetDeg = SURFACE_MAX_DEFLECTION_DEG
    * clampFloat(LEFT_AILERON_SIGN * AILERON_SIGN * aileronCommand, -1.0f, 1.0f);
  float rightTargetDeg = SURFACE_MAX_DEFLECTION_DEG
    * clampFloat(RIGHT_AILERON_SIGN * AILERON_SIGN * -aileronCommand, -1.0f, 1.0f);
  float elevatorTargetDeg = SURFACE_MAX_DEFLECTION_DEG
    * clampFloat(ELEVATOR_SIGN * elevatorCommand, -1.0f, 1.0f);

  leftDeg_ = applySlew(leftTargetDeg, leftDeg_, dt);
  rightDeg_ = applySlew(rightTargetDeg, rightDeg_, dt);
  elevatorDeg_ = applySlew(elevatorTargetDeg, elevatorDeg_, dt);

  leftAileron_.writeMicroseconds(surfaceMicroseconds(leftDeg_));
  rightAileron_.writeMicroseconds(surfaceMicroseconds(rightDeg_));
  elevator_.writeMicroseconds(surfaceMicroseconds(elevatorDeg_));

  float throttle = clampFloat(throttleCommand, 0.0f, 1.0f);
  int throttleUs = THROTTLE_MIN_US
    + (int)((THROTTLE_MAX_US - THROTTLE_MIN_US) * throttle);
  throttle_.writeMicroseconds(throttleUs);
}

float ServoOutputs::leftAileronDeg() const {
  return leftDeg_;
}

float ServoOutputs::rightAileronDeg() const {
  return rightDeg_;
}

float ServoOutputs::elevatorDeg() const {
  return elevatorDeg_;
}

float ServoOutputs::applySlew(float targetDeg, float currentDeg, float dt) const {
#if ENABLE_SURFACE_SLEW_LIMIT
  if (dt <= 0.0f) return currentDeg;
  float maxDelta = BENCH_SURFACE_SLEW_DEG_S * dt;
  return clampFloat(targetDeg, currentDeg - maxDelta, currentDeg + maxDelta);
#else
  (void)currentDeg;
  (void)dt;
  return targetDeg;
#endif
}

int ServoOutputs::surfaceMicroseconds(float degrees) const {
  float normalized = clampFloat(degrees / SURFACE_MAX_DEFLECTION_DEG, -1.0f, 1.0f);
  int us = SERVO_NEUTRAL_US + (int)(normalized * 500.0f);
  if (us < SERVO_MIN_US) us = SERVO_MIN_US;
  if (us > SERVO_MAX_US) us = SERVO_MAX_US;
  return us;
}
