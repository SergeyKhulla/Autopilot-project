#include "pid.h"
float clampFloat(float value, float low, float high) {
  if (value < low) return low;
  if (value > high) return high;
  return value;
}

SimplePID::SimplePID() {
  begin(0.0f, 0.0f, 0.0f, -1.0f, 1.0f, -100.0f, 100.0f);
}

void SimplePID::begin(float kp, float ki, float kd, float outMin, float outMax,
                      float integralMin, float integralMax) {
  kp_ = kp;
  ki_ = ki;
  kd_ = kd;
  outMin_ = outMin;
  outMax_ = outMax;
  integralMin_ = integralMin;
  integralMax_ = integralMax;
  reset();
}

void SimplePID::reset() {
  integral_ = 0.0f;
  previousError_ = 0.0f;
  previousOutput_ = 0.0f;
  hasPreviousError_ = false;
}

float SimplePID::update(float error, float dt) {
  if (dt <= 0.0f) return previousOutput_;

  float derivative = 0.0f;
  if (hasPreviousError_) {
    derivative = (error - previousError_) / dt;
  }

  float candidateIntegral = clampFloat(
    integral_*0.999 + error * dt, // добавил утечку в интеграл, чтобы бороться с windup
    integralMin_,
    integralMax_
  );

  

  float rawOutput = kp_ * error + ki_ * candidateIntegral + kd_ * derivative;
  bool saturatedHighInErrorDirection = rawOutput > outMax_ && error > 0.0f;
  bool saturatedLowInErrorDirection = rawOutput < outMin_ && error < 0.0f;

  if (!saturatedHighInErrorDirection && !saturatedLowInErrorDirection) {
    integral_ = candidateIntegral;
  }

  rawOutput = kp_ * error + ki_ * integral_ + kd_ * derivative;
  float output = clampFloat(rawOutput, outMin_, outMax_);

  previousError_ = error;
  previousOutput_ = output;
  hasPreviousError_ = true;
  return output;
}
