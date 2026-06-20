#pragma once

class SimplePID {
public:
  SimplePID();
  void begin(float kp, float ki, float kd, float outMin, float outMax,
             float integralMin, float integralMax);
  void reset();
  float update(float error, float dt);

private:
  float kp_;
  float ki_;
  float kd_;
  float outMin_;
  float outMax_;
  float integralMin_;
  float integralMax_;
  float integral_;
  float previousError_;
  float previousOutput_;
  bool hasPreviousError_;
};

float clampFloat(float value, float low, float high);
