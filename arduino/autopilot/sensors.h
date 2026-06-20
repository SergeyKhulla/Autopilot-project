#pragma once

#include <Arduino.h>

struct SensorState {
  float pitchDeg;
  float rollDeg;
  float gyroXDegS;
  float gyroYDegS;
  float gyroZDegS;
  bool mpuOk;
  bool bmp180Present;
  bool compassPresent;
};

class Sensors {
public:
  bool begin();
  bool update(float dt);
  float pitchDeg();
  float rollDeg();
  const SensorState &state() const;

private:
  SensorState state_;
  float gyroXOffset_;
  float gyroYOffset_;
  float gyroZOffset_;
  float pitchZeroDeg_;
  float rollZeroDeg_;
  bool initialized_;

  bool writeRegister(uint8_t address, uint8_t reg, uint8_t value);
  bool readMpuRaw(int16_t &ax, int16_t &ay, int16_t &az,
                  int16_t &gx, int16_t &gy, int16_t &gz);
  bool devicePresent(uint8_t address);
  void accelAngles(float axG, float ayG, float azG,
                   float &pitchDeg, float &rollDeg) const;
};
