#include "sensors.h"

#include <Wire.h>
#include <math.h>

#include "config.h"

bool Sensors::begin() {
  state_ = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, false, false, false};
  gyroXOffset_ = 0.0f;
  gyroYOffset_ = 0.0f;
  gyroZOffset_ = 0.0f;
  pitchZeroDeg_ = 0.0f;
  rollZeroDeg_ = 0.0f;
  initialized_ = false;

  Wire.begin();
  state_.bmp180Present = devicePresent(BMP180_ADDRESS);
  state_.compassPresent = devicePresent(HMC5883L_ADDRESS);
  state_.mpuOk = devicePresent(MPU6050_ADDRESS);
  //state_.mpuOk = 1;  
  if (!state_.mpuOk && state_.compassPresent && state_.bmp180Present) return false;
  
  writeRegister(MPU6050_ADDRESS, 0x6B, 0x00); // wake up
  writeRegister(MPU6050_ADDRESS, 0x1B, 0x00); // gyro +/-250 deg/s
  writeRegister(MPU6050_ADDRESS, 0x1C, 0x00); // accel +/-2g
  delay(100);
  
  int32_t gxSum = 0;
  int32_t gySum = 0;
  int32_t gzSum = 0;
  for (uint16_t i = 0; i < GYRO_CALIBRATION_SAMPLES; ++i) {
    int16_t ax, ay, az, gx, gy, gz;
    if (readMpuRaw(ax, ay, az, gx, gy, gz)) {
      gxSum += gx;
      gySum += gy;
      gzSum += gz;
    }
    delay(20);
  }
  gyroXOffset_ = (float)gxSum / (float)GYRO_CALIBRATION_SAMPLES;
  gyroYOffset_ = (float)gySum / (float)GYRO_CALIBRATION_SAMPLES;
  gyroZOffset_ = (float)gzSum / (float)GYRO_CALIBRATION_SAMPLES;

  float pitchSum = 0.0f;
  float rollSum = 0.0f;
  for (uint16_t i = 0; i < LEVEL_CALIBRATION_SAMPLES; ++i) {
    int16_t axRaw, ayRaw, azRaw, gxRaw, gyRaw, gzRaw;
    if (readMpuRaw(axRaw, ayRaw, azRaw, gxRaw, gyRaw, gzRaw)) {
      float pitch, roll;
      accelAngles(
        ACC_X_SIGN * ((float)axRaw / 16384.0f),
        ACC_Y_SIGN * ((float)ayRaw / 16384.0f),
        ACC_Z_SIGN * ((float)azRaw / 16384.0f),
        pitch,
        roll
      );
      pitchSum += pitch;
      rollSum += roll;
    }
    delay(20);
  }
  pitchZeroDeg_ = pitchSum / (float)LEVEL_CALIBRATION_SAMPLES;
  rollZeroDeg_ = rollSum / (float)LEVEL_CALIBRATION_SAMPLES;
  state_.pitchDeg = 0.0f;
  state_.rollDeg = 0.0f;
  initialized_ = true;
  return true;
}

bool Sensors::update(float dt) {
  int16_t axRaw, ayRaw, azRaw, gxRaw, gyRaw, gzRaw;

  
  if (!readMpuRaw(axRaw, ayRaw, azRaw, gxRaw, gyRaw, gzRaw)) {
    state_.mpuOk = false;
    return false;
  }

  float axG = ACC_X_SIGN * ((float)axRaw / 16384.0f);
  float ayG = ACC_Y_SIGN * ((float)ayRaw / 16384.0f);
  float azG = ACC_Z_SIGN * ((float)azRaw / 16384.0f);

  state_.gyroXDegS = GYRO_X_SIGN * (((float)gxRaw - gyroXOffset_) / 131.0f);
  state_.gyroYDegS = GYRO_Y_SIGN * (((float)gyRaw - gyroYOffset_) / 131.0f);
  state_.gyroZDegS = GYRO_Z_SIGN * (((float)gzRaw - gyroZOffset_) / 131.0f);

  float accelPitchDeg = 0.0f;
  float accelRollDeg = 0.0f;
  accelAngles(axG, ayG, azG, accelPitchDeg, accelRollDeg);
  accelPitchDeg -= pitchZeroDeg_;
  accelRollDeg -= rollZeroDeg_;

  if (!initialized_ || dt <= 0.0f) {
    state_.pitchDeg = accelPitchDeg;
    state_.rollDeg = accelRollDeg;
  } else {
    float gyroPitch = state_.pitchDeg + state_.gyroYDegS * dt;
    float gyroRoll = state_.rollDeg + state_.gyroXDegS * dt;
    state_.pitchDeg = COMPLEMENTARY_ALPHA * gyroPitch
      + (1.0f - COMPLEMENTARY_ALPHA) * accelPitchDeg;
    state_.rollDeg = COMPLEMENTARY_ALPHA * gyroRoll
      + (1.0f - COMPLEMENTARY_ALPHA) * accelRollDeg;
  }

  state_.mpuOk = true;
  return true;
}

const SensorState &Sensors::state() const {
  return state_;
}

float Sensors::pitchDeg(){
  return state_.pitchDeg;
}


float Sensors::rollDeg()
{
  return state_.rollDeg;
}

bool Sensors::writeRegister(uint8_t address, uint8_t reg, uint8_t value) {
  Wire.beginTransmission(address);
  Wire.write(reg);
  Wire.write(value);
  return Wire.endTransmission() == 0;
}

bool Sensors::readMpuRaw(int16_t &ax, int16_t &ay, int16_t &az,
                         int16_t &gx, int16_t &gy, int16_t &gz) {
  
  Wire.beginTransmission(MPU6050_ADDRESS);
  Wire.write(0x3B);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(MPU6050_ADDRESS, (uint8_t)14) != 14) return false;

  ax = (int16_t)((Wire.read() << 8) | Wire.read());
  ay = (int16_t)((Wire.read() << 8) | Wire.read());
  az = (int16_t)((Wire.read() << 8) | Wire.read());
  Wire.read();
  Wire.read();
  gx = (int16_t)((Wire.read() << 8) | Wire.read());
  gy = (int16_t)((Wire.read() << 8) | Wire.read());
  gz = (int16_t)((Wire.read() << 8) | Wire.read());

  if (isnan(ax) || isnan(ay) || isnan(az) || isnan(gx) || isnan(gy) || isnan(gz)) {
    ax = 0;
    ay = 0;
    az = 0;
    gx = 0;
    gy = 0;
    gz = 0;
    return false;
  }  
 /* ax = 0;
  ay = 0;
  az = 0;
  gx = 0;
  gy = 0;
  gz = 0;
  */
  return true;
}

bool Sensors::devicePresent(uint8_t address) {
  Wire.beginTransmission(address);
  return Wire.endTransmission() == 0;
}

void Sensors::accelAngles(float axG, float ayG, float azG,
                          float &pitchDeg, float &rollDeg) const {
  pitchDeg = atan2(-axG, sqrt(abs(ayG * ayG + azG * azG))) * 57.2957795f;
  rollDeg = atan2(ayG, azG) * 57.2957795f;
}
