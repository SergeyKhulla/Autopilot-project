#include <Arduino.h>

#include "config.h"
#include "pid.h"
#include "sensors.h"
#include "servos.h"

#if defined(__AVR__)
#include <avr/io.h>
#endif

Sensors sensors;
ServoOutputs servos;
SimplePID pitchPid;
SimplePID rollPid;

//float targetPitchDeg = 0.0f;
//float targetRollDeg = 0.0f;
//float targetThrottle = 0.0f;
//float elevatorCommand = 0.0f;
//float aileronCommand = 0.0f;

uint32_t previousMs = 0;
uint32_t lastLogMs = 0;

const char *modeName() {
#if BENCH_ATTITUDE_DEMO
  return "BENCH";
#elif JSBSIM_LIKE_MODE
  return "JSBSIM_LIKE";
#else
  return "UNKNOWN";
#endif
}

const char *safetyText() {
  return sensors.state().mpuOk ? "NORMAL" : "IMU_FAIL";
}

uint8_t readAndClearResetFlags() {
#if defined(__AVR__)
  uint8_t flags = MCUSR;
  MCUSR = 0;
  return flags;
#else
  return 0;
#endif
}

void printResetFlags(uint8_t flags) {
  Serial.print(F("reset_flags=0x"));
  if (flags < 16) Serial.print('0');
  Serial.print(flags, HEX);
  Serial.print(F(" ["));
#if defined(__AVR__)
  if (flags & _BV(PORF)) Serial.print(F(" POR"));
  if (flags & _BV(BORF)) Serial.print(F(" BOR"));
  if (flags & _BV(EXTRF)) Serial.print(F(" EXTR"));
  if (flags & _BV(WDRF)) Serial.print(F(" WDR"));
  if (flags == 0) Serial.print(F(" none"));
#else
  Serial.print(F(" unsupported"));
#endif
  Serial.println(F(" ]"));
}

void setup() {

  uint8_t resetFlags = readAndClearResetFlags();

  Serial.begin(SERIAL_BAUD);
  delay(500);
  

  printResetFlags(resetFlags);
  Serial.println(F("simple_autopilot: startup calibration, keep the stand still"));

  servos.begin();


  servos.write(-1, -1, 0, 1);

  delay(1500);

  servos.write(1, 1, 0, 2);

  delay(2500);
  
  servos.write(0, 0, 0, 1);

  delay(1500);

  
  bool sensorsOk = sensors.begin();

  pitchPid.begin(PITCH_KP, PITCH_KI, PITCH_KD, -1.0f, 1.0f, -500.0f, 500.0f);
  rollPid.begin(ROLL_KP, ROLL_KI, ROLL_KD, -1.0f, 1.0f, -5.0f, 5.0f);

  previousMs = millis();
  const SensorState &s = sensors.state();
  Serial.println(s.bmp180Present ? F("bmp180 OK") : F("bmp180 MISSING"));
  Serial.println(s.compassPresent ? F("compass OK") : F("compass MISSING"));
  Serial.println(s.mpuOk ? F("MPU6050 OK") : F("MPU6050 MISSING"));
  Serial.println(F("Expected at rest: pitch near 0, roll near 0, gyro values near 0"));
}

void loop() {
  //uint32_t nowUs = micros();
  uint32_t nowMs = millis();
  //float dt = (float)(nowUs - previousMs) * 0.000001f;


  float dt = (float)(nowMs - previousMs) * 0.001f;
  
  if (dt <= 0.0f || dt > 0.2f) dt = 0.02f;

  

  sensors.update(dt);
  const SensorState &s = sensors.state();


#if BENCH_ATTITUDE_DEMO
  float targetPitchDeg = 0.0f;
  float targetRollDeg = 0.0f;
  float targetThrottle = 0.0f;
#elif JSBSIM_LIKE_MODE
  float targetPitchDeg = 0.0f;
  float targetRollDeg = 0.0f;
  float targetThrottle = 0.35f;
#endif

  float pitchErrorDeg = targetPitchDeg - s.pitchDeg;
  float rollErrorDeg = targetRollDeg - s.rollDeg;

  float elevatorCommand = pitchPid.update(pitchErrorDeg, dt);
  float aileronCommand = rollPid.update(rollErrorDeg, dt);

#if BENCH_ATTITUDE_DEMO
  float servoElevatorCommand = clampFloat(
    elevatorCommand * BENCH_VISUAL_SERVO_GAIN,
    -1.0f,
    1.0f
  );
  float servoAileronCommand = clampFloat(
    aileronCommand * BENCH_VISUAL_SERVO_GAIN,
    -1.0f,
    1.0f
  );
#else
  float servoElevatorCommand = elevatorCommand;
  float servoAileronCommand = aileronCommand;
#endif

  servos.write(servoAileronCommand, servoElevatorCommand, targetThrottle, dt);

  delay(11);
  
  if (nowMs - lastLogMs >= SERIAL_LOG_INTERVAL_MS) {
    lastLogMs = nowMs;
    Serial.print(nowMs);
  /*  Serial.print("mode=");
    Serial.print(modeName());
    Serial.print(" pitch=");
    Serial.print(ensors.pitchDeg(), 2);
    Serial.print(" roll=");
    Serial.print(s.rollDeg, 2);
    Serial.print(" gyro_x=");
    Serial.print(s.gyroXDegS, 2);
    Serial.print(" gyro_y=");
    Serial.print(s.gyroYDegS, 2);
    Serial.print(" gyro_z=");
    Serial.print(s.gyroZDegS, 2);
  */
    Serial.print(" pitch_error=");
    Serial.print(pitchErrorDeg, 2);
    Serial.print(" roll_error=");
    Serial.print(rollErrorDeg, 2);
    Serial.print(" elevator_cmd=");
    Serial.print(elevatorCommand, 3);
    Serial.print(" aileron_cmd=");
    Serial.print(aileronCommand, 3);
  /*
    Serial.print(" left_servo_deg=");
    Serial.print(servos.leftAileronDeg(), 1);
    Serial.print(" right_servo_deg=");
    Serial.print(servos.rightAileronDeg(), 1);
    Serial.print(" elevator_servo_deg=");
    Serial.print(servos.elevatorDeg(), 1);*/
    Serial.print(F(" pitchDeg="));
    Serial.print(s.pitchDeg, 2);
    Serial.print(F(" rollDeg="));
    Serial.println(s.rollDeg, 2);
  
  }
  previousMs = nowMs; 
}
