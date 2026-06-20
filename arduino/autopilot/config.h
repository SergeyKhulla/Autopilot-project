#pragma once

// Select one mode. Bench mode is the default for the physical stand.
#define BENCH_ATTITUDE_DEMO 1
#define JSBSIM_LIKE_MODE 0

#define SERIAL_BAUD 115200UL
#define SERIAL_LOG_INTERVAL_MS 1000UL

#define LEFT_AILERON_PIN 9
#define RIGHT_AILERON_PIN 10
#define ELEVATOR_PIN 8
#define THROTTLE_PIN 11

#define MPU6050_ADDRESS 0x68
#define BMP180_ADDRESS 0x77
#define HMC5883L_ADDRESS 0x0D

// Axis signs are the first place to fix an incorrectly mounted MPU6050.
#define ACC_X_SIGN 1.0f
#define ACC_Y_SIGN 1.0f
#define ACC_Z_SIGN 1.0f
#define GYRO_X_SIGN 1.0f
#define GYRO_Y_SIGN 1.0f
#define GYRO_Z_SIGN 1.0f

// Positive command conventions:
// elevator > 0 means nose-up command;
// aileron > 0 means right-roll command before per-servo signs.
#define AILERON_SIGN 1.0f
#define ELEVATOR_SIGN -1.0f
#define LEFT_AILERON_SIGN 1.0f
#define RIGHT_AILERON_SIGN -1.0f

#define BENCH_VISUAL_SERVO_GAIN 1.0f
#define ENABLE_SURFACE_SLEW_LIMIT 1
#define BENCH_SURFACE_SLEW_DEG_S 90.0f
#define SURFACE_MAX_DEFLECTION_DEG 30.0f

#define SERVO_MIN_US 1000
#define SERVO_NEUTRAL_US 1500
#define SERVO_MAX_US 2000
#define THROTTLE_MIN_US 1000
#define THROTTLE_MAX_US 2000

#define COMPLEMENTARY_ALPHA 0.98f
#define GYRO_CALIBRATION_SAMPLES 100
#define LEVEL_CALIBRATION_SAMPLES 100

#define PITCH_KP 0.08f
#define PITCH_KI 0.002f
#define PITCH_KD 0.004f
#define ROLL_KP 0.06f
#define ROLL_KI 0.002f
#define ROLL_KD 0.003f
