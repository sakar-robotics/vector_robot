// Copyright (c) 2025 Sakar Robotics

#ifndef CONFIGURATIONS_HPP_
#define CONFIGURATIONS_HPP_

namespace Config
{

//* MOTOR DRIVE PINS: direction and PWM pins
struct MotorDriverPins
{
  int dir;
  int pwm;
};

static constexpr MotorDriverPins MOTOR_DRIVER_PINS[4] = {
  { 2, 3 },  // Motor 1
  { 4, 5 },  // Motor 2
  { 6, 7 },  // Motor 3
  { 8, 9 }   // Motor 4
};

//* ENCODER PINS: Encoder has two pins(A and B)
struct EncoderPins
{
  int A;
  int B;
};

static constexpr EncoderPins ENCODER_PINS[4] = {
  { 18, 19 },  // Encoder 1
  { 20, 21 },  // Encoder 2
  { 22, 23 },  // Encoder 3
  { 24, 25 }   // Encoder 4
};

//. PID configuration parameters for each motor
struct PIDParameters
{
  float Kp;
  float Ki;
  float Kd;
  float alpha;
};

static constexpr PIDParameters PID_PARAMS[4] = {
  { 1.0f, 0.01f, 0.1f, 0.1f },  // Motor 1
  { 1.0f, 0.01f, 0.1f, 0.1f },  // Motor 2
  { 1.0f, 0.01f, 0.1f, 0.1f },  // Motor 3
  { 1.0f, 0.01f, 0.1f, 0.1f }   // Motor 4
};

// Other Parameters

}  // namespace Config

#endif  // CONFIGURATIONS_HPP_
