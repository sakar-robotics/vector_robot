// Copyright (c) 2025 Sakar Robotics
#include "PID.hpp"

PIDController::PIDController(float _Kp, float _Ki, float _Kd, float _derivativeFilterAlpha, float _minOutput,
                             float _maxOutput, uint32_t _sampleTime)
  : Kp(_Kp)
  , Ki(_Ki)
  , Kd(_Kd)
  , derivativeFilterAlpha(_derivativeFilterAlpha)
  , minOutput(_minOutput)
  , maxOutput(_maxOutput)
  , sampleTime(_sampleTime)
{
  integral               = 0.0f;
  prevError              = 0.0f;
  prevFilteredDerivative = 0.0f;
  prevMeasurement        = 0.0f;
  lastOutput             = 0.0f;
  lastTime               = millis();

  // Calculate maximum integral to mitigate windup (if Ki is non-zero)
  if(Ki != 0.0f)
    maxIntegral = maxOutput / Ki;
  else
    maxIntegral = 0.0f;
}

void PIDController::setTunings(float _Kp, float _Ki, float _Kd)
{
  Kp = _Kp;
  Ki = _Ki;
  Kd = _Kd;
  if(Ki != 0.0f)
    maxIntegral = maxOutput / Ki;
}

float PIDController::compute(float setpoint, float measurement)
{
  uint32_t now = millis();
  // Check if the fixed sample time has elapsed.
  if((now - lastTime) < sampleTime) {
    return lastOutput;  // Return the last output if not elapsed
  }

  // Calculate elapsed time
  float dt = (now - lastTime) / 1000.0f;  // seconds
  lastTime = now;

  // Compute error: error = setpoint - measurement.
  float error = setpoint - measurement;

  // Conditional integration (only if Ki != 0)
  if(Ki != 0.0f) {
    integral += error * dt;
    // Clamp the integral term to prevent windup
    integral = constrain(integral, -maxIntegral, maxIntegral);
  } else {
    integral = 0.0f;
  }

  // Proportional component
  float Pout = Kp * error;
  // Integral Component
  float Iout = Ki * integral;

  // Derivative component: use measurement derivative to avoid derivative kick.
  // D = Kd * ( - (d(measurement)/dt) )
  float derivative = (measurement - prevMeasurement) / dt;
  derivative       = -derivative;  // Invert derivative, so D counters the measurement change

  // LOW PASS FILTER
  float filteredDerivative =
      derivativeFilterAlpha * derivative + (1.0f - derivativeFilterAlpha) * prevFilteredDerivative;
  float Dout = Kd * filteredDerivative;

  // Save State for the next iteration
  prevError              = error;
  prevFilteredDerivative = filteredDerivative;
  prevMeasurement        = measurement;

  // Calculate total output.
  float output = Pout + Iout + Dout;

  // Clamp output.
  output = constrain(output, minOutput, maxOutput);

  // Save computed output.
  lastOutput = output;

  return output;
}

void PIDController::reset()
{
  integral               = 0.0f;
  prevError              = 0.0f;
  prevFilteredDerivative = 0.0f;
  prevMeasurement        = 0.0f;
  lastOutput             = 0.0f;
  lastTime               = millis();
}
