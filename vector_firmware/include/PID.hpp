#ifndef PID_HPP_
#define PID_HPP_

#include <Arduino.h>

/**
 * @brief PIDController class for motor speed control using encoder ticks per second.
 * 
 * This class implements an enhanced PID controller with the following characteristics:
 * 
 * 1. Derivative Term Calculation:
 *    - Uses the derivative of the measurement to avoid the derivative kick caused by abrupt setpoint changes.
 *    - Equation: D = Kd * (-d(measurement) / dt)
 * 
 * 2. Integral Windup Prevention:
 *    - The integral term is accumulated only if Ki != 0.
 *    - Uses conditional integration with clamping:
 *      I(t) = I(t-1) + error * dt  if |I(t)| < maxIntegral
 * 
 * 3. Non-Blocking Computation:
 *    - The compute() function returns the previous output if the sample time is not elapsed.
 * 
 * Usage: 
 * To use this  class in the project, include "PID.hpp" and create an instance of the PIDController class.
 * Update the setpoint in your callbacks and call the compute() continuously in your loop() to receive
 * a PWM output value in the configured range. 
 */
class PIDController
{
public:
  /**
     * @brief Constructor for PIDController.
     * 
     * @param _Kp Proportional gain.
     * @param _Ki Integral gain.
     * @param _Kd Derivative gain.
     * @param _derivativeFilterAlpha Coefficient for low-pass filtering of the derivative (range 0.0 - 1.0).
     * @param _minOutput Minimum output value (e.g., 0 for PWM).
     * @param _maxOutput Maximum output value (e.g., 100 for PWM).
     * @param _sampleTime Fixed sample time in milliseconds for PID computation.
     */
  PIDController(float _Kp, float _Ki, float _Kd, float _derivativeFilterAlpha,
                float _minOutput, float _maxOutput, uint32_t _sampleTime);

  /**
     * @brief Set new PID gains dynamically.
     * 
     * @param _Kp Proportional gain.
     * @param _Ki Integral gain.
     * @param _Kd Derivative gain.
     */
  void setTunings(float _Kp, float _Ki, float _Kd);

  /**
     * @brief Compute the PID controller output.
     *  
     * This method calculates the control output based on the setpoint and measurement.
     * The following steps are executed: 
     * 
     *  - Check if the fixed sample time has elapsed; if not, return the last output.
     * 
     *  - Compute dt, the elapsed time (in seconds).
     * 
     *  - Compute error: error = setpoint - measurement.
     * 
     *  - Accumulate the integral term only if Ki ≠ 0 and clamp it to prevent windup.
     * 
     *  - Compute the derivative term based on the change in measurement:
     *        derivative = -((measurement - prevMeasurement)/dt)
     * 
     *  - Apply low-pass filtering on the derivative:
     *        filteredDerivative = derivativeFilterAlpha * derivative + (1 - derivativeFilterAlpha) * prevFilteredDerivative
     * 
     *  - Compute the total output: output = Kp * error + Ki * integral + Kd * filteredDerivative.
     * 
     *  - Clamp the output between minOutput and maxOutput.
     * 
     * @param setpoint Desired encoder ticks per second.
     * @param measurement Current encoder ticks per second.
     * @return float PID controller output.
     */
  float compute(float setpoint, float measurement);

  /**
     * @brief Reset the PID controller's internal state.
     */
  void reset();

private:
  // PID gains
  float Kp, Ki, Kd;
  // Derivative filter coefficient for low-pass filtering (range 0.0 - 1.0)
  float derivativeFilterAlpha;
  // Internal state for PID components
  float integral;
  float prevError;
  float prevFilteredDerivative;
  float prevMeasurement;  // For derivative calculation using measurement
  // Output limits
  float minOutput, maxOutput;
  // Maximum allowable integral term for anti-windup
  float maxIntegral;
  // Fixed sample time in milliseconds
  uint32_t sampleTime;
  // Last time the compute() function computed a new output
  uint32_t lastTime;
  // Last computed output value
  float lastOutput;
};

#endif  // PID_HPP_
