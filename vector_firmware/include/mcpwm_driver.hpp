#ifndef MCPWM_DRIVER_HPP_
#define MCPWM_DRIVER_HPP_

#include <Arduino.h>

#include "configurations.hpp"

/**
 * @brief Initializes the MCPWM modules.
 *
 * This function sets up the PWM and direction pins based on the configuration provided in
 * Config::MOTOR_DRIVER_PINS. It also initializes the MCPWM timers and parameters.
 */
void setupMCPWM();

/**
 * @brief Stops the specified motor.
 *
 * This function stops the motor by setting the corresponding MCPWM signal low.
 *
 * @param motor Motor number (1-4).
 */
void motorStop(uint8_t motor);

/**
 * @brief Sets the specified motor to full speed.
 *
 * This function sets the motor to operate at full speed by setting the corresponding MCPWM signal high.
 *
 * @param motor Motor number (1-4).
 */
void motorFullSpeed(uint8_t motor);

/**
 * @brief Sets the speed and direction of the specified motor.
 *
 * If speed is zero, the motor is stopped. Otherwise, the motor's direction is determined by the sign of
 * the speed, and the PWM duty cycle is set according to the speed value.
 *
 * @param motor Motor number (1-4).
 * @param speed Speed value (-100 to 100). A negative speed reverses the direction.
 */
void motorSetSpeed(uint8_t motor, int8_t speed);

/**
 * @brief Maps a value from one range to another.
 *
 * @param x Value to be mapped.
 * @param in_min Minimum value of the input range.
 * @param in_max Maximum value of the input range.
 * @param out_min Minimum value of the output range.
 * @param out_max Maximum value of the output range.
 * @return Mapped value.
 */
int speed_map(int x, int in_min, int in_max, int out_min, int out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
#endif  // MCPWM_DRIVER_HPP_
