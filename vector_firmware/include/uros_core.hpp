#ifndef UROS_CORE_HPP
#define UROS_CORE_HPP

#include <Arduino.h>
#include <micro_ros_platformio.h>
#include <rclc_parameter/rclc_parameter.h>
#include <rmw_microros/rmw_microros.h>
#include <vector_interfaces/msg/encoder_ticks.h>
#include <vector_interfaces/msg/motor_ticks_sec.h>

#include "ESP32Encoder.h"
#include "PID.hpp"
#include "configurations.hpp"
#include "mcpwm_driver.hpp"

// . Macro Definations
/**
 * @brief Execute a function and abort on error
 *
 * Calls the expression and enters error loop if result isn't RCL_RET_OK
 */
#define RCCHECK(fn)                                                                                                    \
  {                                                                                                                    \
    rcl_ret_t temp_rc = fn;                                                                                            \
    if((temp_rc != RCL_RET_OK)) {                                                                                      \
      rclErrorLoop();                                                                                                  \
    }                                                                                                                  \
  }

/**
 * @brief Execute a function with soft error checking
 *
 * Calls the expression without action if result isn't RCL_RET_OK
 */
#define RCSOFTCHECK(fn)                                                                                                \
  {                                                                                                                    \
    rcl_ret_t temp_rc = fn;                                                                                            \
    if((temp_rc != RCL_RET_OK)) {                                                                                      \
    }                                                                                                                  \
  }

/**
 * @brief Executes code block every specified milliseconds
 *
 * Runs the given code X only if the elapsed time is greater than MS.
 */
#define EXECUTE_EVERY_N_MS(MS, X)                                                                                      \
  do {                                                                                                                 \
    static volatile int64_t init = -1;                                                                                 \
    if(init == -1) {                                                                                                   \
      init = uxr_millis();                                                                                             \
    }                                                                                                                  \
    if(uxr_millis() - init > MS) {                                                                                     \
      X;                                                                                                               \
      init = uxr_millis();                                                                                             \
    }                                                                                                                  \
  } while(0)

//. Function declarations for callbacks and hardware setup
/**
 * @brief Control loop callback function
 *
 * This function is called every 100ms (10Hz) and performs the following steps:
 *    1. Read the current encoder counts
 *    2. Calculates the encoder ticks/sec for each motor
 *    3. Computes the PID output for each motor.
 *    4. Sets the motor speeds
 *
 * @param timer  Pointer to the rcl_timer_t instance
 * @param last_call_time  Timestamp of the last time the timer was called
 */
void control_callback(rcl_timer_t* timer, int64_t last_call_time);

/**
 * @brief Encoder callback function.
 *
 * This function is called every 50 ms (20Hz) and publishes the encoder ticks for each motor.
 *
 * @param timer Pointer to the rcl_timer_t instance.
 * @param last_call_time Timestamp of the last time the timer was called.
 */
void encoder_callback(rcl_timer_t* timer, int64_t last_call_time);

/**
 * @brief Motor subscription callback function.
 *
 * This function is called when a new message is received on the "motor_ticks_sec" topic.
 * It updates the global motor setpoint variables with the received values.
 *
 * @param msgin Pointer to the received message.
 */
void motor_callback(const void* msgin);

/**
 * @brief Callback for updating PID controller parameters.
 *
 * This function is invoked when a parameter update is received by the parameter server.
 * It expects the parameter name in the format "<param>_motor<index>" (e.g. "Kp_motor1", "Ki_motor2", "Kd_motor3").
 * The function extracts the parameter type and motor index, selects the corresponding PIDController,
 * converts the new parameter value to a float, and updates the controller's tunings accordingly.
 *
 * If the parameter name format is invalid or the motor index is out of range, the update is rejected.
 *
 * @param old_param Pointer to the parameter before the update (can be NULL if not applicable).
 * @param new_param Pointer to the new parameter value.
 * @param context Unused callback context.
 * @return true if the parameter was updated successfully; false otherwise.
 */
bool param_callback(const Parameter* old_param, const Parameter* new_param, void* context);

/**
 * @brief Hardware setup function.
 *
 * This function initializes the encoders and the MCPWM motor driver.
 */
void setup_hardware();

/**
 * @brief Creates ROS 2 entities (node, publisher, subscription, timers, executors).
 *
 * @return true if successful.
 */
bool create_entities();

/**
 * @brief Destroys ROS 2 entities.
 *
 * @return true if successful.
 */
bool destroyEntities();

//. Global URos and hardware structures (defined in uros_core.cpp)
extern rclc_support_t support;
extern rcl_allocator_t allocator;
extern rcl_node_t node;
extern rclc_parameter_server_t param_server;
extern rclc_executor_t executor_one;
extern rclc_executor_t executor_two;

extern vector_interfaces__msg__EncoderTicks encoder_ticks_msg;
extern vector_interfaces__msg__MotorTicksSec motor_ticks_sec_msg;

extern rcl_publisher_t encoder_publisher;
extern rcl_subscription_t motor_subscription;
extern rcl_timer_t control_timer;
extern rcl_timer_t encoder_timer;

//. Global encoder and PID controller instances
extern ESP32Encoder encoder1;
extern ESP32Encoder encoder2;
extern ESP32Encoder encoder3;
extern ESP32Encoder encoder4;
extern PIDController pidMotor1;
extern PIDController pidMotor2;
extern PIDController pidMotor3;
extern PIDController pidMotor4;

//. Global variables for motor setpoints(encoder_ticks/sec) and control loop state
extern volatile float motor1_setpoint;
extern volatile float motor2_setpoint;
extern volatile float motor3_setpoint;
extern volatile float motor4_setpoint;
extern u_int64_t last_encoder_read_time;
extern int64_t lastCount1;
extern int64_t lastCount2;
extern int64_t lastCount3;
extern int64_t lastCount4;

/**
 * @brief State Machine States
 *
 * The state machine states for the main loop.
 */
enum states
{
  WAITING_AGENT,
  AGENT_AVAILABLE,
  AGENT_CONNECTED,
  AGENT_DISCONNECTED
};

extern enum states state;

//. Utility functions
/**
 * @brief  Flash the onboard LED a given number of times
 *
 * This function turns the LED on and off with a fixed delay
 *
 * @param n_times Number of times to flash the LED
 */
inline void flashLED(int n_times)
{
  for(int i = 0; i < n_times; i++) {
    digitalWrite(LED_BUILTIN, HIGH);
    delay(150);
    digitalWrite(LED_BUILTIN, LOW);
    delay(150);
  }
  delay(1000);
}

/**
 * @brief Error loop function
 *
 * This function is called if an error is encountered during initialization.
 * It continuously flashes the onboard LED.
 */
inline void rclErrorLoop()
{
  while(true) {
    flashLED(2);
  }
}
#endif  // UROS_CORE_HPP
