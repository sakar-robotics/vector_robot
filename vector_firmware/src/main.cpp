// Copyright (c) 2025 Sakar Robotics

#include <Arduino.h>
#include <micro_ros_platformio.h>
#include <rcl/error_handling.h>
#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>
#include <rmw_microros/rmw_microros.h>
#include <stdio.h>
#include <vector_interfaces/msg/encoder_ticks.h>
#include <vector_interfaces/msg/motor_ticks_sec.h>

#include "ESP32Encoder.h"
#include "PID.hpp"
#include "configurations.hpp"
#include "mcpwm_driver.hpp"

// Uncomment one of the following transport definations as needed
// #define USE_WIFI_TRANSPORT
#define USE_SERIAL_TRANSPORT

#define RCCHECK(fn)                                                                                                    \
  {                                                                                                                    \
    rcl_ret_t temp_rc = fn;                                                                                            \
    if((temp_rc != RCL_RET_OK)) {                                                                                      \
      rclErrorLoop();                                                                                                  \
    }                                                                                                                  \
  }

#define RCSOFTCHECK(fn)                                                                                                \
  {                                                                                                                    \
    rcl_ret_t temp_rc = fn;                                                                                            \
    if((temp_rc != RCL_RET_OK)) {                                                                                      \
    }                                                                                                                  \
  }

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

//* URos support structures
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rclc_executor_t executor_one;
rclc_executor_t executor_two;

//* Message types
vector_interfaces__msg__EncoderTicks encoder_ticks_msg;
vector_interfaces__msg__MotorTicksSec motor_ticks_sec_msg;

//* ROS 2 communication objects
rcl_publisher_t encoder_publisher;
rcl_subscription_t motor_subscription;
rcl_timer_t control_timer;
rcl_timer_t encoder_timer;

// State Machine States
enum states
{
  WAITING_AGENT,
  AGENT_AVAILABLE,
  AGENT_CONNECTED,
  AGENT_DISCONNECTED
} state;

// Global Encoder Instances
ESP32Encoder encoder1;
ESP32Encoder encoder2;
ESP32Encoder encoder3;
ESP32Encoder encoder4;

// PID Controllers instances for four motors(100ms)
PIDController pidMotor1(Config::PID_PARAMS[0].Kp,
                        Config::PID_PARAMS[0].Ki,
                        Config::PID_PARAMS[0].Kd,
                        Config::PID_PARAMS[0].alpha,
                        -100.0f,
                        100.0f,
                        100);
PIDController pidMotor2(Config::PID_PARAMS[1].Kp,
                        Config::PID_PARAMS[1].Ki,
                        Config::PID_PARAMS[1].Kd,
                        Config::PID_PARAMS[1].alpha,
                        -100.0f,
                        100.0f,
                        100);
PIDController pidMotor3(Config::PID_PARAMS[2].Kp,
                        Config::PID_PARAMS[2].Ki,
                        Config::PID_PARAMS[2].Kd,
                        Config::PID_PARAMS[2].alpha,
                        -100.0f,
                        100.0f,
                        100);
PIDController pidMotor4(Config::PID_PARAMS[3].Kp,
                        Config::PID_PARAMS[3].Ki,
                        Config::PID_PARAMS[3].Kd,
                        Config::PID_PARAMS[3].alpha,
                        -100.0f,
                        100.0f,
                        100);

//. Global Variables

//* Motor setpoints (encoder ticks/sec) received via the motor_callback
volatile float motor1_setpoint = 0.0f;
volatile float motor2_setpoint = 0.0f;
volatile float motor3_setpoint = 0.0f;
volatile float motor4_setpoint = 0.0f;

// Control loop timings and encoder count difference
u_int64_t last_encoder_read_time;
int64_t lastCount1 = 0;
int64_t lastCount2 = 0;
int64_t lastCount3 = 0;
int64_t lastCount4 = 0;

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
void control_callback(rcl_timer_t* timer, int64_t last_call_time)
{
  RCLC_UNUSED(last_call_time);

  if(timer != NULL) {
    uint64_t now           = millis();
    float dt               = (now - last_encoder_read_time) / 1000.0f;  // Seconds
    last_encoder_read_time = now;

    // Read current encoder counts directly from the encoder instances
    int64_t current_encoder1 = encoder1.getCount();
    int64_t current_encoder2 = encoder2.getCount();
    int64_t current_encoder3 = encoder3.getCount();
    int64_t current_encoder4 = encoder4.getCount();

    // Calculate encoder ticks/sec
    float motor1_ticks_per_sec = (current_encoder1 - lastCount1) / dt;
    float motor2_ticks_per_sec = (current_encoder2 - lastCount2) / dt;
    float motor3_ticks_per_sec = (current_encoder3 - lastCount3) / dt;
    float motor4_ticks_per_sec = (current_encoder4 - lastCount4) / dt;

    // Compute PID outputs for each motor
    float motor1_pwm = pidMotor1.compute(motor1_setpoint, motor1_ticks_per_sec);
    float motor2_pwm = pidMotor2.compute(motor2_setpoint, motor2_ticks_per_sec);
    float motor3_pwm = pidMotor3.compute(motor3_setpoint, motor3_ticks_per_sec);
    float motor4_pwm = pidMotor4.compute(motor4_setpoint, motor4_ticks_per_sec);

    // Set the motor speeds
    motorSetSpeed(1, motor1_pwm);
    motorSetSpeed(2, motor2_pwm);
    motorSetSpeed(3, motor3_pwm);
    motorSetSpeed(4, motor4_pwm);

    // Store current encoder counts for the next iteration
    lastCount1 = current_encoder1;
    lastCount2 = current_encoder2;
    lastCount3 = current_encoder3;
    lastCount4 = current_encoder4;
  }
}

/**
 * @brief Encoder callback function.
 *
 * This function is called every 50 ms (20Hz) and publishes the encoder ticks for each motor.
 *
 * @param timer Pointer to the rcl_timer_t instance.
 * @param last_call_time Timestamp of the last time the timer was called.
 */
void encoder_callback(rcl_timer_t* timer, int64_t last_call_time)
{
  RCLC_UNUSED(last_call_time);
  if(timer != NULL) {
    // Populate the encoder ticks message
    encoder_ticks_msg.motor1_encoder_ticks = encoder1.getCount();
    encoder_ticks_msg.motor2_encoder_ticks = encoder2.getCount();
    encoder_ticks_msg.motor3_encoder_ticks = encoder3.getCount();
    encoder_ticks_msg.motor4_encoder_ticks = encoder4.getCount();

    // Publish the encoder ticks message
    RCSOFTCHECK(rcl_publish(&encoder_publisher, &encoder_ticks_msg, NULL));
  }
}

/**
 * @brief Motor subscription callback function.
 *
 * This function is called when a new message is received on the "motor_ticks_sec" topic.
 * It updates the global motor setpoint variables with the received values.
 *
 * @param msgin Pointer to the received message.
 */
void motor_callback(const void* msgin)
{
  const vector_interfaces__msg__MotorTicksSec* motor_ticks_sec_msg =
      (const vector_interfaces__msg__MotorTicksSec*)msgin;

  motor1_setpoint = motor_ticks_sec_msg->motor1_encoder_ticks_per_sec;
  motor2_setpoint = motor_ticks_sec_msg->motor2_encoder_ticks_per_sec;
  motor3_setpoint = motor_ticks_sec_msg->motor3_encoder_ticks_per_sec;
  motor4_setpoint = motor_ticks_sec_msg->motor4_encoder_ticks_per_sec;
}

/**
 * @brief Creates ROS 2 entities (node, publisher, subscription, timers, executors).
 *
 * @return true if successful.
 */
bool create_entities()
{
  allocator = rcl_get_default_allocator();

  // Create and modify init_options
  rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
  RCCHECK(rcl_init_options_init(&init_options, allocator));
  RCCHECK(rcl_init_options_set_domain_id(&init_options, Config::ROS_DOMAIN_ID));
  RCCHECK(rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator));

  // Create Node
  RCCHECK(rclc_node_init_default(&node, "vector_base_esp", "", &support));

  // Create Encoder Ticks Publisher
  RCCHECK(rclc_publisher_init_default(&encoder_publisher,
                                      &node,
                                      ROSIDL_GET_MSG_TYPE_SUPPORT(vector_interfaces, msg, EncoderTicks),
                                      "encoder_ticks"));  // Topic name

  // Create Motor Ticks/Sec Subscription
  RCCHECK(rclc_subscription_init_default(&motor_subscription,
                                         &node,
                                         ROSIDL_GET_MSG_TYPE_SUPPORT(vector_interfaces, msg, MotorTicksSec),
                                         "motor_ticks_sec"));  // Topic name

  // Create Control Timer (at 10Hz -> 1000/100)
  const unsigned int control_timer_timeout = 100;
  RCCHECK(rclc_timer_init_default(&control_timer, &support, RCL_MS_TO_NS(control_timer_timeout), control_callback));

  // Create Encoder Timer (at 20Hz -> 1000/50)
  const unsigned int encoder_timer_timeout = 50;
  RCCHECK(rclc_timer_init_default(&encoder_timer, &support, RCL_MS_TO_NS(encoder_timer_timeout), encoder_callback));

  // Create Executor ONE with 2 handles (control timer + motor subscription)
  executor_one = rclc_executor_get_zero_initialized_executor();
  RCCHECK(rclc_executor_init(&executor_one, &support.context, 2, &allocator));
  RCCHECK(rclc_executor_add_timer(&executor_one, &control_timer));
  RCCHECK(rclc_executor_add_subscription(&executor_one,
                                         &motor_subscription,
                                         &motor_ticks_sec_msg,
                                         motor_callback,
                                         ON_NEW_DATA));

  // Create Executor TWO with 1 handle (encoder timer)
  executor_two = rclc_executor_get_zero_initialized_executor();
  RCCHECK(rclc_executor_init(&executor_two, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_timer(&executor_two, &encoder_timer));

  return true;
}

/**
 * @brief Destroys ROS 2 entities.
 *
 * @return true if successful.
 */

bool destroyEntities()
{
  rmw_context_t* rmw_context = rcl_context_get_rmw_context(&support.context);
  (void)rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

  RCCHECK(rcl_publisher_fini(&encoder_publisher, &node));
  RCCHECK(rcl_subscription_fini(&motor_subscription, &node));
  RCCHECK(rcl_timer_fini(&control_timer));
  RCCHECK(rcl_timer_fini(&encoder_timer));
  rclc_executor_fini(&executor_one);
  rclc_executor_fini(&executor_two);
  RCCHECK(rcl_node_fini(&node));
  rclc_support_fini(&support);

  return true;
}

/**
 * @brief Hardware setup function.
 *
 * This function initializes the encoders and the MCPWM motor driver.
 */
void setup_hardware()
{
  encoder1.attachFullQuad(Config::ENCODER_PINS[0].A, Config::ENCODER_PINS[0].B);
  encoder2.attachFullQuad(Config::ENCODER_PINS[1].A, Config::ENCODER_PINS[1].B);
  encoder3.attachFullQuad(Config::ENCODER_PINS[2].A, Config::ENCODER_PINS[2].B);
  encoder4.attachFullQuad(Config::ENCODER_PINS[3].A, Config::ENCODER_PINS[3].B);

  encoder1.setCount(0);
  encoder2.setCount(0);
  encoder3.setCount(0);
  encoder4.setCount(0);

  setupMCPWM();
}

/**
 * @brief Error loop function.
 *
 * This function is called if an error is encountered during initialization.
 * It flashes the onboard LED indefinitely.
 */
void rclErrorLoop()
{
  while(true) {
    flashLED(2);
  }
}

void flashLED(int n_times)
{
  for(int i = 0; i < n_times; i++) {
    digitalWrite(LED_BUILTIN, HIGH);
    delay(150);
    digitalWrite(LED_BUILTIN, LOW);
    delay(150);
  }
  delay(1000);
}

void setup()
{
  pinMode(LED_BUILTIN, OUTPUT);
  flashLED(3);

  Serial.begin(115200);

#ifdef USE_WIFI_TRANSPORT
  // Wifi Transport initialization
  IPAddress agent_ip = Config::WIFI_CONFIG.getAgentIP();
  set_microros_wifi_transport(Config::WIFI_CONFIG.ssid,
                              Config::WIFI_CONFIG.password,
                              agent_ip,
                              Config::WIFI_CONFIG.port);

#elif defined(USE_SERIAL_TRANSPORT)
  // Serial Transport initialization
  set_microros_serial_transports(Serial);

#else
#error " No Transport defined! Please define USE_WIFI_TRANSPORT or USE_SERIAL_TRANSPORT"
#endif

  setup_hardware();
  state = WAITING_AGENT;
}

void loop()
{
  switch(state) {
    case WAITING_AGENT:
      EXECUTE_EVERY_N_MS(500, state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_AVAILABLE : WAITING_AGENT;);
      break;

    case AGENT_AVAILABLE:
      state = (true == create_entities()) ? AGENT_CONNECTED : WAITING_AGENT;
      if(state == WAITING_AGENT) {
        destroyEntities();
      }
      break;

    case AGENT_CONNECTED:
      EXECUTE_EVERY_N_MS(200,
                         state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_CONNECTED : AGENT_DISCONNECTED;);
      if(state == AGENT_CONNECTED) {
        rclc_executor_spin_some(&executor_one, RCL_MS_TO_NS(10));
        rclc_executor_spin_some(&executor_two, RCL_MS_TO_NS(10));
      }
      break;

    case AGENT_DISCONNECTED:
      destroyEntities();
      state = WAITING_AGENT;
      break;
    default:
      break;
  }
}