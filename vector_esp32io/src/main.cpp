#include <Arduino.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>

#include "button.h"
#include "configuration.h"
#include "micro_ros_platformio.h"
#include "relay.h"
#include "vector_interfaces/msg/led_states.h"
#include "vector_interfaces/msg/push_button_states.h"

// . Macro Definations
/**
 * @brief Execute a function and abort on error
 *
 * Calls the expression and enters error loop if result isn't RCL_RET_OK
 */
#define RCCHECK(fn)                                                                                \
  {                                                                                                \
    rcl_ret_t temp_rc = fn;                                                                        \
    if ((temp_rc != RCL_RET_OK)) {                                                                 \
      rclErrorLoop();                                                                              \
    }                                                                                              \
  }

/**
 * @brief Execute a function with soft error checking
 *
 * Calls the expression without action if result isn't RCL_RET_OK
 */
#define RCSOFTCHECK(fn)                                                                            \
  {                                                                                                \
    rcl_ret_t temp_rc = fn;                                                                        \
    if ((temp_rc != RCL_RET_OK)) {                                                                 \
    }                                                                                              \
  }

/**
 * @brief Executes code block every specified milliseconds
 *
 * Runs the given code X only if the elapsed time is greater than MS.
 */
#define EXECUTE_EVERY_N_MS(MS, X)                                                                  \
  do {                                                                                             \
    static volatile int64_t init = -1;                                                             \
    if (init == -1) {                                                                              \
      init = uxr_millis();                                                                         \
    }                                                                                              \
    if (uxr_millis() - init > MS) {                                                                \
      X;                                                                                           \
      init = uxr_millis();                                                                         \
    }                                                                                              \
  } while (0)

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
  for (int i = 0; i < n_times; i++) {
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
  while (true) {
    flashLED(2);
  }
}
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
} state;

//* URos support structures
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rclc_executor_t executor_one;

//* Message types
vector_interfaces__msg__LedStates led_states_msg;
vector_interfaces__msg__PushButtonStates push_button_states_msg;

//* ROS 2 communication objects
rcl_publisher_t push_button_states_publisher;
rcl_subscription_t led_states_subscription;
rcl_timer_t push_button_states_timer;

// Relay - LEDs
Relay redLed(Config::relayPin1);
Relay greenLed(Config::relayPin2);
Relay orangeLed(Config::relayPin3);

// Push buttons
button button1(Config::buttonPin1);
button button2(Config::buttonPin2);

// Previous LED states
volatile bool prev_red_led_state    = false;
volatile bool prev_green_led_state  = false;
volatile bool prev_orange_led_state = false;

void led_states_callback(const void * msgin)
{
  const vector_interfaces__msg__LedStates * led_states_msg =
    (const vector_interfaces__msg__LedStates *)msgin;

  //
  bool red    = led_states_msg->red_led;
  bool green  = led_states_msg->green_led;
  bool orange = led_states_msg->orange_led;

  (prev_red_led_state != red) ? (red ? redLed.turnOn() : redLed.turnOff()) : (void)0;
  (prev_green_led_state != green) ? (green ? greenLed.turnOn() : greenLed.turnOff()) : (void)0;
  (prev_orange_led_state != orange) ? (orange ? orangeLed.turnOn() : orangeLed.turnOff()) : (void)0;

  // Update previous LED states
  prev_red_led_state    = red;
  prev_green_led_state  = green;
  prev_orange_led_state = orange;
}

void push_button_states_timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{
  RCLC_UNUSED(last_call_time);
  if (timer != NULL) {
    // Populate the push button states message
    push_button_states_msg.button1 = !button1.getState();
    push_button_states_msg.button2 = !button2.getState();

    // Publish the push button states message
    RCSOFTCHECK(rcl_publish(&push_button_states_publisher, &push_button_states_msg, NULL));
  }
}

void hardware_init()
{
  // Initialize the onboard LED
  pinMode(LED_BUILTIN, OUTPUT);

  // Initialize the relay pins
  redLed.begin();
  greenLed.begin();
  orangeLed.begin();

  // Initialize the push button pins
  button1.setDebounceTime(50);
  button2.setDebounceTime(50);
}

bool create_entities()
{
  allocator = rcl_get_default_allocator();

  // Create and modify init_options
  rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
  RCCHECK(rcl_init_options_init(&init_options, allocator));
  RCCHECK(rcl_init_options_set_domain_id(&init_options, Config::ROS_DOMAIN_ID));
  RCCHECK(rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator));

  // Create Node
  RCCHECK(rclc_node_init_default(&node, "vector_io_esp32", "", &support));

  // Create Push Button States Publisher
  RCCHECK(rclc_publisher_init_default(
    &push_button_states_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(vector_interfaces, msg, PushButtonStates),
    "push_button_states"));  // Topic name

  // Create LED States Subscription
  RCCHECK(
    rclc_subscription_init_default(&led_states_subscription,
                                   &node,
                                   ROSIDL_GET_MSG_TYPE_SUPPORT(vector_interfaces, msg, LedStates),
                                   "led_states"));  // Topic name

  // Create Push Button States Timer (at 20Hz -> 1000/50)
  const unsigned int push_button_states_timer_timeout = 50;
  RCCHECK(rclc_timer_init_default(&push_button_states_timer,
                                  &support,
                                  RCL_MS_TO_NS(push_button_states_timer_timeout),
                                  push_button_states_timer_callback));

  // Create Executor ONE with 2 handles (push button states timer + led states subscription)
  executor_one = rclc_executor_get_zero_initialized_executor();
  RCCHECK(rclc_executor_init(&executor_one, &support.context, 2, &allocator));
  RCCHECK(rclc_executor_add_timer(&executor_one, &push_button_states_timer));
  RCCHECK(rclc_executor_add_subscription(&executor_one,
                                         &led_states_subscription,
                                         &led_states_msg,
                                         led_states_callback,
                                         ON_NEW_DATA));

  return true;
}

bool destroyEntities()
{
  rmw_context_t * rmw_context = rcl_context_get_rmw_context(&support.context);
  (void)rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

  rclc_executor_fini(&executor_one);
  RCCHECK(rcl_publisher_fini(&push_button_states_publisher, &node));
  RCCHECK(rcl_subscription_fini(&led_states_subscription, &node));
  RCCHECK(rcl_timer_fini(&push_button_states_timer));
  RCCHECK(rcl_node_fini(&node));
  rclc_support_fini(&support);

  return true;
}

void setup()
{
  pinMode(LED_BUILTIN, OUTPUT);
  Serial.begin(115200);

  Serial.println("Starting Vector Base ESP32...");

#ifdef USE_WIFI_TRANSPORT
  // Wifi Transport initialization
  IPAddress agent_ip = Config::WIFI_CONFIG.getAgentIP();
  set_microros_wifi_transports(Config::WIFI_CONFIG.ssid,
                               Config::WIFI_CONFIG.password,
                               agent_ip,
                               Config::WIFI_CONFIG.port);

#elif defined(USE_SERIAL_TRANSPORT)
  // Serial Transport initialization
  set_microros_serial_transports(Serial);
#else
#error " No Transport defined! Please define USE_WIFI_TRANSPORT or USE_SERIAL_TRANSPORT"
#endif
  flashLED(3);
  hardware_init();
  state = WAITING_AGENT;
  flashLED(5);
}

void loop()
{
  switch (state) {
    case WAITING_AGENT:
      EXECUTE_EVERY_N_MS(500,
                         state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_AVAILABLE :
                                                                               WAITING_AGENT;);
      break;

    case AGENT_AVAILABLE:
      state = (true == create_entities()) ? AGENT_CONNECTED : WAITING_AGENT;

      if (state == WAITING_AGENT) {
        destroyEntities();
      }
      break;

    case AGENT_CONNECTED:
      EXECUTE_EVERY_N_MS(200,
                         state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_CONNECTED :
                                                                               AGENT_DISCONNECTED;);
      if (state == AGENT_CONNECTED) {
        // Buttons Loop
        button1.loop();
        button2.loop();

        rclc_executor_spin_some(&executor_one, RCL_MS_TO_NS(10));
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
