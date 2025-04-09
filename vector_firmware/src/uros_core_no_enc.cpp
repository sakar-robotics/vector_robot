#include "uros_core.hpp"

#define max_motor_ticks_sec 3333

//* URos support structures
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rclc_parameter_server_t param_server;
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

// Global Encoder Instances
ESP32Encoder encoder1;
ESP32Encoder encoder2;
ESP32Encoder encoder3;
ESP32Encoder encoder4;

// PID Controllers instances for four motors (100ms)
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

// Global Variables
volatile float motor1_setpoint = 0.0f;
volatile float motor2_setpoint = 0.0f;
volatile float motor3_setpoint = 0.0f;
volatile float motor4_setpoint = 0.0f;
u_int64_t last_encoder_read_time;
int64_t lastCount1 = 0;
int64_t lastCount2 = 0;
int64_t lastCount3 = 0;
int64_t lastCount4 = 0;

enum states state = WAITING_AGENT;

void control_callback(rcl_timer_t * timer, int64_t last_call_time)
{
  RCLC_UNUSED(last_call_time);

  if (timer != NULL) {
    // uint64_t now           = millis();
    // float dt               = (now - last_encoder_read_time) / 1000.0f;  // Seconds
    // last_encoder_read_time = now;

    // // Read current encoder counts directly from the encoder instances
    // int64_t current_encoder1 = encoder1.getCount();
    // int64_t current_encoder2 = encoder2.getCount();
    // int64_t current_encoder3 = encoder3.getCount();
    // int64_t current_encoder4 = encoder4.getCount();

    // // Calculate encoder ticks/sec
    // float motor1_ticks_per_sec = (current_encoder1 - lastCount1) / dt;
    // float motor2_ticks_per_sec = (current_encoder2 - lastCount2) / dt;
    // float motor3_ticks_per_sec = (current_encoder3 - lastCount3) / dt;
    // float motor4_ticks_per_sec = (current_encoder4 - lastCount4) / dt;

    // // Compute PID outputs for each motor
    // float motor1_pwm = pidMotor1.compute(motor1_setpoint, motor1_ticks_per_sec);
    // float motor2_pwm = pidMotor2.compute(motor2_setpoint, motor2_ticks_per_sec);
    // float motor3_pwm = pidMotor3.compute(motor3_setpoint, motor3_ticks_per_sec);
    // float motor4_pwm = pidMotor4.compute(motor4_setpoint, motor4_ticks_per_sec);

    // // Set the motor speeds
    // motorSetSpeed(1, motor1_pwm);
    // motorSetSpeed(2, motor2_pwm);
    // motorSetSpeed(3, motor3_pwm);
    // motorSetSpeed(4, motor4_pwm);

    // // Store current encoder counts for the next iteration
    // lastCount1 = current_encoder1;
    // lastCount2 = current_encoder2;
    // lastCount3 = current_encoder3;
    // lastCount4 = current_encoder4;
  }
}

void encoder_callback(rcl_timer_t * timer, int64_t last_call_time)
{
  RCLC_UNUSED(last_call_time);
  if (timer != NULL) {
    // Populate the encoder ticks message
    // encoder_ticks_msg.motor1_encoder_ticks = encoder1.getCount();
    // encoder_ticks_msg.motor2_encoder_ticks = encoder2.getCount();
    // encoder_ticks_msg.motor3_encoder_ticks = encoder3.getCount();
    // encoder_ticks_msg.motor4_encoder_ticks = encoder4.getCount();

    // Publish the encoder ticks message
    RCSOFTCHECK(rcl_publish(&encoder_publisher, &encoder_ticks_msg, NULL));
  }
}

void motor_callback(const void * msgin)
{
  const vector_interfaces__msg__MotorTicksSec * motor_ticks_sec_msg =
    (const vector_interfaces__msg__MotorTicksSec *)msgin;

  //   motor1_setpoint = motor_ticks_sec_msg->motor1_encoder_ticks_per_sec;
  //   motor2_setpoint = motor_ticks_sec_msg->motor2_encoder_ticks_per_sec;
  //   motor3_setpoint = motor_ticks_sec_msg->motor3_encoder_ticks_per_sec;
  //   motor4_setpoint = motor_ticks_sec_msg->motor4_encoder_ticks_per_sec;

  // Set motor speeds directly without encoder and PID
  // Interpolate the motor ticks/sec to PWM values using map function

  int motor1_ticks_sec = motor_ticks_sec_msg->motor1_encoder_ticks_per_sec;
  int motor2_ticks_sec = motor_ticks_sec_msg->motor2_encoder_ticks_per_sec;
  int motor3_ticks_sec = motor_ticks_sec_msg->motor3_encoder_ticks_per_sec;
  int motor4_ticks_sec = motor_ticks_sec_msg->motor4_encoder_ticks_per_sec;

  // Use absolute values to map to PWM send negative pwm if motor ticks is negative but map only in
  // 0 -100
  float motor1_pwm =
    map(abs(motor1_ticks_sec), 0, max_motor_ticks_sec, 0, 100) * (motor1_ticks_sec < 0 ? -1 : 1);

  float motor2_pwm =
    map(abs(motor2_ticks_sec), 0, max_motor_ticks_sec, 0, 100) * (motor2_ticks_sec < 0 ? -1 : 1);

  float motor3_pwm =
    map(abs(motor3_ticks_sec), 0, max_motor_ticks_sec, 0, 100) * (motor3_ticks_sec < 0 ? -1 : 1);

  float motor4_pwm =
    map(abs(motor4_ticks_sec), 0, max_motor_ticks_sec, 0, 100) * (motor4_ticks_sec < 0 ? -1 : 1);

  Serial.println("Motor 1 PWM: " + String(motor1_pwm) + " Motor 2 PWM: " + String(motor2_pwm) +
                 " Motor 3 PWM: " + String(motor3_pwm) + " Motor 4 PWM: " + String(motor4_pwm));

  motorSetSpeed(1, motor1_pwm);
  motorSetSpeed(2, motor2_pwm);
  motorSetSpeed(3, motor3_pwm);
  motorSetSpeed(4, motor4_pwm);

  //   motorSetSpeed(1,
  //                 map(motor_ticks_sec_msg->motor1_encoder_ticks_per_sec,
  //                     -max_motor_ticks_sec,
  //                     max_motor_ticks_sec,
  //                     -100,
  //                     100));
  //   motorSetSpeed(2,
  //                 map(motor_ticks_sec_msg->motor2_encoder_ticks_per_sec,
  //                     -max_motor_ticks_sec,
  //                     max_motor_ticks_sec,
  //                     -100,
  //                     100));
  //   motorSetSpeed(3,
  //                 map(motor_ticks_sec_msg->motor3_encoder_ticks_per_sec,
  //                     -max_motor_ticks_sec,
  //                     max_motor_ticks_sec,
  //                     -100,
  //                     100));
  //   motorSetSpeed(4,
  //                 map(motor_ticks_sec_msg->motor4_encoder_ticks_per_sec,
  //                     -max_motor_ticks_sec,
  //                     max_motor_ticks_sec,
  //                     -100,
  //                     100));
}

bool param_callback(const Parameter * old_param, const Parameter * new_param, void * context)
{
  RCLC_UNUSED(context);

  // Check if an existing parameter is updated
  if (old_param != NULL && new_param != NULL) {
    // Expect parameter name in format: "<param>_motor<index>"
    // For example: "Kp_motor1", "Ki_motor2", "Kd_motor3"
    char param_type[3] = {0};
    int motor_index    = 0;
    if (sscanf(new_param->name.data, "%2[^_]_motor%d", param_type, &motor_index) != 2) {
      return false;
    }

    // Create an array of PIDController pointers for easier access
    PIDController * pidControllers[4] = {&pidMotor1, &pidMotor2, &pidMotor3, &pidMotor4};

    if (motor_index < 1 || motor_index > 4) {
      return false;
    }

    PIDController * controller = pidControllers[motor_index - 1];

    // Fetch the new_value (convert to float)
    float new_val = static_cast<float>(new_param->value.double_value);

    // Update the appropriate tuning
    if (strcmp(param_type, "Kp") == 0) {
      controller->setTunings(new_val, controller->getKi(), controller->getKd());
    } else if (strcmp(param_type, "Ki") == 0) {
      controller->setTunings(controller->getKp(), new_val, controller->getKd());
    } else if (strcmp(param_type, "Kd") == 0) {
      controller->setTunings(controller->getKp(), controller->getKi(), new_val);
    } else {
      return false;
    }

    // Log Kp,Ki,Kd of all controllers
    // Serial.println("Kp_motor1: " + String(pidMotor1.getKp()) + " Ki_motor1: " +
    // String(pidMotor1.getKi()) +
    //                " Kd_motor1: " + String(pidMotor1.getKd()));
    // Serial.println("Kp_motor2: " + String(pidMotor2.getKp()) + " Ki_motor2: " +
    // String(pidMotor2.getKi()) +
    //                " Kd_motor2: " + String(pidMotor2.getKd()));
    // Serial.println("Kp_motor3: " + String(pidMotor3.getKp()) + " Ki_motor3: " +
    // String(pidMotor3.getKi()) +
    //                " Kd_motor3: " + String(pidMotor3.getKd()));
    // Serial.println("Kp_motor4: " + String(pidMotor4.getKp()) + " Ki_motor4: " +
    // String(pidMotor4.getKi()) +
    //                " Kd_motor4: " + String(pidMotor4.getKd()));

    return true;
  } else  //! Reject loading of new parameters or deletion of existing parameters
    return false;
}

bool create_entities()
{
  flashLED(4);
  allocator = rcl_get_default_allocator();

  // Create and modify init_options
  rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
  RCCHECK(rcl_init_options_init(&init_options, allocator));
  RCCHECK(rcl_init_options_set_domain_id(&init_options, Config::ROS_DOMAIN_ID));
  RCCHECK(rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator));

  // Create Node
  RCCHECK(rclc_node_init_default(&node, "vector_base_esp", "", &support));

  // Create Parameter Server
  const rclc_parameter_options_t options = {.notify_changed_over_dds     = true,
                                            .max_params                  = 12,
                                            .allow_undeclared_parameters = false,
                                            .low_mem_mode                = false};
  RCCHECK(rclc_parameter_server_init_with_option(&param_server, &node, &options));

  // Create Encoder Ticks Publisher
  RCCHECK(
    rclc_publisher_init_default(&encoder_publisher,
                                &node,
                                ROSIDL_GET_MSG_TYPE_SUPPORT(vector_interfaces, msg, EncoderTicks),
                                "encoder_ticks"));  // Topic name

  // Create Motor Ticks/Sec Subscription
  RCCHECK(rclc_subscription_init_default(
    &motor_subscription,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(vector_interfaces, msg, MotorTicksSec),
    "motor_ticks_sec"));  // Topic name

  // Create Control Timer (at 10Hz -> 1000/100)
  const unsigned int control_timer_timeout = 100;
  RCCHECK(rclc_timer_init_default(&control_timer,
                                  &support,
                                  RCL_MS_TO_NS(control_timer_timeout),
                                  control_callback));

  // Create Encoder Timer (at 20Hz -> 1000/50)
  const unsigned int encoder_timer_timeout = 50;
  RCCHECK(rclc_timer_init_default(&encoder_timer,
                                  &support,
                                  RCL_MS_TO_NS(encoder_timer_timeout),
                                  encoder_callback));

  // Create Executor ONE with 2 handles (control timer + motor subscription)
  executor_one = rclc_executor_get_zero_initialized_executor();
  RCCHECK(rclc_executor_init(&executor_one, &support.context, 2, &allocator));
  RCCHECK(rclc_executor_add_timer(&executor_one, &control_timer));
  RCCHECK(rclc_executor_add_subscription(&executor_one,
                                         &motor_subscription,
                                         &motor_ticks_sec_msg,
                                         motor_callback,
                                         ON_NEW_DATA));

  // Create Executor TWO with 1 + (param_no) handles (encoder timer + parameter server)
  executor_two = rclc_executor_get_zero_initialized_executor();
  RCCHECK(rclc_executor_init(&executor_two,
                             &support.context,
                             1 + RCLC_EXECUTOR_PARAMETER_SERVER_HANDLES,
                             &allocator));  // 1 + 6 handles for parameter server
  RCCHECK(rclc_executor_add_parameter_server(&executor_two, &param_server, param_callback));
  RCCHECK(rclc_executor_add_timer(&executor_two, &encoder_timer));

  // Add parameters to the server
  RCCHECK(rclc_add_parameter(&param_server, "Kp_motor1", RCLC_PARAMETER_DOUBLE));
  RCCHECK(rclc_add_parameter(&param_server, "Ki_motor1", RCLC_PARAMETER_DOUBLE));
  RCCHECK(rclc_add_parameter(&param_server, "Kd_motor1", RCLC_PARAMETER_DOUBLE));

  RCCHECK(rclc_add_parameter(&param_server, "Kp_motor2", RCLC_PARAMETER_DOUBLE));
  RCCHECK(rclc_add_parameter(&param_server, "Ki_motor2", RCLC_PARAMETER_DOUBLE));
  RCCHECK(rclc_add_parameter(&param_server, "Kd_motor2", RCLC_PARAMETER_DOUBLE));

  RCCHECK(rclc_add_parameter(&param_server, "Kp_motor3", RCLC_PARAMETER_DOUBLE));
  RCCHECK(rclc_add_parameter(&param_server, "Ki_motor3", RCLC_PARAMETER_DOUBLE));
  RCCHECK(rclc_add_parameter(&param_server, "Kd_motor3", RCLC_PARAMETER_DOUBLE));

  RCCHECK(rclc_add_parameter(&param_server, "Kp_motor4", RCLC_PARAMETER_DOUBLE));
  RCCHECK(rclc_add_parameter(&param_server, "Ki_motor4", RCLC_PARAMETER_DOUBLE));
  RCCHECK(rclc_add_parameter(&param_server, "Kd_motor4", RCLC_PARAMETER_DOUBLE));

  // Set parameter default values.
  RCCHECK(rclc_parameter_set_double(&param_server, "Kp_motor1", Config::PID_PARAMS[0].Kp));
  RCCHECK(rclc_parameter_set_double(&param_server, "Ki_motor1", Config::PID_PARAMS[0].Ki));
  RCCHECK(rclc_parameter_set_double(&param_server, "Kd_motor1", Config::PID_PARAMS[0].Kd));

  RCCHECK(rclc_parameter_set_double(&param_server, "Kp_motor2", Config::PID_PARAMS[1].Kp));
  RCCHECK(rclc_parameter_set_double(&param_server, "Ki_motor2", Config::PID_PARAMS[1].Ki));
  RCCHECK(rclc_parameter_set_double(&param_server, "Kd_motor2", Config::PID_PARAMS[1].Kd));

  RCCHECK(rclc_parameter_set_double(&param_server, "Kp_motor3", Config::PID_PARAMS[2].Kp));
  RCCHECK(rclc_parameter_set_double(&param_server, "Ki_motor3", Config::PID_PARAMS[2].Ki));
  RCCHECK(rclc_parameter_set_double(&param_server, "Kd_motor3", Config::PID_PARAMS[2].Kd));

  RCCHECK(rclc_parameter_set_double(&param_server, "Kp_motor4", Config::PID_PARAMS[3].Kp));
  RCCHECK(rclc_parameter_set_double(&param_server, "Ki_motor4", Config::PID_PARAMS[3].Ki));
  RCCHECK(rclc_parameter_set_double(&param_server, "Kd_motor4", Config::PID_PARAMS[3].Kd));

  return true;
}

bool destroyEntities()
{
  flashLED(5);
  rmw_context_t * rmw_context = rcl_context_get_rmw_context(&support.context);
  (void)rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

  rclc_executor_fini(&executor_one);
  rclc_executor_fini(&executor_two);
  RCCHECK(rcl_publisher_fini(&encoder_publisher, &node));
  RCCHECK(rcl_subscription_fini(&motor_subscription, &node));
  RCCHECK(rcl_timer_fini(&control_timer));
  RCCHECK(rcl_timer_fini(&encoder_timer));
  RCCHECK(rcl_node_fini(&node));
  rclc_support_fini(&support);

  return true;
}

void setup_hardware()
{
  //   encoder1.attachFullQuad(Config::ENCODER_PINS[0].A, Config::ENCODER_PINS[0].B);
  //   encoder2.attachFullQuad(Config::ENCODER_PINS[1].A, Config::ENCODER_PINS[1].B);
  //   encoder3.attachFullQuad(Config::ENCODER_PINS[2].A, Config::ENCODER_PINS[2].B);
  //   encoder4.attachFullQuad(Config::ENCODER_PINS[3].A, Config::ENCODER_PINS[3].B);

  //   encoder1.setCount(0);
  //   encoder2.setCount(0);
  //   encoder3.setCount(0);
  //   encoder4.setCount(0);

  setupMCPWM();
}
