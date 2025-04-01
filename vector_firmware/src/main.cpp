// Copyright (c) 2025 Sakar Robotics

#include <Arduino.h>
#include <micro_ros_platformio.h>

#include "configurations.hpp"
#include "uros_core.hpp"

void setup()
{
  pinMode(LED_BUILTIN, OUTPUT);
  Serial.begin(115200);
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

  setup_hardware();
  state = WAITING_AGENT;
  motorStop(1);
  motorStop(2);
  motorStop(3);
  motorStop(4);
  flashLED(5);
}

void loop()
{
  switch (state) {
    case WAITING_AGENT:
      EXECUTE_EVERY_N_MS(500,
                         state = (RMW_RET_OK == rmw_uros_ping_agent(200, 1)) ? AGENT_AVAILABLE :
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

// void loop()
// {
//   motorSetSpeed(1, 10);
//   motorSetSpeed(2, 10);
//   motorSetSpeed(3, -10);
//   motorSetSpeed(4, -10);
//   delay(100);
// }
