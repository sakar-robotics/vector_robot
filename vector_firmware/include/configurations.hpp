// Copyright (c) 2025 Sakar Robotics

#ifndef CONFIGURATIONS_HPP_
#define CONFIGURATIONS_HPP_

namespace Config
{

//. MOTOR DRIVE PINS: direction and PWM pins
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

//. ENCODER PINS: Encoder has two pins(A and B)
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
  { 1.0f, 0.01f, 0.1f, 0.9f },  // Motor 1
  { 1.0f, 0.01f, 0.1f, 0.9f },  // Motor 2
  { 1.0f, 0.01f, 0.1f, 0.9f },  // Motor 3
  { 1.0f, 0.01f, 0.1f, 0.9f }   // Motor 4
};

//. WIFI Configuration Parameters
struct WifiConfig
{
  const char* ssid;     /**< Wifi network SSID */
  const char* password; /**< Wifi network password */
  uint8_t ip[4];        /**< Agent IP address represented as four octets */
  size_t port;          /**< Agent port number */

  inline IPAddress getAgentIP() const
  {
    return IPAddress(ip[0], ip[1], ip[2], ip[3]);
  }
};

static constexpr WifiConfig WIFI_CONFIG = {
  "sakar_wifi_ssid",      // SSID
  "sakar_wifi_password",  // Password
  { 10, 42, 0, 1 },       // IP
  8888                    // Port
};

//. Other Parameters
static constexpr int ROS_DOMAIN_ID = 20;

}  // namespace Config

#endif  // CONFIGURATIONS_HPP_
