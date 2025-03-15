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
  {22, 23},  // Motor 1 (D22 - MD1Dir, D23 - MD1pwm)
  {17, 18},  // Motor 2 (D17 - MD2dir, D18 - MD2pwm)
  {19, 21},  // Motor 3 (D19 - MD3dir, D21 - MD3pwm)
  {4, 16},   // Motor 4 (D4 - MD4dir, D16 - MD4pwm)
};

//. ENCODER PINS: Encoder has two pins(A and B)
struct EncoderPins
{
  int A;
  int B;
};

static constexpr EncoderPins ENCODER_PINS[4] = {
  {14, 27},  // Encoder 1 (D14 - Enc1A, D27 - Enc1B)
  {26, 25},  // Encoder 2 (D26 - Enc2A, D25 - Enc2B)
  {33, 32},  // Encoder 3 (D33 - Enc3A, D32 - Enc3B)
  {35, 34}   // Encoder 4 (D35 - Enc4A, D34 - Enc4B)
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
  {1.0f, 0.01f, 0.1f, 0.9f},   // Motor 1
  {1.2f, 0.015f, 0.1f, 0.9f},  // Motor 2
  {1.3f, 0.016f, 0.1f, 0.9f},  // Motor 3
  {1.4f, 0.014f, 0.1f, 0.9f}   // Motor 4
};

//. WIFI Configuration Parameters
struct WifiConfig
{
  const char * ssid;     /**< Wifi network SSID */
  const char * password; /**< Wifi network password */
  uint8_t ip[4];         /**< Agent IP address represented as four octets */
  size_t port;           /**< Agent port number */

  inline IPAddress getAgentIP() const
  {
    return IPAddress(ip[0], ip[1], ip[2], ip[3]);
  }
};

static constexpr WifiConfig WIFI_CONFIG = {
  "adyansh_sakar",  // SSID
  "12345678",       // Password
  {10, 42, 0, 1},   // IP
  8888              // Port
};

//. Other Parameters
static constexpr int ROS_DOMAIN_ID = 20;

}  // namespace Config

#endif  // CONFIGURATIONS_HPP_
