#ifndef CONFIGURATION_H
#define CONFIGURATION_H

namespace Config
{
// Relay pins
constexpr int relayPin1 = 23;
constexpr int relayPin2 = 19;
constexpr int relayPin3 = 18;
constexpr int relayPin4 = 17;

// Button pins
constexpr int buttonPin1 = 4;
constexpr int buttonPin2 = 16;

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

static constexpr int ROS_DOMAIN_ID = 20;

// Uncomment one of the following transport definations as needed
// #define USE_WIFI_TRANSPORT
#define USE_SERIAL_TRANSPORT

}  // namespace Config

#endif  // CONFIGURATION_H