#ifndef _RELAY_H_
#define _RELAY_H_

#include <Arduino.h>

class Relay
{
private:
  int relayPin;  // GPIO pin connected to the relay
  bool state;    // Current state of the relay (true = ON, false = OFF)

public:
  // Constructor: Sets the relay pin.
  Relay(int pin);

  // Initializes the relay pin as an OUTPUT and sets the relay to OFF.
  void begin();

  // Turns the relay on.
  void turnOn();

  // Turns the relay off.
  void turnOff();

  // Toggles the relay state.
  void toggle();

  // Returns the current state of the relay.
  bool getState() const;
};

#endif  // _RELAY_H_