#include "relay.h"

Relay::Relay(int pin)
  : relayPin(pin)
  , state(false)
{}

void Relay::begin()
{
  pinMode(relayPin, OUTPUT);
  digitalWrite(relayPin, LOW);  // Ensure the relay is OFF at start.
}

void Relay::turnOn()
{
  digitalWrite(relayPin, HIGH);
  state = true;
}

void Relay::turnOff()
{
  digitalWrite(relayPin, LOW);
  state = false;
}

void Relay::toggle()
{
  if (state)
    turnOff();
  else
    turnOn();
}

bool Relay::getState() const
{
  return state;
}