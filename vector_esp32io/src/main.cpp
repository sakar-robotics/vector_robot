#include <Arduino.h>

#include "button.h"
const int buttonPin = 4;

button p1(buttonPin);

void setup()
{
  Serial.begin(115200);
  p1.setDebounceTime(50);
}

void loop()
{
  p1.loop();

  if (p1.isPressed()) {
    Serial.println("Button pressed");
  } else if (p1.isReleased()) {
    Serial.println("Button released");
  }
}