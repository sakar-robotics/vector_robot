#ifndef _BUTTON_H_
#define _BUTTON_H_

#include <Arduino.h>

#define COUNT_FALLING 0
#define COUNT_RISING 1
#define COUNT_BOTH 2

// Constants for button modes
#define INTERNAL_PULLUP INPUT_PULLUP
#ifdef INPUT_PULLDOWN
#define INTERNAL_PULLDOWN INPUT_PULLDOWN
#else
#define INTERNAL_PULLDOWN INPUT
#endif

#define EXTERNAL_PULLUP 0xFE
#define EXTERNAL_PULLDOWN 0xFF

class button
{
private:
  int buttonPin;
  unsigned long debounceDelay;
  unsigned long pressCount;
  int countingMode;
  int buttonPressedState;    // the state when the button is considered pressed
  int buttonUnpressedState;  // the state when the button is considered unpressed

  int prevSteadyState;     // the previous steady state from the input pin, used to detect pressed
                           // and released event
  int currentSteadyState;  // the last steady state from the input pin
  int lastBouncyState;     // the last flickerable state from the input pin

  unsigned long lastDebounceCheck;  // the last time the output pin was toggled

public:
  button(int pin);
  button(int pin, int mode);
  void setDebounceTime(unsigned long time);
  int getState(void);
  int getStateRaw(void);
  bool isPressed(void);
  bool isReleased(void);
  void setCountMode(int mode);
  unsigned long getCount(void);
  void resetCount(void);
  void loop(void);
};

#endif  // _BUTTON_H_