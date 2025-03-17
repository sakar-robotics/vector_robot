#ifndef _BUTTON_H_
#define _BUTTON_H_

#include <Arduino.h>

#define COUNT_FALLING 0
#define COUNT_RISING 1
#define COUNT_BOTH 2

// Button wiring modes
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
  int buttonPin;                // GPIO pin connected to the button
  unsigned long debounceDelay;  // Duration (in ms) to filter out switch bounce
  unsigned long pressCount;     // Number of validated button press events
  int countingMode;             // Number of validated button press events
  int buttonPressedState;       // Logical state indicating the button is pressed
  int buttonUnpressedState;     // Logical state indicating the button is not pressed

  int prevSteadyState;     // Last stable (debounced) state recorded
  int currentSteadyState;  // Current stable state based on debouncing
  int lastBouncyState;     // Last raw reading (may include bounce artifacts)

  unsigned long lastDebounceCheck;  // Timestamp of the most recent state change

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