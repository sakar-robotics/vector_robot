#include <button.h>

button::button(int pin)
  : button(pin, INTERNAL_PULLUP) {};

button::button(int pin, int mode)
{
  buttonPin     = pin;
  debounceDelay = 0;
  pressCount    = 0;
  countingMode  = COUNT_FALLING;

  if (mode == INTERNAL_PULLUP || mode == INTERNAL_PULLDOWN) {
    pinMode(buttonPin, mode);
  } else if (mode == EXTERNAL_PULLUP || mode == EXTERNAL_PULLDOWN) {
    pinMode(buttonPin, INPUT);  // External pull-up/pull-down, set as INPUT
  }

  // Set the pressed and unpressed states based on the mode
  if (mode == INTERNAL_PULLDOWN || mode == EXTERNAL_PULLDOWN) {
    buttonPressedState   = HIGH;
    buttonUnpressedState = LOW;
  } else {
    buttonPressedState   = LOW;
    buttonUnpressedState = HIGH;
  }

  prevSteadyState    = digitalRead(buttonPin);
  currentSteadyState = prevSteadyState;
  lastBouncyState    = prevSteadyState;

  lastDebounceCheck = 0;
}

void button::setDebounceTime(unsigned long time)
{
  debounceDelay = time;
}

int button::getState(void)
{
  return currentSteadyState;
}

int button::getStateRaw(void)
{
  return digitalRead(buttonPin);
}

bool button::isPressed(void)
{
  if (prevSteadyState == buttonUnpressedState && currentSteadyState == buttonPressedState)
    return true;
  else
    return false;
}

bool button::isReleased(void)
{
  if (prevSteadyState == buttonPressedState && currentSteadyState == buttonUnpressedState)
    return true;
  else
    return false;
}

void button::setCountMode(int mode)
{
  countingMode = mode;
}

unsigned long button::getCount(void)
{
  return pressCount;
}

void button::resetCount(void)
{
  pressCount = 0;
}

void button::loop(void)
{
  // read the state of the switch/button:
  int currentState          = digitalRead(buttonPin);
  unsigned long currentTime = millis();

  // check to see if you just pressed the button
  // (i.e. the input went from LOW to HIGH), and you've waited long enough
  // since the last press to ignore any noise:

  // If the switch/button changed, due to noise or pressing:
  if (currentState != lastBouncyState) {
    // reset the debouncing timer
    lastDebounceCheck = currentTime;
    // save the the last flickerable state
    lastBouncyState = currentState;
  }

  if ((currentTime - lastDebounceCheck) >= debounceDelay) {
    // whatever the reading is at, it's been there for longer than the debounce
    // delay, so take it as the actual current state:

    // save the the steady state
    prevSteadyState    = currentSteadyState;
    currentSteadyState = currentState;
  }

  if (prevSteadyState != currentSteadyState) {
    if (countingMode == COUNT_BOTH)
      pressCount++;
    else if (countingMode == COUNT_FALLING) {
      if (prevSteadyState == HIGH && currentSteadyState == LOW) pressCount++;
    } else if (countingMode == COUNT_RISING) {
      if (prevSteadyState == LOW && currentSteadyState == HIGH) pressCount++;
    }
  }
}