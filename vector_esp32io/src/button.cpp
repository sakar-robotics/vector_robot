#include <button.h>

button::button(int pin)
  : button(pin, INTERNAL_PULLUP) {};

button::button(int pin, int mode)
{
  // Initialize parameters for debouncing and press counting.
  buttonPin     = pin;
  debounceDelay = 0;
  pressCount    = 0;
  countingMode  = COUNT_FALLING;

  // Set the digital pin mode based on the provided wiring configuration.
  if (mode == INTERNAL_PULLUP || mode == INTERNAL_PULLDOWN) {
    pinMode(buttonPin, mode);
  } else if (mode == EXTERNAL_PULLUP || mode == EXTERNAL_PULLDOWN) {
    pinMode(buttonPin, INPUT);  // For external pull configs, use basic input mode
  }

  // Establish the logical pressed/unpressed values according to the wiring.
  if (mode == INTERNAL_PULLDOWN || mode == EXTERNAL_PULLDOWN) {
    buttonPressedState   = HIGH;
    buttonUnpressedState = LOW;
  } else {
    buttonPressedState   = LOW;
    buttonUnpressedState = HIGH;
  }

  // Initialize state tracking with the current reading.
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
  // A valid press is detected when the state transitions from unpressed to pressed
  if (prevSteadyState == buttonUnpressedState && currentSteadyState == buttonPressedState)
    return true;
  else
    return false;
}

bool button::isReleased(void)
{
  // A valid release is detected when the state transitions from pressed to unpressed
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
  // Sample the current state from the button pin.
  int currentState          = digitalRead(buttonPin);
  unsigned long currentTime = millis();

  // Detect raw state changes that could be due to bounce or an actual press.
  if (currentState != lastBouncyState) {
    // Restart the debounce timer and update the raw state.
    lastDebounceCheck = currentTime;
    lastBouncyState   = currentState;
  }

  // If the new state persists for the debounce duration, consider it stable.
  if ((currentTime - lastDebounceCheck) >= debounceDelay) {
    // Update debounced state values.
    prevSteadyState    = currentSteadyState;
    currentSteadyState = currentState;
  }

  // When a change in the stable state is confirmed, update the press count based on mode.
  if (prevSteadyState != currentSteadyState) {
    if (countingMode == COUNT_BOTH)
      pressCount++;
    else if (countingMode == COUNT_FALLING) {
      // Count only falling edge transitions.
      if (prevSteadyState == HIGH && currentSteadyState == LOW) pressCount++;
    } else if (countingMode == COUNT_RISING) {
      // Count only rising edge transitions.
      if (prevSteadyState == LOW && currentSteadyState == HIGH) pressCount++;
    }
  }
}