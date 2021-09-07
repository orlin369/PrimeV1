// 
// 
// 

#include "Button.h"

void ButtonClass::init(int pin, int debounce = 70, int mode = INPUT_PULLUP)
{
	m_pin = pin;
	m_debounce = debounce;
	pinMode(m_pin, mode);
}

void ButtonClass::update()
{
	// read the state of the switch into a local variable:
	int StateL = digitalRead(m_pin);

	// check to see if you just pressed the button
	// (i.e. the input went from LOW to HIGH), and you've waited long enough
	// since the last press to ignore any noise:

	// If the switch changed, due to noise or pressing:
	if (StateL != m_lastState) {
		// reset the debouncing timer
		m_lastDebounceTime = millis();
	}

	if ((millis() - m_lastDebounceTime) > m_debounce) {
		// whatever the StateL is at, it's been there for longer than the debounce
		// delay, so take it as the actual current state:

		// if the button state has changed:
		if (StateL != m_state) {
			m_state = StateL;
		}
	}

	// save the StateL. Next time through the loop, it'll be the LastButtonState_g:
	m_state = StateL;
}

int ButtonClass::getState()
{
	return m_state;
}

void ButtonClass::setDebounce(int debounce)
{
	m_debounce = debounce;
}
