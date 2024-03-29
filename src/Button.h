// Button.h

#ifndef _BUTTON_h
#define _BUTTON_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "arduino.h"
#else
	#include "WProgram.h"
#endif

class ButtonClass
{
 protected:

#pragma region Variables

	/**
	 * @brief Pin that is responsible for the button.
	 * 
	 */
	int m_pin;

	 /**
	  * @brief The current reading from the input pin.
	  * 
	  */
	 int m_state;

	/**
	 * @brief The previous reading from the input pin.
	 * 
	 */
	 int m_lastState;

	/**
	 * @brief Debounce time in [ms].
	 * 
	 */
	 int m_debounce = 50;

	 /**
	  * @brief The last time the output pin was toggled.
	  * 
	  */
	 unsigned long m_lastDebounceTime = 0;

#pragma endregion

 public:

#pragma region Methods

	 void init(int pin, int debounce = 70, int mode = INPUT_PULLUP);

	 void update();

	 int getState();

	 void setDebounce(int debounce);

#pragma endregion

};

#endif
