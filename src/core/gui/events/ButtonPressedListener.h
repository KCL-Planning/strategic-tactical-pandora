#ifndef CORE_GUI_EVENTS_BUTTON_PRESSED_LISTENER_H
#define CORE_GUI_EVENTS_BUTTON_PRESSED_LISTENER_H

class Button;

class ButtonPressedListener
{
public:
	virtual void buttonPressed(const Button& source) = 0;
};

#endif
