#ifndef DEMO_GUI_DEMO_OMNI_REPORTER_H
#define DEMO_GUI_DEMO_OMNI_REPORTER_H

class Label;

#include "../../core/gui/events/ButtonPressedListener.h"
#include "../../core/gui/events/CheckBoxChangeListener.h"

class OmniReporter : public ButtonPressedListener, public CheckBoxChangeListener
{
public:
	OmniReporter(Label& status_reporter);

	void buttonPressed(const Button& source);
	void checkBoxStateChanged(const CheckBox& source);
private:
	Label* status_reporter_;
};

#endif
