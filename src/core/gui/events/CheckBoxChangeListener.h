#ifndef CORE_GUI_EVENTS_CHECK_BOX_CHANGE_EVENT_H
#define CORE_GUI_EVENTS_CHECK_BOX_CHANGE_EVENT_H

class CheckBox;

class CheckBoxChangeListener
{
public:
	virtual void checkBoxStateChanged(const CheckBox& source) = 0;
};

#endif
