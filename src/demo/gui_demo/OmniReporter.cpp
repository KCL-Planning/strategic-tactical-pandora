#include "OmniReporter.h"

#include <sstream>

#include "../../core/gui/Button.h"
#include "../../core/gui/CheckBox.h"
#include "../../core/gui/Label.h"

OmniReporter::OmniReporter(Label& status_reporter)
	: status_reporter_(&status_reporter)
{

}

void OmniReporter::buttonPressed(const Button& source)
{
	std::stringstream ss;
	ss << source.getLabel() << " has been pressed!";
	status_reporter_->setLabel(ss.str());
}

void OmniReporter::checkBoxStateChanged(const CheckBox& source)
{
	std::stringstream ss;
	ss << "A check box has been " << (source.isPressed() ? " pressed!" : " not pressed!");
	status_reporter_->setLabel(ss.str());
}
