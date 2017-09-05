#ifndef DEMO_PANDORA_PLANNING_GUI_H
#define DEMO_PANDORA_PLANNING_GUI_H

#include "../../core/gui/events/ButtonPressedListener.h"

class RRT;
class AUV;
class Button;
class Label;
class MyGUITheme;
class Font;

/**
 * Creates the GUI that controls the planning aspect of the simulator.
 */
class PlanningGUI : public ButtonPressedListener
{
public:
	PlanningGUI(MyGUITheme& theme, Font& font, AUV& auv, RRT& rrt);
	
	/**
	 * Callback function for the buttons in the GUI.
	 */
	void buttonPressed(const Button& source);
private:
	MyGUITheme* theme_;
	AUV* auv_;
	RRT* rrt_;
	
	Button* start_rrt_;
	Button* stop_rrt_;

	Label* label_;
	Font* font_;
};

#endif
