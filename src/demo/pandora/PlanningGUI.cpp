#include "GL/glew.h"
#include "GL/glfw.h"

#include "PlanningGUI.h"
#include "AUV.h"
#include "RRT.h"

#include "controllers/FollowWaypointController.h"

// GUI stuff.
#include "../../core/gui/GUIManager.h"
#include "../../core/gui/Frame.h"
#include "../../core/gui/Button.h"
#include "../../core/gui/events/ButtonPressedListener.h"
#include "../../core/gui/themes/MyGUITheme.h"
#include "../../core/gui/Label.h"
#include "../../core/gui/TextBox.h"
#include "../../core/gui/CheckBox.h"
#include "../../core/gui/fonts/Font.h"

PlanningGUI::PlanningGUI(MyGUITheme& theme, Font& font, AUV& auv, RRT& rrt)
	: theme_(&theme), auv_(&auv), rrt_(&rrt), font_(&font)
{
	int width, height;
	glfwGetWindowSize(&width, &height);
	GUIManager& gui_manager = GUIManager::getInstance();

	Frame& frame = gui_manager.createFrame(theme, 10, height - 170, 300, 150);

	start_rrt_ = new Button(theme, 90, 20, "Start RRT");
	frame.addElement(*start_rrt_, 20, -70);
	start_rrt_->addListener(*this);

	stop_rrt_ = new Button(theme, 90, 20, "Clear RRT");
	frame.addElement(*stop_rrt_, 140, -70);
	stop_rrt_->addListener(*this);

	label_ = new Label(theme, 150, 30, "Test Label");
	frame.addElement(*label_, 70, -120);

	TextBox* tb = new TextBox(theme, font.clone(), "", 200, 35);
	frame.addElement(*tb, 70, -30);

	CheckBox* cb = new CheckBox(theme, 15, 15);
	frame.addElement(*cb, 10, -120);

	CheckBox* cb2 = new CheckBox(theme, 15, 15);
	frame.addElement(*cb2, 40, -120);
}

void PlanningGUI::buttonPressed(const Button& source)
{
	if (&source == start_rrt_)
	{
		std::vector<glm::vec3> goals;
		goals.push_back(glm::vec3(0, 7, -7));
		//rrt_->createRRT(auv_->getGlobalLocation(), goals, std::min(auv_->getGlobalLocation().x, 0.0f) - 5, std::max(auv_->getGlobalLocation().x, 0.0f) + 5, std::min(auv_->getGlobalLocation().y, 0.0f) - 5, std::max(auv_->getGlobalLocation().y, 0.0f) + 5, std::min(auv_->getGlobalLocation().z, 0.0f) - 5, std::max(auv_->getGlobalLocation().z, 0.0f) + 5);
		//auv_->getController().followWaypoints(rrt_->getPathLocations());
	}
	else if (&source == stop_rrt_)
	{
		rrt_->clear();
	}
}
