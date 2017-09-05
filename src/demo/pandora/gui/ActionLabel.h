#ifndef DEMO_PANDORA_GUI_ACTION_LABEL_H
#define DEMO_PANDORA_GUI_ACTION_LABEL_H

#include <vector>
#include <string>

#ifndef _WIN32
#include <planning_msgs/ActionDispatch.h>
#endif
#include "../../../core/gui/Container.h"
#include "../../../core/gui/fonts/Font.h"

class ButtonPressedListener;
//class FreeTypeFont;
class Label;
class Font;

class ActionLabel : public Container
{
public:
#ifndef _WIN32
	ActionLabel(const Theme& theme, Font& font, const glm::vec4& colour, float size_x, float size_y, const std::string& label, const planning_msgs::ActionDispatch& action);
#else
	ActionLabel(const Theme& theme, Font& font, const glm::vec4& colour, float size_x, float size_y, const std::string& label);
#endif
	void setLabel(const std::string& label) { label_ = label; font_->setString(label, 12); }

	void draw(const glm::mat4& perspective_matrix, int level) const;
#ifndef _WIN32	
	const planning_msgs::ActionDispatch& getAction() const { return action_; }
#endif

	void setStartTime(float start_time) { start_time_ = start_time; }
	void setDuration(float duration) { duration_ = duration; }
	float getStartTime() const { return start_time_; }
	float getDuration() const { return duration_; }

	void setDimensions(float width, float height);
	
	GUIElement* processMousePressEvent(int x, int y);
	
	void processMouseReleasedEvent(int x, int y);
	
	void onResize(int width, int height);
private:
	Font* font_;
	glm::vec4 colour_;
	std::string label_;
#ifndef _WIN32
	planning_msgs::ActionDispatch action_;
#endif
	float start_time_;
	float duration_;

	Label* tool_hint_;
};

#endif
