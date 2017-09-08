#ifndef DEMO_PANDORA_GUI_ACTION_LABEL_H
#define DEMO_PANDORA_GUI_ACTION_LABEL_H

#include <vector>
#include <string>

#ifndef _WIN32
#include <planning_msgs/ActionDispatch.h>
#endif
#include "dpengine/gui/Container.h"
#include "dpengine/gui/fonts/Font.h"

namespace DreadedPE
{
	class ButtonPressedListener;
	class Label;
	class Font;
	class Shape;
};

class ActionLabel : public DreadedPE::Container
{
public:
	ActionLabel(const DreadedPE::Theme& theme, DreadedPE::Font& font, const glm::vec4& colour, float size_x, float size_y, const std::string& label, const planning_msgs::ActionDispatch& action);
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
	
	DreadedPE::GUIElement* processMousePressEvent(int x, int y);
	
	void processMouseReleasedEvent(int x, int y);
	
	void onResize(int width, int height);
private:
	DreadedPE::Font* font_;
	glm::vec4 colour_;
	std::string label_;
#ifndef _WIN32
	planning_msgs::ActionDispatch action_;
#endif
	float start_time_;
	float duration_;
	DreadedPE::Shape* shape_;

	DreadedPE::Label* tool_hint_;
};

#endif
