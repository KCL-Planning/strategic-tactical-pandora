#ifndef CORE_GUI_CHECK_BOX_H
#define CORE_GUI_CHECK_BOX_H

#include <vector>

#include "GUIElement.h"

class CheckBoxChangeListener;

/**
 * Implements a check box.
 */
class CheckBox : public GUIElement
{
public:
	CheckBox(Theme& theme, float size_x, float size_y);

	GUIElement* processMousePressEvent(int x, int y);
	void processMouseReleasedEvent(int x, int y);

	void addStateChangeListener(CheckBoxChangeListener& listener);
	void removeStateChangeListener(CheckBoxChangeListener& listener);

	bool isPressed() const { return is_pressed_; }

	//void draw(const glm::mat4& perspective_matrix, int level) const;
	void onResize(int width, int height);
private:
	std::vector<glm::vec2> complete_tex_coords_;

	bool is_pressed_;
	bool mouse_is_down_;

	std::vector<CheckBoxChangeListener*> listeners_;
};

#endif
