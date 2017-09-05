#ifndef CORE_GUI_WINDOW_H
#define CORE_GUI_WINDOW_H

#include <GL/glew.h>
#include <vector>
#include <glm/glm.hpp>

#include "Container.h"
#include "events/ButtonPressedListener.h"

class Button;
class Theme;
class Font;

/**
 * This represent a frame that can be resized and closed.
 */
class Frame : public Container, public ButtonPressedListener
{
public:
	Frame(Theme& theme, Font& font, float x, float y, float size_x, float size_y);

	GUIElement* processMousePressEvent(int mouse_x, int mouse_y);
	void processMouseReleasedEvent(int mouse_x, int mouse_y);

	virtual void draw(const glm::mat4& perspective_matrix, int level) const;

	virtual void update(float dt);

	virtual void buttonPressed(const Button& source);
	
	virtual void addElement(GUIElement& child, float x, float y);
	virtual void removeElement(GUIElement& child);
	
	virtual void addElement(Container& child, float x, float y);
	virtual void removeElement(Container& child);

private:
	Font* font_;

	Button* close_button_;
	Button* resize_button_;

	float org_size_x, org_size_y_;

	Container* menu_bar_;
	Container* content_pane_;

	bool is_minimized_;
	bool frame_grapped_;
	int grap_location_x_, grap_location_y_;
	int org_location_x_, org_location_y_;
};

#endif
