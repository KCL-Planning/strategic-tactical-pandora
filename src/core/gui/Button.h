#ifndef CORE_GUI_BUTTON_H
#define CORE_GUI_BUTTON_H

#include <vector>

#include "fonts/FontRenderingGUIElement.h"

class ButtonPressedListener;
//class FreeTypeFont;

class Button : public FontRenderingGUIElement
{
public:
	Button(Theme& theme, float size_x, float size_y, const std::string& label, float font_size);

	Button(Theme& theme, float size_x, float size_y, const std::string& label, float font_size, const std::vector<glm::vec2>& uv_mapping);

	float getFontSize() const { return font_size_; }

	const std::string& getText() const { return label_; }

	GUIElement* processMousePressEvent(int x, int y);
	void processMouseReleasedEvent(int x, int y);

	void addListener(ButtonPressedListener& listener);
	void removeListener(ButtonPressedListener& listener);

	std::string getLabel() const { return label_; }
	void setLabel(const std::string& label) { label_ = label; setFontNeedsUpdating(true); }

	void onResize(int width, int height);
private:
	std::string label_;
	float font_size_;

	std::vector<glm::vec2> complete_tex_coords_;

	bool is_pressed_;

	std::vector<ButtonPressedListener*> listeners_;
};

#endif
