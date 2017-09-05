#ifndef CORE_GUI_LABEL_H
#define CORE_GUI_LABEL_H

#include <vector>
#include <string>

#include "fonts/FontRenderingGUIElement.h"

class ButtonPressedListener;
class Font;
//class FreeTypeFont;

class Label : public FontRenderingGUIElement
{
public:
	Label(const Theme& theme, float size_x, float size_y, const std::string& label, float font_size);

	Label(const Theme& theme, float size_x, float size_y, const std::string& label, float font_size, const std::vector<glm::vec2>& uv_mapping);

	void setLabel(const std::string& label);

	//void draw(const glm::mat4& perspective_matrix, int level) const;

	float getFontSize() const { return font_size_; }

	const std::string& getText() const { return label_; }

	void onResize(int width, int height);

private:
	std::string label_;

	float font_size_;
};

#endif
