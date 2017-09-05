#ifndef CORE_GUI_FONTS_FONT_RENDERING_GUI_ELEMENT_H
#define CORE_GUI_FONTS_FONT_RENDERING_GUI_ELEMENT_H

#include "../GUIElement.h"

/**
 * A GUI Element that renders a font.
 */
class FontRenderingGUIElement : public GUIElement
{
public:
	FontRenderingGUIElement(const Theme& theme, float x, float y, float size_x, float size_y)
		: GUIElement(theme, x, y, size_x, size_y), need_to_update_font_(true)
	{

	}

	/**
	 * @return The font size of this element.
	 */
	virtual float getFontSize() const = 0;

	/**
	 * @return The font that is used to render this element.
	 */
	virtual const std::string& getText() const = 0;

	/**
	 * Set the flag that determines whether the font will get updated.
	 * @param need_to_update_font If true than the text that is rendered by the font will be updated to the GPU.
	 */
	void setFontNeedsUpdating(bool need_to_update_font) { need_to_update_font_ = need_to_update_font; }

	/**
	 * A function that checks whether the text inside a GUI element has been updated.
	 * @return True if the text has been updated and the buffers in the GPU need to be updated, false otherwise.
	 */
	bool fontIsUpdated() const { return need_to_update_font_; }

	virtual void onResize(int width, int height) { }
protected:
	bool need_to_update_font_;
};

#endif
