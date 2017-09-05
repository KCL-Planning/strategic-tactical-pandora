/**
 * A single line box where the user can enter text.
 */

#ifndef CORE_GUI_TEXT_BOX_H
#define CORE_GUI_TEXT_BOX_H

#include <string>
#include <vector>

#include "Container.h"

#include "Label.h"
#include "fonts/FontRenderingGUIElement.h"

class Cursor;
class Font;

//class FreeTypeFont;
//class Line;

class TextBox : public Container
{
public:
	TextBox(Theme& theme, Font& font, const std::string& text, float size_x, float size_y, float font_size);
	
	/**
	 * Process the user input and place the text into the text box.
	 */
	void update(float dt);

	/**
	 * Move the cursor to the location in the text box where the users has clicked.
	 */
	GUIElement* processMousePressEvent(int x, int y);

	/**
	 * @return The font size of this element.
	 */
	float getFontSize() const { return font_size_; }

	/**
	 * @return The font that is used to render this element.
	 */
	const std::string& getText() const { return label_->getText(); }

	//void onResize(int width, int height);

private:

	Label* label_;
	Label* cursor_;

	//std::string text_;    // The text that is visible on the Text Box.
	int cursor_location_; // The character index where the cursur is currently present.

	float font_size_;     // The font size.

	//int offset_x_; // The offset of this element where the text start.
	//int offset_y_; // The offset of this element where the text start.

	float cursor_blink_rate_;              // The rate at which the cursor will blink.
	float cursor_time_since_last_blink_;   // The last time since the cursor blinked.
	bool show_cursor_;                     // If true the cursor will be shown.

	float time_since_last_key_processed_;  // In order to slow down capturing key strokes we only allow a few keys per second.
};

#endif
