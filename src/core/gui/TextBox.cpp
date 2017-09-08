#include "dpengine/gui/TextBox.h"

#include "dpengine/gui/themes/Theme.h"
#include "dpengine/gui/Container.h"
#include "dpengine/shapes/Line.h"
#include "dpengine/shaders/GUIShader.h"
#include "dpengine/renderer/Window.h"

namespace DreadedPE
{

TextBox::TextBox(Theme& theme, Font& font, const std::string& text, float size_x, float size_y, float font_size)
	: Container(theme, font, 0.0f, 0.0f, size_x, size_y, false), cursor_location_(0), font_size_(font_size), cursor_blink_rate_(0.5f), cursor_time_since_last_blink_(0.0f), show_cursor_(false)
{
	label_ = new Label(theme, size_x, size_y, text, font_size);
	cursor_ = new Label(theme, 2, size_y - 2, "", 0, theme.getSolidBlackTexture());

	addElement(*label_, 0, 0);
	addElement(*cursor_, 0, 0);
	Window* window = Window::getActiveWindow();
	window->addKeyListener(*this);
}

void TextBox::keyPressed(int key, int mods)
{
	if (is_selected_)
	{
		// Handle moving the cursor.
		if (key == GLFW_KEY_LEFT && cursor_location_ > 0)
		{
			--cursor_location_;
			updateBuffers();
		}
		if (key == GLFW_KEY_RIGHT && cursor_location_ < label_->getText().size())
		{
			++cursor_location_;
			updateBuffers();
		}
		if (key == GLFW_KEY_END)
		{
			cursor_location_ = label_->getText().size();
			updateBuffers();
		}
		if (key == GLFW_KEY_HOME)
		{
			cursor_location_ = 0;
			updateBuffers();
		}

		// Handle deleting text.
		if (key == GLFW_KEY_BACKSPACE && cursor_location_ > 0)
		{
			label_->setLabel(label_->getText().substr(0, cursor_location_ - 1) + label_->getText().substr(cursor_location_));
			--cursor_location_;
			updateBuffers();
		}
		if (key == GLFW_KEY_DELETE && cursor_location_ < label_->getText().size())
		{
			label_->setLabel(label_->getText().substr(0, cursor_location_) + label_->getText().substr(cursor_location_ + 1));
		}

		// Handle new text.
		for (char i = ' '; i < '~' + 1; ++i)
		{
			char key_char = i;
			if (key == key_char)
			{
				if (mods & GLFW_MOD_SHIFT == 0 && i >= 'A' && i <= 'Z')
				{
					key_char += 32;
				}
				label_->setLabel(label_->getText().substr(0, cursor_location_) + key_char + label_->getText().substr(cursor_location_));
				++cursor_location_;
				updateBuffers();
			}
		}

		cursor_->setPosition(cursor_location_ * font_size_, 0);
	}
}

void TextBox::update(float dt)
{
	// Blink the cursor.
	if (is_selected_)
	{
		cursor_time_since_last_blink_ += dt;
		while (cursor_time_since_last_blink_ > cursor_blink_rate_)
		{
			cursor_time_since_last_blink_ -= cursor_blink_rate_;
			show_cursor_ = !show_cursor_;

			cursor_->setVisible(show_cursor_);

			updateBuffers();
		}
	}
	Container::update(dt);
}

GUIElement* TextBox::processMousePressEvent(int x, int y)
{
	markForUpdate();
	is_selected_ = true;
	// Set the cursor where the user clicked on the text.
	int local_x = x - global_transformation_[3][0];
	
	// Check which character this is.
	int current_location = 0;
	int i = 0;
	while (current_location < local_x && i < label_->getText().size())
	{
		// Update the location based on the character width.
		current_location += font_size_;
		++i;
	}
	cursor_location_ = i;
	cursor_->setPosition(cursor_location_ * font_size_, 0);
	return this;
}

};
