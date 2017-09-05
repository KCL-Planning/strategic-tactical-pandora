#include "TextBox.h"

#include <GL/glfw.h>

#include "themes/Theme.h"
#include "Container.h"
#include "../../shapes/Line.h"
#include "../shaders/LineShader.h"
#include "../shaders/GUIShader.h"

TextBox::TextBox(Theme& theme, Font& font, const std::string& text, float size_x, float size_y, float font_size)
	: Container(theme, font, 0.0f, 0.0f, size_x, size_y, false), cursor_location_(0), font_size_(font_size), cursor_blink_rate_(0.3f), cursor_time_since_last_blink_(0.0f), show_cursor_(false), time_since_last_key_processed_(0)
{
	int width, height;
	glfwGetWindowSize(&width, &height);

	label_ = new Label(theme, size_x, size_y, text, font_size);
	cursor_ = new Label(theme, 10, size_y - 2, "", 0, theme.getSolidBlackTexture());

	addElement(*label_, 0, 0);
	addElement(*cursor_, 0, 0);
}
	
void TextBox::update(float dt)
{
	if (time_since_last_key_processed_ > 0.1f)
	{
		// Handle moving the cursor.
		if (glfwGetKey(GLFW_KEY_LEFT) == GLFW_PRESS && cursor_location_ > 0)
		{
			--cursor_location_;
			updateBuffers();
		}
		if (glfwGetKey(GLFW_KEY_RIGHT) == GLFW_PRESS && cursor_location_ < label_->getText().size())
		{
			++cursor_location_;
			updateBuffers();
		}
		if (glfwGetKey(GLFW_KEY_END) == GLFW_PRESS)
		{
			cursor_location_ = label_->getText().size();
			updateBuffers();
		}
		if (glfwGetKey(GLFW_KEY_HOME) == GLFW_PRESS)
		{
			cursor_location_ = 0;
			updateBuffers();
		}

		// Handle deleting text.
		if (glfwGetKey(GLFW_KEY_BACKSPACE) == GLFW_PRESS && cursor_location_ > 0)
		{
			label_->setLabel(label_->getText().substr(0, cursor_location_ - 1) + label_->getText().substr(cursor_location_));
			--cursor_location_;
			updateBuffers();
		}
		if (glfwGetKey(GLFW_KEY_DEL) == GLFW_PRESS && cursor_location_ < label_->getText().size())
		{
			label_->setLabel(label_->getText().substr(0, cursor_location_) + label_->getText().substr(cursor_location_ + 1));
		}

		// Handle new text.
		for (char i = ' '; i < '~' + 1; ++i)
		{
			char key = i;
			if (glfwGetKey(key) == GLFW_PRESS)
			{
				if (glfwGetKey(GLFW_KEY_LSHIFT) != GLFW_PRESS && glfwGetKey(GLFW_KEY_RSHIFT) != GLFW_PRESS && i >= 'A' && i <= 'Z')
				{
					key += 32;
				}
				label_->setLabel(label_->getText().substr(0, cursor_location_) + key + label_->getText().substr(cursor_location_));
				++cursor_location_;
				updateBuffers();
			}
		}

		time_since_last_key_processed_ = 0;
	}
	else
	{
		time_since_last_key_processed_ += dt;
	}

	cursor_->setPosition(cursor_location_ * font_size_, 0);

	// Blink the cursor.
	cursor_time_since_last_blink_ += dt;
	while (cursor_time_since_last_blink_ > cursor_blink_rate_)
	{
		cursor_time_since_last_blink_ -= cursor_blink_rate_;
		show_cursor_ = !show_cursor_;

		cursor_->setVisible(show_cursor_);

		updateBuffers();
	}

	Container::update(dt);
}

GUIElement* TextBox::processMousePressEvent(int x, int y)
{
	// Set the cursor where the user clicked on the text.
	int local_x = x - global_transformation_[3][0];
	//int local_y = y + global_transformation_[3][1] - offset_y_;

#ifdef _WIN32
	std::stringstream ss;
	ss << "Clicked on the text box: " << x << " - " << global_transformation_[3][0] << " + " << " = " << local_x << " ---- " << std::endl; 
	//ss << "Clicked on the text box: " << y << " + " << global_transformation_[3][1] << " + " << offset_y_ << " = " << local_y << std::endl;
	OutputDebugString(ss.str().c_str());
#endif

	// Check which character this is.
	int current_location = 0;
	int i = 0;
	while (current_location < local_x && i < label_->getText().size())
	{
		// Update the location based on the character width.
		current_location += font_size_;
#ifdef _WIN32
	std::stringstream ss;
	ss << "After processing: " << label_->getText().at(i) << " current_location= " << current_location << std::endl;
	OutputDebugString(ss.str().c_str());
#endif
		++i;
	}
	cursor_location_ = i;

	return this;
}
/*
void TextBox::onResize(int width, int height)
{
	m_vertices_.clear();
	m_vertices_.push_back(glm::vec3(0, height, 0));
	m_vertices_.push_back(glm::vec3(size_x_, height, 0));
	m_vertices_.push_back(glm::vec3(0, height - size_y_, 0));
	m_vertices_.push_back(glm::vec3(size_x_, height - size_y_, 0));
}*/
