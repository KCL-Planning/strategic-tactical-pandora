#include "dpengine/gui/themes/MyGUITheme.h"

#include "dpengine/texture/TargaTexture.h"

namespace DreadedPE
{

MyGUITheme::MyGUITheme()
	: Theme(*(TargaTexture::loadTexture("textures/gui/gui.tga")))
{
	// Frame background.
	frame_texture_.push_back(glm::vec2(372.0f / 512.0f, (256.0f - 2.0f) / 256.0f));
	frame_texture_.push_back(glm::vec2(442.0f / 512.0f, (256.0f - 2.0f) / 256.0f));
	frame_texture_.push_back(glm::vec2(372.0f / 512.0f, (256.0f - 64.0f) / 256.0f));
	frame_texture_.push_back(glm::vec2(442.0f / 512.0f, (256.0f - 64.0f) / 256.0f));

	// Bordered frame.
	bordered_frame_texture_.push_back(glm::vec2(169.0f / 512.0f, (256.0f - 164.0f) / 256.0f));
	bordered_frame_texture_.push_back(glm::vec2(196.0f / 512.0f, (256.0f - 164.0f) / 256.0f));
	bordered_frame_texture_.push_back(glm::vec2(169.0f / 512.0f, (256.0f - 188.0f) / 256.0f));
	bordered_frame_texture_.push_back(glm::vec2(196.0f / 512.0f, (256.0f - 188.0f) / 256.0f));

	// Button up.
	button_texture_.push_back(glm::vec2(136.0f / 512.0f, (256.0f - 56.0f) / 256.0f));
	button_texture_.push_back(glm::vec2(162.0f / 512.0f, (256.0f - 56.0f) / 256.0f));
	button_texture_.push_back(glm::vec2(136.0f / 512.0f, (256.0f - 79.0f) / 256.0f));
	button_texture_.push_back(glm::vec2(162.0f / 512.0f, (256.0f - 79.0f) / 256.0f));

	// Button down.
	button_texture_.push_back(glm::vec2(136.0f / 512.0f, (256.0f - 83.0f) / 256.0f));
	button_texture_.push_back(glm::vec2(162.0f / 512.0f, (256.0f - 83.0f) / 256.0f));
	button_texture_.push_back(glm::vec2(136.0f / 512.0f, (256.0f - 106.0f) / 256.0f));
	button_texture_.push_back(glm::vec2(162.0f / 512.0f, (256.0f - 106.0f) / 256.0f));

	// Close button up.
	close_button_texture_.push_back(glm::vec2(71.0f / 512.0f, (256.0f - 39.0f) / 256.0f));
	close_button_texture_.push_back(glm::vec2(89.0f / 512.0f, (256.0f - 39.0f) / 256.0f));
	close_button_texture_.push_back(glm::vec2(71.0f / 512.0f, (256.0f - 56.0f) / 256.0f));
	close_button_texture_.push_back(glm::vec2(89.0f / 512.0f, (256.0f - 56.0f) / 256.0f));

	// Close button down.
	close_button_texture_.push_back(glm::vec2(71.0f / 512.0f, (256.0f - 58.0f) / 256.0f));
	close_button_texture_.push_back(glm::vec2(89.0f / 512.0f, (256.0f - 58.0f) / 256.0f));
	close_button_texture_.push_back(glm::vec2(71.0f / 512.0f, (256.0f - 75) / 256.0f));
	close_button_texture_.push_back(glm::vec2(89.0f / 512.0f, (256.0f - 75.0f) / 256.0f));

	// Label.
	label_texture_.push_back(glm::vec2(169.0f / 512.0f, (256.0f - 2.0f) / 256.0f));
	label_texture_.push_back(glm::vec2(196.0f / 512.0f, (256.0f - 2.0f) / 256.0f));
	label_texture_.push_back(glm::vec2(169.0f / 512.0f, (256.0f - 27.0f) / 256.0f));
	label_texture_.push_back(glm::vec2(196.0f / 512.0f, (256.0f - 27.0f) / 256.0f));

	// Minimise.
	minimise_texture_.push_back(glm::vec2(94.0f / 512.0f, (256.0f - 27.0f) / 256.0f));
	minimise_texture_.push_back(glm::vec2(107.0f / 512.0f, (256.0f - 27.0f) / 256.0f));
	minimise_texture_.push_back(glm::vec2(94.0f / 512.0f, (256.0f - 39.0f) / 256.0f));
	minimise_texture_.push_back(glm::vec2(107.0f / 512.0f, (256.0f - 39.0f) / 256.0f));

	// Maximise.
	maximise_texture_.push_back(glm::vec2(94.0f / 512.0f, (256.0f - 40.0f) / 256.0f));
	maximise_texture_.push_back(glm::vec2(107.0f / 512.0f, (256.0f - 40.0f) / 256.0f));
	maximise_texture_.push_back(glm::vec2(94.0f / 512.0f, (256.0f - 52.0f) / 256.0f));
	maximise_texture_.push_back(glm::vec2(107.0f / 512.0f, (256.0f - 52.0f) / 256.0f));

	// Check box.
	check_box_texture_.push_back(glm::vec2(2.0f / 512.0f, (256.0f - 46.0f) / 256.0f));
	check_box_texture_.push_back(glm::vec2(22.0f / 512.0f, (256.0f - 46.0f) / 256.0f));
	check_box_texture_.push_back(glm::vec2(2.0f / 512.0f, (256.0f - 66.0f) / 256.0f));
	check_box_texture_.push_back(glm::vec2(22.0f / 512.0f, (256.0f - 66.0f) / 256.0f));

	check_box_texture_.push_back(glm::vec2(2.0f / 512.0f, (256.0f - 134.0f) / 256.0f));
	check_box_texture_.push_back(glm::vec2(22.0f / 512.0f, (256.0f - 134.0f) / 256.0f));
	check_box_texture_.push_back(glm::vec2(2.0f / 512.0f, (256.0f - 154.0f) / 256.0f));
	check_box_texture_.push_back(glm::vec2(22.0f / 512.0f, (256.0f - 154.0f) / 256.0f));
	
	// Scrollbar.
	scrollbar_texture_.push_back(glm::vec2(169.0f / 512.0f, (256.0f - 2.0f) / 256.0f));
	scrollbar_texture_.push_back(glm::vec2(196.0f / 512.0f, (256.0f - 2.0f) / 256.0f));
	scrollbar_texture_.push_back(glm::vec2(169.0f / 512.0f, (256.0f - 27.0f) / 256.0f));
	scrollbar_texture_.push_back(glm::vec2(196.0f / 512.0f, (256.0f - 27.0f) / 256.0f));
	
	// Scrollbar unit.
	scrollbar_unit_texture_.push_back(glm::vec2(2.0f / 512.0f, (256.0f - 46.0f) / 256.0f));
	scrollbar_unit_texture_.push_back(glm::vec2(22.0f / 512.0f, (256.0f - 46.0f) / 256.0f));
	scrollbar_unit_texture_.push_back(glm::vec2(2.0f / 512.0f, (256.0f - 66.0f) / 256.0f));
	scrollbar_unit_texture_.push_back(glm::vec2(22.0f / 512.0f, (256.0f - 66.0f) / 256.0f));
	
	// Solid black.
	solid_black_texture_.push_back(glm::vec2(510.0f / 512.0f, (256.0f - 1.0f) / 256.0f));
	solid_black_texture_.push_back(glm::vec2(511.0f / 512.0f, (256.0f - 1.0f) / 256.0f));
	solid_black_texture_.push_back(glm::vec2(510.0f / 512.0f, (256.0f - 2.0f) / 256.0f));
	solid_black_texture_.push_back(glm::vec2(511.0f / 512.0f, (256.0f - 2.0f) / 256.0f));
	
	// Invisible texture.
	invisible_texture_.push_back(glm::vec2(1.0f / 512.0f, (256.0f - 250.0f) / 256.0f));
	invisible_texture_.push_back(glm::vec2(2.0f / 512.0f, (256.0f - 250.0f) / 256.0f));
	invisible_texture_.push_back(glm::vec2(1.0f / 512.0f, (256.0f - 251.0f) / 256.0f));
	invisible_texture_.push_back(glm::vec2(2.0f / 512.0f, (256.0f - 251.0f) / 256.0f));

	// Forward texture.
	forward_texture_.push_back(glm::vec2(94.0f / 512.0f, (256.0f - 195.0f) / 256.0f));
	forward_texture_.push_back(glm::vec2(106.0f / 512.0f, (256.0f - 195.0f) / 256.0f));
	forward_texture_.push_back(glm::vec2(94.0f / 512.0f, (256.0f - 208.0f) / 256.0f));
	forward_texture_.push_back(glm::vec2(106.0f / 512.0f, (256.0f - 208.0f) / 256.0f));

	// Backward texture.
	back_texture_.push_back(glm::vec2(94.0f / 512.0f, (256.0f - 130.0f) / 256.0f));
	back_texture_.push_back(glm::vec2(106.0f / 512.0f, (256.0f - 130.0f) / 256.0f));
	back_texture_.push_back(glm::vec2(94.0f / 512.0f, (256.0f - 143.0f) / 256.0f));
	back_texture_.push_back(glm::vec2(106.0f / 512.0f, (256.0f - 143.0f) / 256.0f));
}

};
