#ifndef CORE_GUI_THEME_MY_GUI_THEME_H
#define CORE_GUI_THEME_MY_GUI_THEME_H

#include "Theme.h"


class MyGUITheme : public Theme
{
public:
	MyGUITheme();

	const std::vector<glm::vec2>& getFrameTexture() const { return frame_texture_; }
	const std::vector<glm::vec2>& getCloseButtonTexture() const { return close_button_texture_; }
	const std::vector<glm::vec2>& getButtonTexture() const { return button_texture_; }
	const std::vector<glm::vec2>& getLabelTexture() const { return label_texture_; }
	const std::vector<glm::vec2>& getMinimiseTexture() const { return minimise_texture_; }
	const std::vector<glm::vec2>& getMaximiseTexture() const { return maximise_texture_; }
	const std::vector<glm::vec2>& getCheckBoxTexture() const { return check_box_texture_; }
	const std::vector<glm::vec2>& getScrollbarTexture() const { return scrollbar_texture_; }
	const std::vector<glm::vec2>& getScrollbarUnitTexture() const { return scrollbar_unit_texture_; }
	const std::vector<glm::vec2>& getSolidBlackTexture() const { return solid_black_texture_; }
	const std::vector<glm::vec2>& getInvisibleTexture() const  { return invisible_texture_; }

private:
	std::vector<glm::vec2> frame_texture_;
	std::vector<glm::vec2> close_button_texture_;
	std::vector<glm::vec2> button_texture_;
	std::vector<glm::vec2> label_texture_;
	std::vector<glm::vec2> minimise_texture_;
	std::vector<glm::vec2> maximise_texture_;
	std::vector<glm::vec2> check_box_texture_;
	std::vector<glm::vec2> scrollbar_texture_;
	std::vector<glm::vec2> scrollbar_unit_texture_;
	std::vector<glm::vec2> solid_black_texture_;
	std::vector<glm::vec2> invisible_texture_;
};

#endif
