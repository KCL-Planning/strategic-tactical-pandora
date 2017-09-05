/**
 * A theme stores an atlas texture of a GUI and all the UV mappings.
 * This should probably become an XML document but I can't be arsed right now :).
 */

#ifndef CORE_GUI_THEMES_THEME_H
#define CORE_GUI_THEMES_THEME_H

#include <vector>
#include <glm/glm.hpp>

class Texture;

class Theme
{
public:
	Theme(Texture& texture)
		: texture_(&texture)
	{

	}

	virtual const std::vector<glm::vec2>& getFrameTexture() const = 0;
	virtual const std::vector<glm::vec2>& getCloseButtonTexture() const = 0;
	virtual const std::vector<glm::vec2>& getButtonTexture() const = 0;
	virtual const std::vector<glm::vec2>& getLabelTexture() const = 0;
	virtual const std::vector<glm::vec2>& getCheckBoxTexture() const = 0;

	virtual const std::vector<glm::vec2>& getMinimiseTexture() const = 0;
	virtual const std::vector<glm::vec2>& getMaximiseTexture() const = 0;
	
	virtual const std::vector<glm::vec2>& getScrollbarTexture() const = 0;
	virtual const std::vector<glm::vec2>& getScrollbarUnitTexture() const = 0;
	virtual const std::vector<glm::vec2>& getSolidBlackTexture() const = 0;
	virtual const std::vector<glm::vec2>& getInvisibleTexture() const = 0;

	Texture& getTexture() const { return *texture_; }
protected:
	Texture* texture_;
};

#endif
