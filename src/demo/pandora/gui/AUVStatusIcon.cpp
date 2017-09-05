#include "AUVStatusIcon.h"

#include "../../../core/texture/TargaTexture.h"

AUVStatusIcon* AUVStatusIcon::instance_ = NULL;
Texture* AUVStatusIcon::icons_texture_ = NULL;

AUVStatusIcon::AUVStatusIcon()
{
	icons_texture_ = TargaTexture::loadTexture("data/textures/icons.tga");
	
	// Goto icon.
	goto_icon_.push_back(glm::vec2(0.75f, 0.75f));
	goto_icon_.push_back(glm::vec2(1.0f, 0.75f));
	goto_icon_.push_back(glm::vec2(0.75f, 0.5f));
	goto_icon_.push_back(glm::vec2(1.0f, 0.5f));
	
	// Observe icon.
	observe_icon_.push_back(glm::vec2(0.25f, 1.0f));
	observe_icon_.push_back(glm::vec2(0.5f, 1.0f));
	observe_icon_.push_back(glm::vec2(0.25f, 0.75));
	observe_icon_.push_back(glm::vec2(0.5f, 0.75f));
	
	// Turn valve icon.
	turn_valve_icon_.push_back(glm::vec2(0.0f, 1.0f));
	turn_valve_icon_.push_back(glm::vec2(0.25f, 1.0f));
	turn_valve_icon_.push_back(glm::vec2(0.0f, 0.75f));
	turn_valve_icon_.push_back(glm::vec2(0.25f, 0.75f));
	
	// Collision icon.
	collision_icon_.push_back(glm::vec2(0.5f, 1.0f));
	collision_icon_.push_back(glm::vec2(0.75f, 1.0f));
	collision_icon_.push_back(glm::vec2(0.5f, 0.75f));
	collision_icon_.push_back(glm::vec2(0.75f, 0.75f));
	
	// Recharge icon.
	recharge_icon_.push_back(glm::vec2(0.0f, 0.5f));
	recharge_icon_.push_back(glm::vec2(0.25f, 0.5f));
	recharge_icon_.push_back(glm::vec2(0.0f, 0.25f));
	recharge_icon_.push_back(glm::vec2(0.25f, 0.25f));
	
	// Dock icon.
	dock_icon_.push_back(glm::vec2(0.25f, 0.5f));
	dock_icon_.push_back(glm::vec2(0.5f, 0.5f));
	dock_icon_.push_back(glm::vec2(0.25f, 0.25f));
	dock_icon_.push_back(glm::vec2(0.5f, 0.25f));
	
	// Undock icon.
	undock_icon_.push_back(glm::vec2(0.5f, 0.5f));
	undock_icon_.push_back(glm::vec2(0.75f, 0.5f));
	undock_icon_.push_back(glm::vec2(0.5f, 0.25f));
	undock_icon_.push_back(glm::vec2(0.75f, 0.25f));
	
	// Empty icon.
	empty_icon_.push_back(glm::vec2(0, 0));
	empty_icon_.push_back(glm::vec2(0, 0));
	empty_icon_.push_back(glm::vec2(0, 0));
	empty_icon_.push_back(glm::vec2(0, 0));
}

				

AUVStatusIcon& AUVStatusIcon::getInstance()
{
	if (instance_ == NULL)
	{
		instance_ = new AUVStatusIcon();
	}
	return *instance_;
}
