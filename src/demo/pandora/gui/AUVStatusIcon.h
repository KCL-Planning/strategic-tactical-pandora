#ifndef PANDORA_GUI_AUV_STATUS_ICON_H
#define PANDORA_GUI_AUV_STATUS_ICON_H

#include <vector>
#include <glm/glm.hpp>

class Texture;

class AUVStatusIcon
{
public:
	static AUVStatusIcon& getInstance();
	
	const std::vector<glm::vec2>& getGotoIcon() const { return goto_icon_; }
	const std::vector<glm::vec2>& getObserveIcon() const { return observe_icon_; }
	const std::vector<glm::vec2>& getTurnValveIcon() const { return turn_valve_icon_; }
	const std::vector<glm::vec2>& getCollisionIcon() const { return collision_icon_; }
	const std::vector<glm::vec2>& getRechargeIcon() const { return recharge_icon_; }
	const std::vector<glm::vec2>& getDockIcon() const { return dock_icon_; }
	const std::vector<glm::vec2>& getUndockIcon() const { return undock_icon_; }
	const std::vector<glm::vec2>& getEmptyIcon() const { return empty_icon_; }
	
private:
	
	AUVStatusIcon();
	
	std::vector<glm::vec2> goto_icon_;
	std::vector<glm::vec2> observe_icon_;
	std::vector<glm::vec2> turn_valve_icon_;
	std::vector<glm::vec2> collision_icon_;
	std::vector<glm::vec2> recharge_icon_;
	std::vector<glm::vec2> dock_icon_;
	std::vector<glm::vec2> undock_icon_;
	std::vector<glm::vec2> empty_icon_;
	
	
	static AUVStatusIcon* instance_;
	static Texture* icons_texture_;
};

#endif
