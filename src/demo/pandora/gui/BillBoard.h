#ifndef DEMO_PANDORA_GUI_BILL_BOARD_H
#define DEMO_PANDORA_GUI_BILL_BOARD_H

#include <vector>
#include <glm/glm.hpp>

#include "dpengine/gui/Container.h"

namespace DreadedPE
{
	class Camera;
	class Theme;
	class SceneNode;
	class Font;
}

/**
 * Create a shape that will always face a camera.
 */
class BillBoard : public DreadedPE::Container
{
public:
	BillBoard(const DreadedPE::Theme& theme, DreadedPE::Font& font, DreadedPE::SceneNode& to_follow, DreadedPE::Camera& camera, const glm::vec3& offset, float width, float height, const std::vector<glm::vec2>& tex_coords);
	
	/**
	 * Update the textures of this billboard.
	 */
	virtual void update(float dt);
	
	void setUVMapping(const std::vector<glm::vec2>& uv);
	
	/**
	 * Update the location of the billboard and draw it.
	 */
	void draw(const glm::mat4& perspective_matrix, int level) const;
	
	void onResize(int width, int height);
	
	void enableBlinking(bool blink) { blink_ = blink; }
	
private:
	bool blink_;
	bool blink_visible_;
	float time_to_next_blink_;
	glm::vec3 location_;
	DreadedPE::SceneNode* to_follow_;
	DreadedPE::Camera* camera_;
	glm::vec3 offset_;
};


#endif
