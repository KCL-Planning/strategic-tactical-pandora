#ifndef CORE_SCENE_FRUSTRUM_FRUSTRUM_H
#define CORE_SCENE_FRUSTRUM_FRUSTRUM_H

#include <glm/glm.hpp>

#include <vector>

class Frustum
{
public:
	Frustum(const glm::mat4& model_view_projection_matrix);

	Frustum(const std::vector<glm::vec4>& planes);

	void setFrustum(const glm::mat4& model_view_projection_matrix);

	const std::vector<glm::vec4>& getPlanes() const { return planes_; }

private:
	void finalise(unsigned int i);
	std::vector<glm::vec4> planes_;
};

#endif
