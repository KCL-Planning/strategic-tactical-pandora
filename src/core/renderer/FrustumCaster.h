#ifndef CORE_ENTITES_CAMERA_FRUSTUM_CASTER_H
#define CORE_ENTITES_CAMERA_FRUSTUM_CASTER_H

#include <glm/glm.hpp>

/**
 * Interface for all entities where we can render from.
 */
class FrustumCaster
{
public:
	virtual glm::vec3 getLocation() const = 0;
	virtual glm::mat4 getViewMatrix() const = 0;
	virtual glm::mat4 getPerspectiveMatrix() const = 0;
};

#endif
