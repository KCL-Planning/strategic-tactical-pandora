#ifndef CORE_LOADERS_PLF_CUBE_H
#define CORE_LOADERS_PLF_CUBE_H

#include <glm/glm.hpp>

#include "../../shapes/Shape.h"
#include "../math/Plane.h"

class PLF_Cube : public Shape
{
public:
	PLF_Cube(const glm::vec3& bottom_left_away,
	         const glm::vec3& bottom_right_away,
	         const glm::vec3& top_left_away,
	         const glm::vec3& top_right_away,
	         const glm::vec3& bottom_left_close,
	         const glm::vec3& bottom_right_close,
	         const glm::vec3& top_left_close,
	         const glm::vec3& top_right_close);

	bool operator==(const PLF_Cube& rhs) const;
	
	bool isInside(const glm::vec3& point) const;
	
	const std::vector<Plane>& getPlanes() const { return planes_; }

	glm::vec3 bottom_left_away_;
	glm::vec3 bottom_right_away_;
	glm::vec3 top_left_away_;
	glm::vec3 top_right_away_;
	glm::vec3 bottom_left_close_;
	glm::vec3 bottom_right_close_;
	glm::vec3 top_left_close_;
	glm::vec3 top_right_close_;
	
private:
	
	/**
	 * Create a set of triangles in CCW orientation, store these as a shape.
	 */
	void initialise();
	
	std::vector<Plane> planes_;
};

std::ostream& operator<<(std::ostream& os, const PLF_Cube& cube);

#endif
