#ifndef MY_CUBE_H
#define MY_CUBE_H

#include <glm/glm.hpp>

#include "Shape.h"

class Cube: public Shape
{
public:
	Cube(float size);
	Cube(float width, float height, float depth);
	Cube(const glm::vec3& bottom_left_away, 
         const glm::vec3& bottom_right_away, 
         const glm::vec3& top_left_away, 
         const glm::vec3& top_right_away, 
         const glm::vec3& bottom_left_close, 
         const glm::vec3& bottom_right_close, 
         const glm::vec3& top_left_close, 
         const glm::vec3& top_right_close);

private:
	void initialise(float width, float height, float depth);
};

#endif
