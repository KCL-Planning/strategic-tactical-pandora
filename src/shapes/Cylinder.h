#ifndef SHAPES_CYLINDER_H
#define SHAPES_CYLINDER_H

#include <glm/glm.hpp>

#include "Shape.h"

class Cylinder : public Shape
{
public:
	Cylinder(float height, float width, int sections);

private:
	void initialise(float width, float height, float depth);
};

#endif
