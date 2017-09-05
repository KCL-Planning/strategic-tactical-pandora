#ifndef DEMO_SHAPES_FRUSTUM_H
#define DEMO_SHAPES_FRUSTUM_H

#include "Shape.h"

class FrustumShape : public Shape
{
public:
	FrustumShape(float close_plane, float far_plane, float close_x, float close_y, float far_x, float far_y);
private:
	
};

#endif
