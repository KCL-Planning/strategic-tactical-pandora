#ifndef SHAPE_SKY_BOX_H
#define SHAPE_SKY_BOX_H

#include "Shape.h"


class SkyBox : public Shape
{
public:
	SkyBox(float size);
	SkyBox(float width, float height, float depth);

private:
	void initialise(float width, float height, float depth);
};

#endif
