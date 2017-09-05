#ifndef BOGLGP_SPHERE_H
#define BOGLGP_SPHERE_H
#include <vector>

#include <glm/glm.hpp>

#include "GL/glew.h"
#include "Shape.h"

class Camera;
class GLSLProgram;
class LightManager;
class LightShader;

class Sphere : public Shape
{
public:
	Sphere(int slices, int stacks, float radius);

	void render();
private:
	int m_stacks;
    int m_slices;
};

#endif