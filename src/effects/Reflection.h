#ifndef REFLECTION_H
#define REFLECTION_H

#include "GL/glew.h"

class Camera;
class Sphere;
class Example;
class GLSLProgram;
class FreeTypeFont;
class LightManager;

class Reflection
{
public:
	Reflection(Example& example, Sphere& entity, float width, float height);
	void initialise();
	void prepare(float dt);
	void render(const Camera& cam, bool render_shadow, const LightManager& light_manager, GLSLProgram* shader);
private:
	Example* example;
	Sphere* entity;
	float x, y, z;
	float width, height;
	bool cubeMapInitialised;

	// The textures of each face are stored in a single texture which is 3x2 large. The layout 
	// is as follows.
	// *-----------------------------*
	// |   (1)   |   (0)   |   (0)   |
	// |   (0)   |   (1)   |   (0)   |
	// |   (0)   |   (0)   |   (1)   |
	// *-----------------------------*
	// |   (-1)  |   (0)   |   (0)   |
	// |   (0)   |   (-1)  |   (0)   |
	// |   (0)   |   (0)   |   (-1)  |
	// *-----------------------------*
	enum { Color, Depth, NumRenderbuffers };
	GLuint facesTexturesId;
	GLuint frameBuffers[6], renderBuffers[6 * NumRenderbuffers];
	Camera* cameras[6];

	FreeTypeFont* font;

	GLSLProgram* shader_;
};

#endif
