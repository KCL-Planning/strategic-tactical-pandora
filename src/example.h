#ifndef _EXAMPLE_H
#define _EXAMPLE_H

#include <vector>
#include "GL/glew.h"

#include "core/shaders/LightShader.h"
#include "shapes/Cube.h"
#include "shapes/terrain.h"
#include "core/CommonOpenGLStructs.h"

class Camera;
class Light;
class Sphere;
class Tree;
class Reflection;
class Water;
class Cube;
class LightManager;
class GodRays;
//class FreeTypeFont;

class Example 
{
public:
    Example();

    bool init();
    void prepare(float dt, const Camera& cam);
	void render(const Camera& cam, bool renderReflection, bool renderShadow, bool drawOpaqueSurfaces, bool drawTransparentSurfaces, bool drawLightEntities, bool drawToDefaultFrameBuffer = false, GLSLProgram* shader = NULL);
	void postProcess(const Camera& cam);
    void shutdown();

    void onResize(int width, int height);

	std::vector<std::string> getSupportedExtensions() const;

	bool isExtensionSupprted(const std::string& ext) const;

private:

	unsigned int fbo_id_;
	unsigned int texture_id_, depth_id_;

	bool checkError(const std::string& description) const;

	float m_rotationAngle, m_lightPosZ;
	float position;
	bool moveForward, moveLightForward;

	Cube* shadowMapCube;

	std::vector<Entity*> entities_;
	std::vector<Entity*> transparent_entities_;
	std::vector<Entity*> light_entities_;
	GLuint textures[15];
	LightShader* m_GLSLProgram;

	Reflection* reflection;
	LightManager* light_manager_;

	// Post effects.
	GodRays* god_rays_;

	// Font.
//	FreeTypeFont* font;
};

#endif
