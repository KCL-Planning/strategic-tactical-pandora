#ifndef DEMO_APPLICATION_INTERFACE_H
#define DEMO_APPLICATION_INTERFACE_H

#include <GL/glew.h>
#include "Pandora.h"

namespace DreadedPE
{
	class Camera;
	class Texture;
};

class ApplicationInterface
{
public:
	virtual bool init(int argc, char** argv) = 0;
	virtual bool postInit() = 0;

	virtual GLuint postProcess(DreadedPE::Texture& color_texture, DreadedPE::Texture& depth_texture, float dt) = 0;

	virtual void tick(float dt) = 0;

	virtual DreadedPE::Camera& getCamera() const = 0;

	virtual void onResize(int width, int height) = 0;
};

#endif
