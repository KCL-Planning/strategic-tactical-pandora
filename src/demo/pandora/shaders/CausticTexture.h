#ifndef DEMO_PANDORA_SHADERS_CAUSTIC_TEXTURE_H
#define DEMO_PANDORA_SHADERS_CAUSTIC_TEXTURE_H

#include <GL/glew.h>

#include "../../../core/texture/Texture.h"

/**
 * Class that handles the caustic texture and updates the texture that should be used accordingly. The images used are
 * hardcoded for now.
 */
class CausticTexture : public Texture
{
public:
	CausticTexture();

	void update(float dt);

	GLuint getTextureToDisplayIndex() const;
private:
	float total_time_;
};

#endif
