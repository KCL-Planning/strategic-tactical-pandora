#ifndef CORE_TEXTURE_TARGA_TEXTURE_H
#define CORE_TEXTURE_TARGA_TEXTURE_H

#include <GL/glew.h>
#include <string>
#include <map>

#include "../loaders/targa.h"

class Texture;

/**
 * This class manages the texture units in OpenGL.
 */
class TargaTexture
{
public:
	static Texture* loadTexture(const std::string& path);

	// Cube map.
	static Texture* loadTexture(const std::string& left, const std::string& right, const std::string& top, const std::string& bottom, const std::string& back, const std::string& front);

private:
	static std::map<std::string, Texture*> cached_images_;
};

#endif
