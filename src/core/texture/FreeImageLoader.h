#ifndef CORE_TEXTURE_FREE_IMAGE_LOADER_H
#define CORE_TEXTURE_FREE_IMAGE_LOADER_H

#include <GL/glew.h>
#include <string>
#include <map>
#include <FreeImage.h>

class Texture;

/**
 * This class manages the texture units in OpenGL.
 */
class FreeImageLoader
{
public:
	static Texture* loadTexture(const std::string& path);

	// Cube map.
	static Texture* loadTexture(const std::string& left, const std::string& right, const std::string& top, const std::string& bottom, const std::string& back, const std::string& front);

	static void unInitialise();
private:
	
	static FIBITMAP* getFiBitMap(const std::string& path);
	
	static void initialise();
	
	static bool free_image_loader_initialised_;
	static std::map<std::string, Texture*> cached_images_;
};

#endif
