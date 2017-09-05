#ifndef CORE_LOADERS_TEXTURE_H
#define CORE_LOADERS_TEXTURE_H

#include <GL/glew.h>
#include <string>
#include <vector>

/**
 * This class manages the texture units in OpenGL.
 */
class Texture
{
public:
	/**
	 * Create a texture, this takes care of creating a texture ID and active texture ID.
	 */
	Texture(GLenum target);
	virtual ~Texture();

	bool isComplete() const { return texture_complete_; }

	GLenum getTextureType() const { return target_; }
	GLuint getTextureId() const { return texture_id_; }
	GLuint getActiveTextureId() const { return active_texture_id_; }
	
	/**
	 * Set active texture to a value that will never be used for an actual texture.
	 */
	static void unbindActiveTexture();
	
	/**
	 * Initialise the texture class. This needs to happen prior to loading and using textures.
	 */
	static void initialise();
	
private:
	/**
	 * Reserve an active texture ID.
	 * @return The id that is the static active texture id, -1 if no such ID could be assigned.
	 */
	GLint reserveActiveTextureID();
	
	/**
	 * Release the active texture ID of this texture.
	 */
	void releaseActiveTextureID();

protected:
	static GLint max_active_texture_id_;
	
	bool texture_complete_;         // Debug flag, check to make sure the texture has loaded successfully.
	GLenum target_;                 // The texture type, GL_TEXTURE_1D, GL_TEXTURE_2D, etc.
	GLuint texture_id_;             // The id that the texture has been assigned in OpenGL.
	GLuint active_texture_id_;      // We assigned texture statically to an active texture to avoid state changes whilst rendering.
	
	static std::vector<Texture*> loaded_textures_;
	static std::vector<GLuint> free_active_texture_ids_; 
};

#endif
