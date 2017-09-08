#include "dpengine/texture/Texture.h"

#ifdef _WIN32
#define NOMINMAX
#include <windows.h>
#endif

#include <iostream>
#include <sstream>
#include <iostream>

namespace DreadedPE
{

GLint Texture::max_active_texture_id_ = 0;
std::vector<Texture*> Texture::loaded_textures_;
std::vector<GLuint> Texture::free_active_texture_ids_; 

Texture::Texture(GLenum target)
	: target_(target)
{
	glGenTextures(1, &texture_id_);
	active_texture_id_ = reserveActiveTextureID();
	texture_complete_ = active_texture_id_ != -1;

	if (!texture_complete_)
	{
#ifdef _WIN32
		OutputDebugString("INVALID TEXTURE!");
#else
		std::cerr << "INVALID TEXTURE!" << std::endl;
#endif
	}
#ifdef _WIN32
	std::stringstream ss;
	ss << "Texture: " << active_texture_id_ <<  "= " << (active_texture_id_ - GL_TEXTURE0) << "(" << free_active_texture_ids_.size() << ")." << std::endl;
	OutputDebugString(ss.str().c_str());
#else
	std::cout << "Texture: " << active_texture_id_ <<  "= " << (active_texture_id_ - GL_TEXTURE0) << "(" << free_active_texture_ids_.size() << ")." << std::endl;
#endif
	
	// Bind the texture to the active texture id. This will remain like this until the texture is unloaded.
	glActiveTexture(GL_TEXTURE0 + active_texture_id_);
	glBindTexture(target, texture_id_);
	unbindActiveTexture();
	
	loaded_textures_.push_back(this);
}

Texture::~Texture()
{
	releaseActiveTextureID();
	glDeleteTextures(1, &texture_id_);
}

void Texture::initialise()
{
	glGetIntegerv(GL_MAX_COMBINED_TEXTURE_IMAGE_UNITS, &max_active_texture_id_);
	//max_active_texture_id_ = GL_TEXTURE0;
	
	loaded_textures_.resize(0);
	free_active_texture_ids_.resize(0);
#ifdef _WIN32
	std::stringstream ss;
	ss << "Create active texture IDs: " << max_active_texture_id_ << "." << std::endl;
	OutputDebugString(ss.str().c_str());
#else
	std::cout << "Create active texture IDs: " << max_active_texture_id_ << "." << std::endl;
#endif

	for (int i = 0; i < max_active_texture_id_; ++i)
	{
		free_active_texture_ids_.push_back(i);
	}
}

void Texture::unbindActiveTexture()
{
#ifdef _WIN32
	std::stringstream ss;
	ss << "Unbind texture..." << std::endl;
	OutputDebugString(ss.str().c_str());
#else
	std::cout << "Unbind texture..." << std::endl;
#endif
	glActiveTexture(GL_TEXTURE0 + max_active_texture_id_ - 1);
}

GLint Texture::reserveActiveTextureID()
{
	if (free_active_texture_ids_.empty())
	{
		return -1;
	}
	GLint id = free_active_texture_ids_[0];
	free_active_texture_ids_.erase(free_active_texture_ids_.begin());
	return id;
}
	
void Texture::releaseActiveTextureID()
{
	free_active_texture_ids_.push_back(getActiveTextureId());
}

};
