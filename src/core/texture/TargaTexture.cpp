#include "TargaTexture.h"
#include "Texture.h"

#ifdef _WIN32
#include <windows.h>
#endif

#include <iostream>
#include <sstream>

std::map<std::string, Texture*> TargaTexture::cached_images_ = std::map<std::string, Texture*>();

Texture* TargaTexture::loadTexture(const std::string& path)
{
	// Check if this image has been loaded before.
	std::map<std::string, Texture*>::const_iterator found_map = cached_images_.find(path);
	if (found_map != cached_images_.end())
	{
		return (*found_map).second;
	}
	
	TargaImage image;
	if (!image.load(path))
	{
#ifdef _WIN32
		std::stringstream ss;
		ss << "File does not exists: " << path;
		MessageBox(NULL, ss.str().c_str(), "An error occurred", MB_ICONERROR | MB_OK);
#else
		std::cout << "File does not exists: " << path << std::endl;
#endif
		return NULL;
	}

	if (image.getHeight() == 0 || image.getWidth() == 0)
	{
#ifdef _WIN32
		std::stringstream ss;
		ss << "Texture has no height / width.";
		MessageBox(NULL, ss.str().c_str(), "An error occurred", MB_ICONERROR | MB_OK);
#else
		std::cout << "Texture has no height / width." << std::endl;
#endif
		return NULL;
	}

	Texture* texture = NULL;
	// Check if we have a 1D texture.
	if (image.getHeight() == 1)
	{
		texture = new Texture(GL_TEXTURE_1D);
		glBindTexture(GL_TEXTURE_1D, texture->getTextureId());
		
		glTexParameteri(GL_TEXTURE_1D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
		glTexParameteri(GL_TEXTURE_1D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

		if (image.getBitsPerPixel() == 24)
		{
			glTexImage1D(GL_TEXTURE_1D, 0, GL_RGB8, image.getWidth(), 0, GL_RGB, GL_UNSIGNED_BYTE, image.getImageData());
		}
		else
		{
			glTexImage1D(GL_TEXTURE_1D, 0, GL_RGBA8, image.getWidth(), 0, GL_RGBA, GL_UNSIGNED_BYTE, image.getImageData());
		}
		//glBindTexture(GL_TEXTURE_1D, 0);
	}
	// Otherwise we have a 2D texture.
	else
	{
		texture = new Texture(GL_TEXTURE_2D);
		glBindTexture(GL_TEXTURE_2D, texture->getTextureId());
		
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
		
		if (image.getBitsPerPixel() == 24)
		{
			gluBuild2DMipmaps(GL_TEXTURE_2D, GL_RGB8, image.getWidth(), image.getHeight(), GL_RGB, GL_UNSIGNED_BYTE, image.getImageData());
		}
		else
		{
			gluBuild2DMipmaps(GL_TEXTURE_2D, GL_RGBA8, image.getWidth(), image.getHeight(), GL_RGBA, GL_UNSIGNED_BYTE, image.getImageData());
		}
		//glBindTexture(GL_TEXTURE_2D, 0);
	}
	cached_images_[path] = texture;
	return texture;
}

Texture* TargaTexture::loadTexture(const std::string& left, const std::string& right, const std::string& top, const std::string& bottom, const std::string& back, const std::string& front)
{
	TargaImage image;
	Texture* texture = new Texture(GL_TEXTURE_CUBE_MAP);
	
	//glActiveTexture(active_texture_id_);	
	glBindTexture(GL_TEXTURE_CUBE_MAP, texture->getTextureId());
	//glActiveTexture(max_active_texture_id_ - 1);

	glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
	glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
	glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_R, GL_CLAMP_TO_EDGE);

	std::vector<std::string> texture_files;
	texture_files.push_back(left);
	texture_files.push_back(right);
	texture_files.push_back(top);
	texture_files.push_back(bottom);
	texture_files.push_back(front);
	texture_files.push_back(back);

	// Set the textures.
	for (unsigned int i = 0; i < 6; ++i)
	{
		TargaImage targa_loader;
		if (!targa_loader.load(texture_files[i]))
		{
#ifdef _WIN32
			MessageBox(NULL, "Could not initialise the cube map!", "Error", MB_OK);
#endif
			return NULL;
		}
		
		if (targa_loader.getBitsPerPixel() == 24)
		{
			gluBuild2DMipmaps(GL_TEXTURE_CUBE_MAP_POSITIVE_X + i, GL_RGB8, targa_loader.getWidth(), targa_loader.getHeight(), GL_RGB, GL_UNSIGNED_BYTE, targa_loader.getImageData());
		}
		else
		{
			gluBuild2DMipmaps(GL_TEXTURE_CUBE_MAP_POSITIVE_X + i, GL_RGBA8, targa_loader.getWidth(), targa_loader.getHeight(), GL_RGBA, GL_UNSIGNED_BYTE, targa_loader.getImageData());
		}
	}
	//glBindTexture(GL_TEXTURE_CUBE_MAP, 0);
	return texture;
}

