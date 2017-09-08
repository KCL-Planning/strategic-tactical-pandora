#include "dpengine/texture/FreeImageLoader.h"
#include "dpengine/texture/Texture.h"

#ifdef _WIN32
#define NOMINMAX
#include <windows.h>
#endif

#include <iostream>
#include <sstream>

namespace DreadedPE
{

bool FreeImageLoader::free_image_loader_initialised_ = false;
std::map<std::string, Texture*> FreeImageLoader::cached_images_ = std::map<std::string, Texture*>();

void FreeImageErrorHandler(FREE_IMAGE_FORMAT fif, const char *message)
{
	std::cout << std::endl << "*** ";
	if(fif != FIF_UNKNOWN)
	{
		std::cout << FreeImage_GetFormatFromFIF(fif) << " Format" << std::endl;
	}
	std::cout << std::string(message) << " ***" << std::endl;
}

FIBITMAP* FreeImageLoader::getFiBitMap(const std::string& path)
{
	//std::cout << "FreeImageLoader::getFiBitMap(" << path << ");" << std::endl;
	FREE_IMAGE_FORMAT fif = FIF_UNKNOWN;
	// check the file signature and deduce its format
	// (the second argument is currently not used by FreeImage)
	fif = FreeImage_GetFileType(path.c_str(), 0);
	if(fif == FIF_UNKNOWN)
	{
		// no signature ?
		// try to guess the file format from the file extension
		fif = FreeImage_GetFIFFromFilename(path.c_str());
	}
	// check that the plugin has reading capabilities ...
	if((fif == FIF_UNKNOWN) || !FreeImage_FIFSupportsReading(fif))
	{
		std::cout << "Failed to load that image!" << std::endl;
		return NULL;
	}
	
	//std::cout << "Format: " << FreeImage_GetFormatFromFIF(fif) << std::endl;

	
	// ok, let's load the file
	return FreeImage_Load(fif, path.c_str());
}

Texture* FreeImageLoader::loadTexture(const std::string& path)
{
	//std::cout << "FreeImageLoader::loadTexture(" << path << ");" << std::endl;
	if (!free_image_loader_initialised_)
	{
		initialise();
	}
	
	if (cached_images_.find(path) != cached_images_.end())
	{
		//std::cout << "Return the cached texture (" << path << ");" << std::endl;
		return cached_images_[path];
	}
	
	// ok, let's load the file
	FIBITMAP *dib = getFiBitMap(path.c_str());
	if (dib == NULL)
	{
		std::cout << "Failed to load the image!" << std::endl;
		return NULL;
	}
	unsigned int width = FreeImage_GetWidth(dib);
	unsigned int height = FreeImage_GetHeight(dib);
	unsigned int bits_per_pixel = FreeImage_GetBPP(dib);
	
	//std::cout << "FreeImageLoader::loadTexture: " << width << ", " << height << "; BPP: " << bits_per_pixel << std::endl;
	
	if (height == 0 || width == 0)
	{
#ifdef _WIN32
		std::stringstream ss;
		ss << "Texture has no height / width.";
		//OutputDebugString(ss.str().c_str());
#else
		std::cout << "Texture has no height / width." << std::endl;
#endif
		return NULL;
	}

	Texture* texture = NULL;
	// Check if we have a 1D texture.
	if (height == 1)
	{
		texture = new Texture(GL_TEXTURE_1D);
		glBindTexture(GL_TEXTURE_1D, texture->getTextureId());
		
		glTexParameteri(GL_TEXTURE_1D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
		glTexParameteri(GL_TEXTURE_1D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

		if (bits_per_pixel == 24)
		{
			FIBITMAP *converted_dib = FreeImage_ConvertTo24Bits(dib);
			glTexImage1D(GL_TEXTURE_1D, 0, GL_RGB8, width, 0, GL_BGR, GL_UNSIGNED_BYTE, (void*)FreeImage_GetBits(converted_dib));
		}
		else
		{
			FIBITMAP *converted_dib = FreeImage_ConvertTo32Bits(dib);
			glTexImage1D(GL_TEXTURE_1D, 0, GL_RGBA8, width, 0, GL_BGRA, GL_UNSIGNED_BYTE, (void*)FreeImage_GetBits(converted_dib));
		}
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
		
		if (bits_per_pixel == 24)
		{
			FIBITMAP *converted_dib = FreeImage_ConvertTo24Bits(dib);
			//gluBuild2DMipmaps(GL_TEXTURE_2D, GL_RGB8, width, height, GL_RGB, GL_UNSIGNED_BYTE, (void*)FreeImage_GetBits(converted_dib));
			glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB8, width, height, 0, GL_BGR, GL_UNSIGNED_BYTE, (void*)FreeImage_GetBits(converted_dib));
		}
		else //if (bits_per_pixel == 32)
		{
			FIBITMAP *converted_dib = FreeImage_ConvertTo32Bits(dib);
			//gluBuild2DMipmaps(GL_TEXTURE_2D, GL_RGBA8, width, height, GL_RGBA, GL_UNSIGNED_BYTE, (void*)FreeImage_GetBits(converted_dib));
			glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA8, width, height, 0, GL_BGRA, GL_UNSIGNED_BYTE, (void*)FreeImage_GetBits(converted_dib));
		}
		/*
		else
		{
			exit(0);
		}
		*/
	}
	cached_images_[path] = texture;
	return texture;
}

Texture* FreeImageLoader::loadTexture(const std::string& left, const std::string& right, const std::string& top, const std::string& bottom, const std::string& back, const std::string& front)
{
	if (!free_image_loader_initialised_)
	{
		initialise();
	}
	
	std::stringstream ss;
	ss << left << " " << right << " " << top << " " << bottom << " " << back << " " << front;
	if (cached_images_.find(ss.str()) != cached_images_.end())
	{
		return cached_images_[ss.str()];
	}
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
		// ok, let's load the file
		FIBITMAP *dib = getFiBitMap(texture_files[i].c_str());
		
		if (dib == NULL)
		{
#ifdef _WIN32
			//OutputDebugString("Could not initialise the cube map!");
#endif
			return NULL;
		}
		
		unsigned int width = FreeImage_GetWidth(dib);
		unsigned int height = FreeImage_GetHeight(dib);
		unsigned int bits_per_pixel = FreeImage_GetBPP(dib);
		
		if (bits_per_pixel == 24)
		{
			FIBITMAP *converted_dib = FreeImage_ConvertTo24Bits(dib);
			gluBuild2DMipmaps(GL_TEXTURE_CUBE_MAP_POSITIVE_X + i, GL_RGB8, width, height, GL_RGB, GL_UNSIGNED_BYTE, (void*)FreeImage_GetBits(converted_dib));
		}
		else
		{
			FIBITMAP *converted_dib = FreeImage_ConvertTo32Bits(dib);
			gluBuild2DMipmaps(GL_TEXTURE_CUBE_MAP_POSITIVE_X + i, GL_RGBA8, width, height, GL_RGBA, GL_UNSIGNED_BYTE, (void*)FreeImage_GetBits(converted_dib));
		}
	}
	cached_images_[ss.str()] = texture;
	return texture;
}

void FreeImageLoader::unInitialise()
{
	FreeImage_DeInitialise();
}

void FreeImageLoader::initialise()
{
	std::cout << "FreeImageLoader::initialise()" << std::endl;
	FreeImage_Initialise();
	
	FreeImage_SetOutputMessage(FreeImageErrorHandler);
	
	const char* version_string = FreeImage_GetVersion();
	std::cout << std::string(version_string) << std::endl;

	
	free_image_loader_initialised_ = true;
}

};
