#ifdef _WIN32
#include <Windows.h>
#endif
#include <iostream>
#include <cmath>

#include "CausticTexture.h"

#include <sstream>

#include "dpengine/texture/TargaTexture.h"

CausticTexture::CausticTexture()
	: DreadedPE::Texture(GL_TEXTURE_2D_ARRAY), total_time_(0)
{
	glBindTexture(GL_TEXTURE_2D_ARRAY, texture_id_);

	glTexImage3D(GL_TEXTURE_2D_ARRAY, 0, GL_RGB8, 512, 512, 31, 0, GL_RGB, GL_UNSIGNED_BYTE, NULL);

	glTexParameteri(GL_TEXTURE_2D_ARRAY, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D_ARRAY, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D_ARRAY, GL_TEXTURE_WRAP_S, GL_REPEAT);
	glTexParameteri(GL_TEXTURE_2D_ARRAY, GL_TEXTURE_WRAP_T, GL_REPEAT);

	// Next we load all textures.
	for (unsigned int i = 0; i < 31; ++i)
	{
		std::stringstream texture_path;
		texture_path << "data/textures/caustic/save.";
		if ((i + 1) < 10) texture_path << "0";
		texture_path << (i + 1) << ".tga";

		DreadedPE::TargaImage image;
		if (!image.load(texture_path.str()))
		{
#ifdef _WIN32
			std::stringstream ss;
			ss << "File does not exists: " << texture_path.str();
			MessageBox(NULL, ss.str().c_str(), "An error occurred", MB_ICONERROR | MB_OK);
#else
			std::cout << "File does not exists: " << texture_path.str() << std::endl;
#endif
			return;
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
			return;
		}

		// Load this image.
		glTexSubImage3D(GL_TEXTURE_2D_ARRAY, 0, 0, 0, i, image.getWidth(), image.getHeight(), 1, GL_RGB, GL_UNSIGNED_BYTE, image.getImageData());
	}
}

void CausticTexture::update(float dt)
{
	total_time_ += dt;
}

GLuint CausticTexture::getTextureToDisplayIndex() const
{
	float time_mode = fmod(total_time_, 2.0f);
	time_mode *= 16.0f;
	return floor(time_mode);
}
