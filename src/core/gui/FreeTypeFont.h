#ifndef CORE_GUI_FONT_H
#define CORE_GUI_FONT_H

// FreeType Headers
#include <ft2build.h>

#include <freetype2/freetype/freetype.h>
#include <freetype2/freetype/ftglyph.h>
#include <freetype2/freetype/ftoutln.h>
#include <freetype2/freetype/fttrigon.h>

#include <string>
#include "GL/glew.h"

#include <glm/glm.hpp>

#include "../shaders/GUIShader.h"

class FreeTypeFont
{
public:
    FreeTypeFont(const std::string& fontName, int screenWidth, int screenHeight, int fontSize=16);
    ~FreeTypeFont();

    bool initialize();
    void printString(const glm::mat4& perspective_matrix, const std::string& str, float x, float y);

	/**
	 * Get the width of printing the text on screen.
	 */
	int getWidth(const std::string& text);

	/**
	 * Get the width of the given character.
	 */
	int getWidth(const char& c);

private:
    GLuint m_textureID[128]; //Store room for the character textures
    
    int m_fontSize;
    int m_screenWidth;
    int m_screenHeight;
    std::string m_fontName;

    GLuint m_TexCoordBuffer;
    GLuint m_vertexBuffer;

    bool generateCharacterTexture(unsigned char ch, FT_Face fontInfo);

	GUIShader* shader_;

    std::map<char, std::pair<int, int> > m_glyphDimensions;
    std::map<char, std::pair<int, int> > m_glyphPositions;
    std::map<char, int> m_glyphAdvances;
};

#endif
