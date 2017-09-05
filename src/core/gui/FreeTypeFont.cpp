#include "FreeTypeFont.h"

#include <glm/gtc/matrix_transform.hpp> 

FreeTypeFont::FreeTypeFont(const std::string& fontName, int screenWidth, int screenHeight, int fontSize) 
{
    m_fontName = fontName;
    m_screenWidth = screenWidth;
    m_screenHeight = screenHeight;
    m_fontSize = fontSize;

	shader_ = &GUIShader::getShader();
}

FreeTypeFont::~FreeTypeFont()
{
    glDeleteTextures(128, m_textureID);
}

bool FreeTypeFont::initialize()
{
    FT_Library library; //Create a freetype library instance

    if (FT_Init_FreeType(&library)) {
        std::cerr << "Could not initialize the freetype library" << std::endl;
        return false;
    }

    FT_Face fontInfo;  //Stores information on the loaded font

    //Now we attempt to load the font information
    if(FT_New_Face(library, m_fontName.c_str(), 0, &fontInfo)) {
        std::cerr << "Could not load the specified font" << std::endl;
        return false;
    }

    //FreeType uses heights which are one 64th of the size in pixels so 
    //we set our font height by multiplying by 64. The 96x96 is the dots per inch
    FT_Set_Char_Size(fontInfo, (int)m_fontSize * 64, (int)m_fontSize * 64, 96, 96);

    //Generate 128 textures (each character gets its own texture)
    glGenTextures(128, m_textureID);

    for (unsigned char ch = 0; ch < 128; ++ch) 
    {
        if (!generateCharacterTexture(ch, fontInfo))
        {
            std::cerr << "Could not generate the texture for character: " << ch << std::endl;
            return false;
        }
    }

    FT_Done_Face(fontInfo);
    FT_Done_FreeType(library);

    float vertices [] = {
        0.0f, 0.0f, 0.0f,
        (float)m_fontSize, 0.0f, 0.0f,
        (float)m_fontSize, (float)m_fontSize, 0.0f,
        0.0f, (float)m_fontSize, 0.0f,
    };

    glGenBuffers(1, &m_vertexBuffer);
    glBindBuffer(GL_ARRAY_BUFFER, m_vertexBuffer);
    glBufferData(GL_ARRAY_BUFFER, sizeof(GLfloat) * 12, &vertices[0], GL_STATIC_DRAW);

    //Just initialize with something for now, the tex coords are updated
    //for each character printed
    float TexCoords [] = {
        0.0f, 1.0f,
        1.0f, 1.0f, 
        1.0f, 0.0f,
        0.0f, 0.0f
    };

    glGenBuffers(1, &m_TexCoordBuffer);
    glBindBuffer(GL_ARRAY_BUFFER, m_TexCoordBuffer);
    glBufferData(GL_ARRAY_BUFFER, sizeof(GLfloat) * 8, &TexCoords[0], GL_DYNAMIC_DRAW);
	
    return true;
}

bool FreeTypeFont::generateCharacterTexture(unsigned char ch, FT_Face fontInfo)
{
    if(FT_Load_Glyph(fontInfo, FT_Get_Char_Index(fontInfo, ch), FT_LOAD_DEFAULT))
    {
        return false;
    }

  	FT_Glyph glyph;
    if(FT_Get_Glyph(fontInfo->glyph, &glyph))
    {
        return false;
    }

    if (FT_Glyph_To_Bitmap(&glyph, ft_render_mode_normal, 0, 1)) 
    {
        return false;
    }

    FT_BitmapGlyph bitmapGlyph = (FT_BitmapGlyph) glyph;

    int width = (bitmapGlyph->bitmap.width) ? bitmapGlyph->bitmap.width : 16;
    int rows = (bitmapGlyph->bitmap.rows) ? bitmapGlyph->bitmap.rows : 16;

    //Allocate space for our font texture (R, G, B, A)
    int imageSize = width * rows * 4;

    vector<unsigned char> imageData(imageSize);

    for (int i = 0; i < imageSize / 4; i++) 
    {
        unsigned char gray = 0;
        if (bitmapGlyph->bitmap.buffer) 
        {
            gray = bitmapGlyph->bitmap.buffer[i];
        } 

        
		imageData[i*4] = gray;
		imageData[(i*4)+1] = 0;
        imageData[(i*4)+2] = 0;
        imageData[(i*4)+3] = gray;
		//imageData[i*4] = gray;
        //imageData[(i*4)+1] = gray;
        //imageData[(i*4)+2] = gray;
        //imageData[(i*4)+3] = gray;
    }

    glBindTexture(GL_TEXTURE_2D, m_textureID[ch]);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, width, rows, 0, GL_RGBA, GL_UNSIGNED_BYTE, &imageData[0]);

    m_glyphDimensions[ch] = std::make_pair(width, rows);
    m_glyphPositions[ch] = std::make_pair(bitmapGlyph->left, bitmapGlyph->top);
    m_glyphAdvances[ch] = fontInfo->glyph->advance.x / 64;
    return true;
}

void FreeTypeFont::printString(const glm::mat4& perspective_matrix, const std::string& str, float x, float y)
{
	glPolygonMode( GL_FRONT_AND_BACK, GL_FILL);
	glDisable(GL_CULL_FACE);
	glDisable(GL_DEPTH_TEST);

	glm::mat4 model_matrix(1.0f);
	model_matrix = glm::translate(model_matrix, glm::vec3(x, y, 0.0f));
	for(string::size_type i = 0; i < str.size(); ++i) 
	{
		int ch = int(str[i]);

		float vertices [] = {
			0.0f, 0.0f, 0.0f,
			(float)m_glyphDimensions[ch].first, 0.0f, 0.0f,
			(float)m_glyphDimensions[ch].first, (float)m_glyphDimensions[ch].second, 0.0f,
			0.0f, (float)m_glyphDimensions[ch].second, 0.0f,
		};

		glm::mat4 local_model_matrix = glm::translate(model_matrix, glm::vec3((float)m_glyphPositions[ch].first, (float)m_glyphPositions[ch].second - m_glyphDimensions[ch].second, 0.0f));

		glBindBuffer(GL_ARRAY_BUFFER, m_vertexBuffer);
		glBufferSubData(GL_ARRAY_BUFFER, 0, sizeof(float) * 12, &vertices[0]);
			
		shader_->initialise(m_textureID[ch], m_vertexBuffer, m_TexCoordBuffer, local_model_matrix, perspective_matrix);

		glBindBuffer(GL_ARRAY_BUFFER, m_vertexBuffer);
		glDrawArrays(GL_QUADS, 0, 4);

		model_matrix = glm::translate(model_matrix, glm::vec3((float)m_glyphAdvances[ch], 0.0f, 0.0f));
	}
	glEnable(GL_DEPTH_TEST);
}

int FreeTypeFont::getWidth(const std::string& text) 
{
	int width = 0;
	for(string::size_type i = 0; i < text.size(); ++i) 
    {
        int ch = int(text[i]);
		//width += m_glyphDimensions[ch].first + m_glyphAdvances[ch];
		width += m_glyphAdvances[ch];
	}
	return width;
}

int FreeTypeFont::getWidth(const char& c)
{
	//return m_glyphDimensions[c].first + m_glyphAdvances[c];
	return m_glyphAdvances[c];
}
