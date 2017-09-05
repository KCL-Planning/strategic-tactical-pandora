#ifndef CORE_GUI_FONTS_TEXTURED_FONT_H
#define CORE_GUI_FONTS_TEXTURED_FONT_H

#include <vector>
#include <string>

#include <GL/glew.h>
#include <glm/glm.hpp>

#include "../../../shapes/Shape.h"
#include "Font.h"
#include "../GUIElement.h"

class Texture;

/**
 * Create a font based on a texture. The texture should be in a uniform 6x6 grid as follows:
 * A B C D E F
 * G H I J K L
 * M N O P Q R
 * S T U V W X
 * Y Z 0 1 2 3
 * 4 5 6 7 8 9
 */
class TexturedFont : public Font
{
public:
	/**
	 * Create a Text Banner with the given text.
	 * @param texture The 6x6 texture which contains the letters and numbers.
	 */
	TexturedFont(Texture& texture);
	
	/**
	 * @param string The string that should be set for this font. 
	 * @param font_size The size (in pixels) that font should be rendered as.
	 */
	void setString(const std::string& string, float font_size);

	/**
	 * Append a string to the font.
	 * @param text The text to append to the existing string.
	 * @param font_size The font size this text should be drawn at.
	 */
	void appendString(const glm::mat4& model_translation, const std::string& text, float font_size);
	
	/**
	 * @param perspective_matrix The perspective matrix used to render the font.
	 * @param model_matrix The matrix that transposes the font on screen.
	 */
	void draw(const glm::mat4& perspective_matrix, const glm::mat4& model_matrix);

	/**
	 * Update the fonts after the window has been resized.
	 */
	void onResize(int width, int height);

	/**
	 * Create a clone of this font object.
	 */
	Font& clone();
private:
	/**
	 * Untility function that loads the UV mapping for a character.
	 * @param ch The character to create the UV mapping for.
	 */
	void setUVMapping(const char& ch);

	std::vector<std::string> all_text_; // All added text.
	std::vector<glm::mat4> model_translations_; // All model translations to the added texts.
	std::vector<float> font_sizes_; // All the font sizes of the added texts.
};

#endif
