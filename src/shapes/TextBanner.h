#ifndef SHAPES_TEXT_BANNER_H
#define SHAPES_TEXT_BANNER_H

#include <string>

#include "Shape.h"

class Texture;

/**
 * Display a bit of text on a flat surface.
 */
class TextBanner : public Shape
{
public:
	/**
	 * Create a Text Banner with the given text.
	 */
	TextBanner(const std::string& text, const Texture& font_texture, float font_size);
private:
	
	void setUVMapping(const char& ch);
	
	std::string text_;
	const Texture* font_texture_;
};

#endif
