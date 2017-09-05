#ifndef CORE_GUI_CANVAS_H
#define CORE_GUI_CANVAS_H

#include "Container.h"

class Theme;
class Font;
class Texture;

/**
 * A GUI element that only exists to render an image.
 */
class Canvas : public Container
{
public:
	Canvas(const Theme& theme, Font& font, int x, int y, int size_x, int size_y, Texture& texture);
private:
	
};

#endif
