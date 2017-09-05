#ifndef CORE_RENDERER_GUI_RENDERER_H
#define CORE_RENDERER_GUI_RENDERER_H

#include <GL/glew.h>

class GUIManager;

class GUIRenderer
{
public:
	GUIRenderer(unsigned int fbo_id);

	void render(const GUIManager& frame);

	void onResize(int width, int height);
private:
	void initialise();

	// Off screen buffer.
	GLuint fbo_id_;

	float width_, height_;
};

#endif
