#ifndef CORE_SHADERS_GUI_SHADER_H
#define CORE_SHADERS_GUI_SHADER_H

#include <glm/glm.hpp>
#include <vector>

#include "glslshader.h"

class GUIElement;
class Container;
class Font;

class GUIShader : public GLSLProgram
{
public:
	/**
	 * Render the outline of the container, this is used to fill the stencil buffer.
	 */
	void renderOutline(const Container& container, const glm::mat4& model_matrix, const glm::mat4& projection_matrix);

	/**
	 * Render a container and all elements that a direct child of that container.
	 */
	void renderContainer(const Container& container, const glm::mat4& model_matrix, const glm::mat4& projection_matrix);

	//void initialise(const Texture& texture, GLuint vertex_buffer_id, GLuint texture_buffer_id, const glm::mat4& model_matrix, const glm::mat4& projection_matrix);
	/**
	 * Render a piece of text.
	 */
	void renderFont(const Font& font, const glm::mat4& model_matrix, const glm::mat4& projection_matrix);

	static GUIShader& getShader();

protected:
	GLuint modelview_matrix_loc_, projection_matrix_loc_;
	GUIShader(const std::string& vertex_shader, const std::string& fragment_shader);
	
private:
	static GUIShader* shader_;
	GLuint texture0_loc_;

	std::map<const Font*, GLuint> font_to_vbo_;
	std::map<const GUIElement*, GLuint> gui_element_to_vbo_;
};

#endif
