#include "GUIShader.h"

#include <glm/gtc/type_ptr.hpp>

#include "../gui/GUIElement.h"
#include "../gui/Container.h"
#include "../gui/themes/Theme.h"
#include "../scene/Material.h"
#include "../texture/Texture.h"
#include "../gui/fonts/Font.h"

GUIShader* GUIShader::shader_ = NULL;

GUIShader::GUIShader(const std::string& vertex_shader, const std::string& fragment_shader)
	: GLSLProgram(vertex_shader, fragment_shader)
{
	
}

void GUIShader::renderOutline(const Container& container, const glm::mat4& model_matrix, const glm::mat4& projection_matrix)
{
	// Check if this shape has been rendered using this shader before.
	std::map<const GUIElement*, GLuint>::iterator mapped_i = gui_element_to_vbo_.find(&container);
	GLuint vbo_index;
	if (mapped_i == gui_element_to_vbo_.end())
	{
		glGenVertexArrays(1, &vbo_index);
		glBindVertexArray(vbo_index);

		glEnableVertexAttribArray(0);
		glEnableVertexAttribArray(1);
		glDisableVertexAttribArray(2);
		glDisableVertexAttribArray(3);
		glDisableVertexAttribArray(4);
		glDisableVertexAttribArray(5);
		glDisableVertexAttribArray(6);
		glDisableVertexAttribArray(7);

		glBindBuffer(GL_ARRAY_BUFFER, container.getVertexBufferId());
		glVertexAttribPointer((GLint)0, 3, GL_FLOAT, GL_FALSE, 0, 0);

		glBindBuffer(GL_ARRAY_BUFFER, container.getTextureCoordinatesBufferId());
		glVertexAttribPointer((GLint)1, 2, GL_FLOAT, GL_FALSE, 0, 0);
		
		gui_element_to_vbo_[&container] = vbo_index;
	}
	else
	{
		glBindVertexArray((*mapped_i).second);
	}

	if (last_used_shader_ != this)
	{
		bindShader();
	}

	glm::mat4 model_view_matrix = model_matrix;
	glUniform1i(texture0_loc_, container.getTexture().getActiveTextureId());
	
	//Send the modelview and projection matrices to the shaders
	glUniformMatrix4fv(modelview_matrix_loc_, 1, false, glm::value_ptr(model_view_matrix));
	glUniformMatrix4fv(projection_matrix_loc_, 1, false, glm::value_ptr(projection_matrix));

	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, container.getIndexBufferId());
	glDrawElements(GL_TRIANGLES, container.getIndices().size(), GL_UNSIGNED_INT, 0);

	glBindVertexArray(0);
}


void GUIShader::renderContainer(const Container& container, const glm::mat4& model_matrix, const glm::mat4& projection_matrix)
{
	/*
	// Check if this shape has been rendered using this shader before.
	std::map<const GUIElement*, GLuint>::iterator mapped_i = gui_element_to_vbo_.find(&container);
	GLuint vbo_index;
	if (mapped_i == gui_element_to_vbo_.end())
	{
	
		glGenVertexArrays(1, &vbo_index);
		glBindVertexArray(vbo_index);
		*/
		glEnableVertexAttribArray(0);
		glEnableVertexAttribArray(1);
		glDisableVertexAttribArray(2);
		glDisableVertexAttribArray(3);
		glDisableVertexAttribArray(4);
		glDisableVertexAttribArray(5);
		glDisableVertexAttribArray(6);
		glDisableVertexAttribArray(7);

		glBindBuffer(GL_ARRAY_BUFFER, container.getCombinedVertexBufferId());
		glVertexAttribPointer((GLint)0, 3, GL_FLOAT, GL_FALSE, 0, 0);

		glBindBuffer(GL_ARRAY_BUFFER, container.getCombinedTextureCoordinatesBufferId());
		glVertexAttribPointer((GLint)1, 2, GL_FLOAT, GL_FALSE, 0, 0);
		/*
		gui_element_to_vbo_[&container] = vbo_index;
	}
	else
	{
		glBindVertexArray((*mapped_i).second);
	}
	*/
	if (last_used_shader_ != this)
	{
		bindShader();
	}

	glm::mat4 model_view_matrix = model_matrix;
	glUniform1i(texture0_loc_, container.getTexture().getActiveTextureId());
	
	//Send the modelview and projection matrices to the shaders
	glUniformMatrix4fv(modelview_matrix_loc_, 1, false, glm::value_ptr(model_view_matrix));
	glUniformMatrix4fv(projection_matrix_loc_, 1, false, glm::value_ptr(projection_matrix));

	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, container.getCombinedIndexBufferId());
	glDrawElements(GL_TRIANGLES, container.getCombinedIndices().size(), GL_UNSIGNED_INT, 0);

	glBindVertexArray(0);
}

void GUIShader::renderFont(const Font& font, const glm::mat4& model_matrix, const glm::mat4& projection_matrix)
{
	// Check if this shape has been rendered using this shader before.
	std::map<const Font*, GLuint>::iterator mapped_i = font_to_vbo_.find(&font);
	GLuint vbo_index;
	if (mapped_i == font_to_vbo_.end())
	{
		glGenVertexArrays(1, &vbo_index);
		glBindVertexArray(vbo_index);

		glEnableVertexAttribArray(0);
		glEnableVertexAttribArray(1);
		glDisableVertexAttribArray(2);
		glDisableVertexAttribArray(3);
		glDisableVertexAttribArray(4);
		glDisableVertexAttribArray(5);
		glDisableVertexAttribArray(6);
		glDisableVertexAttribArray(7);

		glBindBuffer(GL_ARRAY_BUFFER, font.getVertexBufferId());
		glVertexAttribPointer((GLint)0, 3, GL_FLOAT, GL_FALSE, 0, 0);

		glBindBuffer(GL_ARRAY_BUFFER, font.getTexCoordBufferId());
		glVertexAttribPointer((GLint)1, 2, GL_FLOAT, GL_FALSE, 0, 0);
		
		font_to_vbo_[&font] = vbo_index;
	}
	else
	{
		glBindVertexArray((*mapped_i).second);
	}
	
	if (last_used_shader_ != this)
	{
		bindShader();
	}

	glm::mat4 model_view_matrix = model_matrix;

	glUniform1i(texture0_loc_, font.getTexture().getActiveTextureId());
	
	//Send the modelview and projection matrices to the shaders
	glUniformMatrix4fv(modelview_matrix_loc_, 1, false, glm::value_ptr(model_view_matrix));
	glUniformMatrix4fv(projection_matrix_loc_, 1, false, glm::value_ptr(projection_matrix));

	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, font.getIndexBufferId());
	glDrawElements(GL_TRIANGLES, font.getIndices().size(), GL_UNSIGNED_INT, 0);

	glBindVertexArray(0);
}

GUIShader& GUIShader::getShader()
{
	if (shader_ == NULL)
	{
		shader_ = new GUIShader("shaders/font.vert", "shaders/font.frag");
		
		// Load the shader.
		if (!shader_->initialize())
		{
			std::cout << "Failed to get the GUI shader!" << std::endl;
			exit(1);
		}
	
		//Bind the attribute locations
		shader_->bindAttrib(0, "a_Vertex");
		shader_->bindAttrib(1, "a_TexCoord0");

		//Re link the program
		shader_->linkProgram();
		shader_->bindShader();

		shader_->modelview_matrix_loc_ = shader_->getUniformLocation("modelview_matrix");
		shader_->projection_matrix_loc_ = shader_->getUniformLocation("projection_matrix");
		shader_->texture0_loc_ = shader_->getUniformLocation("texture0");
	}
	return *shader_;
}
