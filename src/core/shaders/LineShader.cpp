#include "LineShader.h"

#include <glm/gtc/type_ptr.hpp>

#include "../scene/SceneLeafModel.h"
#include "../scene/Material.h"
#include "../../shapes/Shape.h"

LineShader* LineShader::shader_ = NULL;
GLuint LineShader::modelview_matrix_loc_ = 0;
GLuint LineShader::projection_matrix_loc_ = 0;
GLuint LineShader::colour_loc_ = 0;

LineShader::LineShader(const std::string& vertex_shader, const std::string& fragment_shader)
	: GLSLProgram(vertex_shader, fragment_shader)
{

}

void LineShader::initialise(const SceneLeafLight& light_node, const glm::mat4& view_matrix, const glm::mat4& model_matrix, const glm::mat4& projection_matrix, const std::vector<const SceneLeafLight*>& lights)
{

}

void LineShader::initialise(const SceneLeafModel& model_node, const glm::mat4& view_matrix, const glm::mat4& model_matrix, const glm::mat4& projection_matrix, const std::vector<const SceneLeafLight*>& lights)
{
	std::map<const Shape*, GLuint>::iterator mapped_i = shape_to_vbo_.find(&model_node.getModel());
	GLuint vbo_index;
	if (mapped_i == shape_to_vbo_.end())
	{
		glGenVertexArrays(1, &vbo_index);
		glBindVertexArray(vbo_index);

		glEnableVertexAttribArray(0);
		glDisableVertexAttribArray(1);
		glDisableVertexAttribArray(2);
		glDisableVertexAttribArray(3);
		glDisableVertexAttribArray(4);
		glDisableVertexAttribArray(5);
		glDisableVertexAttribArray(6);
		glDisableVertexAttribArray(7);

		//Bind the vertex array and set the vertex pointer to point at it
		glBindBuffer(GL_ARRAY_BUFFER, model_node.getModel().getVertexBufferId());
		glVertexAttribPointer((GLint)0, 3, GL_FLOAT, GL_FALSE, 0, 0);
		
		//glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, model_node.getModel().getIndexBufferId());
		shape_to_vbo_[&model_node.getModel()] = vbo_index;
	}
	else
	{
		glBindVertexArray((*mapped_i).second);
	}

	if (last_used_shader_ != this)
	{
		bindShader();
/*
		glEnableVertexAttribArray(0);
		glDisableVertexAttribArray(1);
		glDisableVertexAttribArray(2);
		glDisableVertexAttribArray(3);
		glDisableVertexAttribArray(4);
		glDisableVertexAttribArray(5);
		glDisableVertexAttribArray(6);
		glDisableVertexAttribArray(7);
*/
	}

	glm::mat4 model_view_matrix = view_matrix * model_matrix;
	
	//Send the modelview and projection matrices to the shaders
	glUniformMatrix4fv(modelview_matrix_loc_, 1, false, glm::value_ptr(model_view_matrix));
	glUniformMatrix4fv(projection_matrix_loc_, 1, false, glm::value_ptr(projection_matrix));
	
	// Send the colour in which the line should be drawn.
	glUniform4f(colour_loc_, model_node.getMaterial().getEmissive().red_, model_node.getMaterial().getEmissive().green_, model_node.getMaterial().getEmissive().blue_, model_node.getMaterial().getEmissive().alpha_);
	
	//Bind the vertex array and set the vertex pointer to point at it
	//glBindBuffer(GL_ARRAY_BUFFER, model_node.getModel().getVertexBufferId());
	//glVertexAttribPointer((GLint)0, 3, GL_FLOAT, GL_FALSE, 0, 0);

	model_node.getModel().render();

	glBindVertexArray(0);
}

void LineShader::initialise(const glm::vec4& colour, const glm::mat4& view_matrix, const glm::mat4& model_matrix, const glm::mat4& projection_matrix, GLuint vertex_buffer_id)
{
	bindShader();

	glEnableVertexAttribArray(0);
	glDisableVertexAttribArray(1);
	glDisableVertexAttribArray(2);
	glDisableVertexAttribArray(3);
	glDisableVertexAttribArray(4);
	glDisableVertexAttribArray(5);
	glDisableVertexAttribArray(6);
	glDisableVertexAttribArray(7);

	glm::mat4 model_view_matrix = view_matrix * model_matrix;
	
	//Send the modelview and projection matrices to the shaders
	glUniformMatrix4fv(modelview_matrix_loc_, 1, false, glm::value_ptr(model_view_matrix));
	glUniformMatrix4fv(projection_matrix_loc_, 1, false, glm::value_ptr(projection_matrix));
	
	// Send the colour in which the line should be drawn.
	glUniform4f(colour_loc_, colour.r, colour.g, colour.b, colour.a);
	
	//Bind the vertex array and set the vertex pointer to point at it
	glBindBuffer(GL_ARRAY_BUFFER, vertex_buffer_id);
	glVertexAttribPointer((GLint)0, 3, GL_FLOAT, GL_FALSE, 0, 0);
}

LineShader& LineShader::getShader()
{
	if (shader_ == NULL)
	{
		shader_ = new LineShader("shaders/line.vert", "shaders/line.frag");
		
		// Load the shader.
		if (!shader_->initialize())
		{
			std::cout << "Failed to get the line shader!" << std::endl;
			exit(1);
		}
	
		//Bind the attribute locations
		shader_->bindAttrib(0, "a_Vertex");
//		shader_->bindAttrib(1, "a_Colour");

		//Re link the program
		shader_->linkProgram();
		shader_->bindShader();

		
		modelview_matrix_loc_ = shader_->getUniformLocation("modelview_matrix");
		projection_matrix_loc_ = shader_->getUniformLocation("projection_matrix");
	}
	return *shader_;
}
