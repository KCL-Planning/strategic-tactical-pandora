#include <iostream>

#include "dpengine/gui/fonts/TexturedFont.h"

#include "dpengine/texture/Texture.h"
#include "dpengine/texture/TargaTexture.h"
#include "dpengine/shaders/GUIShader.h"
#include "dpengine/renderer/Window.h"

namespace DreadedPE
{

TexturedFont::TexturedFont(Texture& texture)
	: Font(texture)
{
	
}

void TexturedFont::setString(const std::string& string, float font_size)
{
	//std::cout << "[TexturedFont::setString]" << std::endl;
	clearText();
	all_text_.clear();
	model_translations_.clear();
	font_sizes_.clear();
	m_vertices_.clear();
	m_normals_.clear();
	m_indices_.clear();

	glm::mat4 identity_matrix(1.0f);
	appendString(identity_matrix, string, font_size);
}

void TexturedFont::appendString(const glm::mat4& model_translation, const std::string& text, float font_size)
{
	//std::cout << "[TexturedFont::appendString]" << std::endl;
	all_text_.push_back(text);
	model_translations_.push_back(model_translation);
	font_sizes_.push_back(font_size);

	Window* window = Window::getActiveWindow();
	int width, height;
	window->getSize(width, height);
	
	unsigned int index_offset = m_vertices_.size();
	for (unsigned int i = 0; i < text.size(); ++i)
	{
		setUVMapping(text.at(i));
		
		glm::vec4 v0(font_size * i, height, 0.0f, 1.0f);
		glm::vec4 v1(font_size * i + font_size, height, 0.0f, 1.0f);
		glm::vec4 v2(font_size * i + font_size, height - font_size, 0.0f, 1.0f);
		glm::vec4 v3(font_size * i, height - font_size, 0.0f, 1.0f);

		v0 = model_translation * v0;
		v1 = model_translation * v1;
		v2 = model_translation * v2;
		v3 = model_translation * v3;

		m_vertices_.push_back(glm::vec3(v0));
		m_vertices_.push_back(glm::vec3(v1));
		m_vertices_.push_back(glm::vec3(v2));
		m_vertices_.push_back(glm::vec3(v3));
		
		m_normals_.push_back(glm::vec3(0, 0, -1));
		m_normals_.push_back(glm::vec3(0, 0, -1));
		m_normals_.push_back(glm::vec3(0, 0, -1));
		m_normals_.push_back(glm::vec3(0, 0, -1));
		
		m_indices_.push_back(index_offset + i * 4);
		m_indices_.push_back(index_offset + i * 4 + 3);
		m_indices_.push_back(index_offset + i * 4 + 1);
		
		m_indices_.push_back(index_offset + i * 4 + 1);
		m_indices_.push_back(index_offset + i * 4 + 3);
		m_indices_.push_back(index_offset + i * 4 + 2);
		//std::cout << "Append string: " << text << " (" << v0.x << ", " << v0.y << ", " << v0.z << ")" << std::endl;
	}
	markBuffersForUpdate();
}

void TexturedFont::draw(const glm::mat4& perspective_matrix, const glm::mat4& model_matrix)
{
	finaliseBuffers();
	GUIShader& shader = GUIShader::getShader();
	/*
	unsigned int vertex_index = 0;
	for (unsigned int i = 0; i < all_text_.size(); ++i)
	{
		if (all_text_[i].size() == 0) continue;
		glm::vec4 loc(m_vertices_[vertex_index].x, m_vertices_[vertex_index].y, m_vertices_[vertex_index].z, 1);
		glm::vec4 v = perspective_matrix * model_matrix * loc;
		std::cout << "Render: " << all_text_[i] << " at: (" << loc.x << ", " << loc.y << ", " << loc.z << ", " << loc.w << ")" << std::endl;
		
		for (unsigned int y = 0; y < 4; ++y)
		{
			std::cout << "[";
			for (unsigned int x = 0; x < 4; ++x)
			{
				std::cout << model_matrix[x][y] << " ";
			}
			std::cout << "]" << std::endl;
		}
		
		std::cout << "Perspective matrix: " << all_text_[i] << " (" << perspective_matrix[3][0] << ", " << perspective_matrix[3][1] << ", " << perspective_matrix[3][2] << ")" << std::endl;
		std::cout << "V: " << all_text_[i] << " (" << v.x << ", " << v.y << ", " << v.z << ", " << v.w << ")" << std::endl;
		vertex_index += all_text_[i].size() * 4;
	}
	*/
	shader.renderFont(*this, model_matrix, perspective_matrix);
}

void TexturedFont::setUVMapping(const char& ch)
{
	float cell_size = 1.0f / 6.0f;
	
	int cell_number = ch;
	// Is it a number?
	if (ch >= 48 && ch <= 57)
	{
		cell_number -= (48 - 26);
	}
	// Is it a literal?
	else if ((ch >= 65 && ch <= 90) || (ch >= 97 && ch <= 122))
	{
		cell_number -= (ch <= 90 ? 65 : 97);
	}
	// Space
	else if (ch == 32)
	{
		m_tex_coords_.push_back(glm::vec2(0.0f, 0.0f));
		m_tex_coords_.push_back(glm::vec2(0.0f, 0.0f));
		m_tex_coords_.push_back(glm::vec2(0.0f, 0.0f));
		m_tex_coords_.push_back(glm::vec2(0.0f, 0.0f));
		return;
	}
	else
	{
		//std::cerr << "Unknown character: " << ch << std::endl;
		// Defaults 'unknown' character texture.
		m_tex_coords_.push_back(glm::vec2(0.0f, 0.0f));
		m_tex_coords_.push_back(glm::vec2(0.0f, 0.0f));
		m_tex_coords_.push_back(glm::vec2(0.0f, 0.0f));
		m_tex_coords_.push_back(glm::vec2(0.0f, 0.0f));
		return;
	}
	
	int row_nr = 5 - (cell_number / 6);
	int column_nr = cell_number % 6;
	
	//std::cout << ch << "[" << (int)ch << "]1 -> (" << column_nr << ", " << row_nr << ")" << std::endl;
	
	m_tex_coords_.push_back(glm::vec2(column_nr * cell_size, row_nr * cell_size + cell_size));
	m_tex_coords_.push_back(glm::vec2(column_nr * cell_size + cell_size, row_nr * cell_size + cell_size));
	m_tex_coords_.push_back(glm::vec2(column_nr * cell_size + cell_size, row_nr * cell_size));
	m_tex_coords_.push_back(glm::vec2(column_nr * cell_size, row_nr * cell_size));
}

void TexturedFont::onResize(int width, int height)
{
	m_vertices_.clear();
	for (unsigned int i = 0; i < all_text_.size(); ++i)
	{
		const std::string& text = all_text_[i];
		for (unsigned j = 0; j < text.size(); ++j)
		{
			glm::vec4 v0(font_sizes_[i] * j, height, 0.0f, 1.0f);
			glm::vec4 v1(font_sizes_[i] * j + font_sizes_[i], height, 0.0f, 1.0f);
			glm::vec4 v2(font_sizes_[i] * j + font_sizes_[i], height - font_sizes_[i], 0.0f, 1.0f);
			glm::vec4 v3(font_sizes_[i] * j, height - font_sizes_[i], 0.0f, 1.0f);
			
			v0 = model_translations_[i] * v0;
			v1 = model_translations_[i] * v1;
			v2 = model_translations_[i] * v2;
			v3 = model_translations_[i] * v3;

			m_vertices_.push_back(glm::vec3(v0));
			m_vertices_.push_back(glm::vec3(v1));
			m_vertices_.push_back(glm::vec3(v2));
			m_vertices_.push_back(glm::vec3(v3));
		}
	}
	markBuffersForUpdate();
}

Font& TexturedFont::clone()
{
	TexturedFont* clone = new TexturedFont(*font_texture_);
	return *clone;
}

};
