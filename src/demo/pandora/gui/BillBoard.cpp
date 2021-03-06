#include "BillBoard.h"

#include <GLFW/glfw3.h>

#include "dpengine/entities/camera/Camera.h"
#include "dpengine/shaders/GUIShader.h"
#include "dpengine/texture/TargaTexture.h"
#include <dpengine/renderer/Window.h>

BillBoard::BillBoard(const DreadedPE::Theme& theme, DreadedPE::Font& font, DreadedPE::SceneNode& to_follow, DreadedPE::Camera& camera, const glm::vec3& offset, float width, float height, const std::vector<glm::vec2>& tex_coords)
	: DreadedPE::Container(theme, font, 0, 0, width, height, true), blink_(false), blink_visible_(true), time_to_next_blink_(0), to_follow_(&to_follow), camera_(&camera), offset_(offset)
{
	int screen_width, screen_height;
	DreadedPE::Window* window = DreadedPE::Window::getActiveWindow();
	window->getSize(screen_width, screen_height);
/*
	m_vertices_.push_back(glm::vec3(0, screen_height, 0));
	m_vertices_.push_back(glm::vec3(width, screen_height, 0));
	m_vertices_.push_back(glm::vec3(0, screen_height - height, 0));
	m_vertices_.push_back(glm::vec3(width, screen_height - height, 0));
*/
	m_vertices_.push_back(glm::vec3(0, 0, 0));
	m_vertices_.push_back(glm::vec3(width, 0, 0));
	m_vertices_.push_back(glm::vec3(0, height, 0));
	m_vertices_.push_back(glm::vec3(width, height, 0));

	m_tex_coords_ = tex_coords;

	m_indices_.push_back(1);
	m_indices_.push_back(0);
	m_indices_.push_back(2);

	m_indices_.push_back(1);
	m_indices_.push_back(2);
	m_indices_.push_back(3);
	
	texture_ = DreadedPE::TargaTexture::loadTexture("data/textures/icons.tga");
}

void BillBoard::setUVMapping(const std::vector<glm::vec2>& uv)
{
	m_tex_coords_ = uv;
	need_to_update_buffers_ = true;
}

void BillBoard::update(float dt)
{
	DreadedPE::Container::update(dt);
	markForUpdate();
	if (blink_)
	{
		time_to_next_blink_ -= dt;
		if (time_to_next_blink_ < -0.5f)
		{
			time_to_next_blink_ = 0.5f + (time_to_next_blink_ + 0.5f);
			blink_visible_ = true;
		}
		else if (time_to_next_blink_ < 0)
		{
			blink_visible_ = false;
			return;
		}
		else
		{
			blink_visible_ = true;
		}
	}
	/*
	int width, height;
	DreadedPE::Window* window = DreadedPE::Window::getActiveWindow();
	window->getSize(width, height);
	
	float scale = 1.0f;
	// Transform the location of the scene node to screen coordinates. This will yield values between -1 and 1.
	glm::vec4 location_on_screen = camera_->getPerspectiveMatrix() * camera_->getViewMatrix() * glm::vec4(to_follow_->getGlobalLocation() + offset_, 1.0f);
	
	scale = 10.0f / location_on_screen.z;
	if (scale < 0.1f) scale = 0.1f;
	if (scale > 1.0f) scale = 1.0f;
	
	location_on_screen /= location_on_screen.w;
	//std::cout << "(" << to_follow_->getGlobalLocation().x << ", " << to_follow_->getGlobalLocation().y << ", " << to_follow_->getGlobalLocation().z << ") >>>=====> (" << location_on_screen.x << ", " << location_on_screen.y << ", " << location_on_screen.z << ", " << location_on_screen.w << ")" << std::endl;
	
	// Don't draw icons for billboards that are behind the camera.
	if (location_on_screen.z < -1 || location_on_screen.z > 1)
	{
		return;
	}
	
	glm::vec3 location_on_screen_3d((location_on_screen.x * 0.5f + 0.5f) * width - size_x_ / 2.0f, -(1 - (location_on_screen.y * 0.5f + 0.5f)) * height + size_y_ / 2.0f, 0.0f);
	location_ = location_on_screen_3d;
	
	combined_vertices_.clear();
	
	float scale_x_offset = (size_x_- size_x_ * scale) / 2.0f;
	float scale_y_offset = (size_y_- size_y_ * scale) / 2.0f;
	combined_vertices_.push_back(glm::vec3(scale_x_offset, height - scale_y_offset, 0));
	combined_vertices_.push_back(glm::vec3(scale_x_offset + size_x_ * scale, height - scale_y_offset, 0));
	combined_vertices_.push_back(glm::vec3(scale_x_offset, height - (size_y_ * scale) - scale_y_offset, 0));
	combined_vertices_.push_back(glm::vec3(scale_x_offset + size_x_ * scale, height - (size_y_ * scale) - scale_y_offset, 0));
	glBindBuffer(GL_ARRAY_BUFFER, combined_vertex_buffer_);
	glBufferData(GL_ARRAY_BUFFER, sizeof(GLfloat) * 3 * combined_vertices_.size(), &combined_vertices_[0], GL_DYNAMIC_DRAW);
	*/
}

void BillBoard::draw(const glm::mat4& perspective_matrix, int level) const
{
	if (!is_visible_) return;
	if (blink_ && !blink_visible_) return;
	
	int width, height;
	DreadedPE::Window* window = DreadedPE::Window::getActiveWindow();
	window->getSize(width, height);
	
	float scale = 1.0f;
	// Transform the location of the scene node to screen coordinates. This will yield values between -1 and 1.
	glm::vec4 location_on_screen = camera_->getPerspectiveMatrix() * camera_->getViewMatrix() * glm::vec4(to_follow_->getGlobalLocation() + offset_, 1.0f);
	
	scale = 10.0f / location_on_screen.z;
	if (scale < 0.1f) scale = 0.1f;
	if (scale > 1.0f) scale = 1.0f;
	
	location_on_screen /= location_on_screen.w;
	//std::cout << "(" << to_follow_->getGlobalLocation().x << ", " << to_follow_->getGlobalLocation().y << ", " << to_follow_->getGlobalLocation().z << ") >>>=====> (" << location_on_screen.x << ", " << location_on_screen.y << ", " << location_on_screen.z << ", " << location_on_screen.w << ")" << std::endl;
	
	// Don't draw icons for billboards that are behind the camera.
	if (location_on_screen.z < -1 || location_on_screen.z > 1)
	{
		return;
	}
	
	glm::vec3 location_on_screen_3d((location_on_screen.x * 0.5f + 0.5f) * width - size_x_ / 2.0f, -(1 - (location_on_screen.y * 0.5f + 0.5f)) * height + size_y_ / 2.0f, 0.0f);
	
	std::vector<glm::vec3> vertices;
	
	float scale_x_offset = (size_x_- size_x_ * scale) / 2.0f;
	float scale_y_offset = (size_y_- size_y_ * scale) / 2.0f;
	vertices.push_back(glm::vec3(scale_x_offset, height - scale_y_offset, 0));
	vertices.push_back(glm::vec3(scale_x_offset + size_x_ * scale, height - scale_y_offset, 0));
	vertices.push_back(glm::vec3(scale_x_offset, height - (size_y_ * scale) - scale_y_offset, 0));
	vertices.push_back(glm::vec3(scale_x_offset + size_x_ * scale, height - (size_y_ * scale) - scale_y_offset, 0));
	glBindBuffer(GL_ARRAY_BUFFER, combined_vertex_buffer_);
	glBufferData(GL_ARRAY_BUFFER, sizeof(GLfloat) * 3 * vertices.size(), &vertices[0], GL_DYNAMIC_DRAW);
	
	// Draw the container and its contents.	
	DreadedPE::GUIShader& shader = DreadedPE::GUIShader::getShader();
	shader.renderContainer(*this, glm::translate(glm::mat4(1.0f), location_on_screen_3d), perspective_matrix);
}

void BillBoard::onResize(int width, int height)
{
	m_vertices_.clear();
	m_vertices_.push_back(glm::vec3(0, height, 0));
	m_vertices_.push_back(glm::vec3(size_x_, height, 0));
	m_vertices_.push_back(glm::vec3(0, height - size_y_, 0));
	m_vertices_.push_back(glm::vec3(size_x_, height - size_y_, 0));
	updateBuffers();
}
