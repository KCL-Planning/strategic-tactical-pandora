#define WIN32_LEAN_AND_MEAN
#define WIN32_EXTRA_LEAN

#define GLX_GLXEXT_LEGACY //Must be declared so that our local glxext.h is picked up, rather than the system one
#define GLFW_NO_GLU // Do not allow GL/glfw to include the gl.h header

//#define HORROR_GAME_ENABLE_DEBUG

#include <ros/ros.h>
#include <std_msgs/String.h>

#include <vector>
#include <string>
#include <iostream>

#include "GL/glew.h"
#include <dpengine/game/Game.h>
#include <dpengine/gui/GUIManager.h>
#include <dpengine/renderer/Window.h>
#include <dpengine/renderer/CameraRenderer.h>
#include <dpengine/scene/SceneManager.h>
#include <dpengine/texture/Texture.h>

#include <time.h>
#include <unistd.h>

#include "demo/Pandora.h"

int main(int argc, char** argv)
{
	if (!glfwInit())
	{
#ifdef _WIN32
        MessageBox(NULL, "Unable to initialise GLFW.", "An error occurred", MB_ICONERROR | MB_OK);
#endif
		return 1;
	}
	std::cout << "GLFW loaded!" << std::endl;

	int width = 1024;
	int height = 768;
	DreadedPE::Window* window = DreadedPE::Window::createWindow(width, height, "PANDORA", false);
	
	if (window == NULL)
	{
		std::cerr << "Failed to open the window. Check that your GPU supports OpenGL 4.3." << std::endl;
		glfwTerminate();
		return 1;
	}
	{
		std::stringstream ss;
		ss << glGetString(GL_VERSION);
		window->setTitle(ss.str());
	}

	// Initialise the extention library.
	GLenum err = glewInit();
	if (err != GLEW_OK)
	{
		std::cerr << "Failed to initialise GLEW: " << glewGetErrorString(err) << std::endl;
		exit(1);
	}
	std::cout << "GLEW loaded!" << std::endl;

	// Initialise the texture framework.
	DreadedPE::Texture::initialise();
	DreadedPE::SceneManager scene_manager;
	
	// Initialise the frustrum culling geometries.
	std::vector<const DreadedPE::SceneNode*> excluded_nodes;
	scene_manager.getRoot().initialiseBoundedBoxes(excluded_nodes);
	
	Pandora* pandora = new Pandora(scene_manager);
	
	if (!pandora->init(argc, argv))
	{
#ifdef _WIN32
		MessageBox(NULL, "Could not initialize the application", "An error occurred", MB_ICONERROR | MB_OK);
#endif
		glfwTerminate();
		return 1;
	}
	std::cout << "Example initialised!" << std::endl;

	scene_manager.getRoot().initialiseBoundedBoxes(excluded_nodes);

	if (!pandora->postInit())
	{
#ifdef _WIN32
        MessageBox(NULL, "Could not post initialize the application", "An error occurred", MB_ICONERROR | MB_OK);
#endif
        glfwTerminate();
		return 1;
	}
	std::cout << "Example post initialised!" << std::endl;
	
	DreadedPE::Game game(0.15f);
	
	game.addGameComponent(*pandora);
	game.addGameComponent(DreadedPE::GUIManager::getInstance());
	game.addGameComponent(scene_manager);
	game.run();
	return EXIT_SUCCESS;

	/*
	bool running = true;

	double m_lastTime = getWallTime();

	float pre_render_time = 0;
	float render_time = 0;
	float final_pre_render_time = 0;
	float final_render_time = 0;
	float player_prepare_time = 0;
	float player_detection_time = 0;

	//glfwGetWindowSize(window, &width_, &height_);
	//glfwSetCursorPos(window, width_ / 2, height_ / 2);

	double last_time = getWallTime();
	double last_fps_time = getWallTime();
	
	unsigned int frame_rendered = 0;
	unsigned int last_frames_rendered = 0;

	bool paused = false;
	bool show_collision_boxes = false;
	bool enable_post_processing = false;
	
	// Initialise the GUI.
	DreadedPE::Texture* frame_texture = DreadedPE::TargaTexture::loadTexture("data/textures/gui.tga");

	DreadedPE::GUIManager& gui_manager = DreadedPE::GUIManager::getInstance();
	gui_renderer_ = new DreadedPE::GUIRenderer(renderer_->getFramebufferId());
	
	//glm::mat4 perspective_matrix = glm::ortho(0.0f, 1024.0f, 0.0f, 768.0f, -1.0f, 1.0f);
	
	while (running)
	{
		if (window->isKeyPressed(GLFW_KEY_P))
		{
			paused = !paused;
		}
		if (window->isKeyPressed(GLFW_KEY_C))
		{
			show_collision_boxes = !show_collision_boxes;
			renderer_->setShowCollisionBoxes(show_collision_boxes);
		}
		if (window->isKeyPressed(GLFW_KEY_H))
		{
			renderer_->setRenderMode(GL_LINE);
		}
		if (window->isKeyPressed(GLFW_KEY_F))
		{
			renderer_->setRenderMode(GL_FILL);
		}
		if (window->isKeyPressed(GLFW_KEY_K))
		{
			enable_post_processing = true;
		}
		if (window->isKeyPressed(GLFW_KEY_L))
		{
			enable_post_processing = false;
		}

///		mPlatform->getRenderManagerPtr()->drawOneFrame();

		double currentTime = getWallTime();
		
		float delta = float(currentTime - m_lastTime);
//		m_lastTime = currentTime;
	//	delta *= 10;
		if (!is_planning)	
		{
			delta *= speedup;
		}

		std::stringstream ss;
		//Entity& player = example.getPlayer();
		double time = getWallTime();
		
		float total_delta = delta;

		// Always update the GUI.
		gui_manager.update(delta);

		if (!paused)
		{
			//if (delta < 0.01f) continue;
			while (delta >= 0)
			{
				scene_manager.prepare(std::min(0.15f, delta));
				example_->tick(std::min(0.15f, delta));
				delta -= 0.15f;
				
				if (window->isKeyPressed(GLFW_KEY_1))
				{
					speedup = 1;
				}
				if (window->isKeyPressed(GLFW_KEY_2))
				{
					speedup = 2;
				}
				if (window->isKeyPressed(GLFW_KEY_3))
				{
					speedup = 4;
				}
				if (window->isKeyPressed(GLFW_KEY_4))
				{
					speedup = 8;
				}
				if (window->isKeyPressed(GLFW_KEY_5))
				{
					speedup = 16;
				}
				if (window->isKeyPressed(GLFW_KEY_6))
				{
					speedup = 32;
				}
				if (window->isKeyPressed(GLFW_KEY_7))
				{
					speedup = 64;
				}
				if (window->isKeyPressed(GLFW_KEY_8))
				{
					speedup = 128;
				}
				if (window->isKeyPressed(GLFW_KEY_9))
				{
					speedup = 256;
				}
				
				
			}
			//particle_system.prepare(total_delta);
		}
		// Even if pauzed we want the camera to work.
		else
		{
//			example.getCameraNode().prepare(delta);
		}

		m_lastTime = currentTime;

		pre_render_time += getWallTime() - time;
		time = getWallTime();

		const DreadedPE::Camera& camera = example_->getCamera();
		renderer_->render(camera);
		render_time = getWallTime() - time;
		
		// Draw the GUI.
		gui_renderer_->render(gui_manager);
		
		GLuint frame_buffer = example_->postProcess(renderer_->getTexture(), renderer_->getDepth(), delta);
		if (frame_buffer != 0)
		{
			glBindFramebuffer(GL_READ_FRAMEBUFFER, frame_buffer);
		}
		else
		{
			glBindFramebuffer(GL_READ_FRAMEBUFFER, renderer_->getFramebufferId());
		}
		
		glBindFramebuffer(GL_DRAW_FRAMEBUFFER, 0);

		// Copy the off screen buffer to the main buffer to draw.
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
		//glBlitFramebuffer(0, 0, 1024, 768, 0, 0, 1024, 768, GL_COLOR_BUFFER_BIT, GL_NEAREST);
		glBlitFramebuffer(0, 0, example_->getCamera().getWidth(), example_->getCamera().getHeight(), 0, 0, example_->getCamera().getWidth(), example_->getCamera().getHeight(), GL_COLOR_BUFFER_BIT, GL_NEAREST);
		glBindFramebuffer(GL_FRAMEBUFFER, 0);
		
		double current_time = getWallTime();
		
		if (current_time - last_fps_time > 1.0f)
		{
			last_fps_time = current_time;
			last_frames_rendered = frame_rendered;
			frame_rendered = 0;
			final_pre_render_time = pre_render_time;
			final_render_time = render_time;
			pre_render_time = 0;
			render_time = 0;
		}
		++frame_rendered;

		glfwSwapBuffers(window);
		running = !glfwWindowShouldClose(window) && glfwGetKey(window, GLFW_KEY_ESCAPE) != GLFW_PRESS;
		//running = !glfwGetKey( GLFW_KEY_ESC ) && glfwGetWindowParam( GLFW_OPENED );
	}
	glfwTerminate();
	*/
}
