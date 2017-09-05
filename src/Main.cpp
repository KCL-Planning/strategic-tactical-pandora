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

#include "core/collision/BoxCollision.h"
#include "GL/glew.h"
#include "GL/glfw.h"
#include "core/entities/camera/Camera.h"
#include "core/entities/camera/DynamicCamera.h"

#include "core/entities/Player.h"
#include "core/entities/Bridge.h"

#include "core/renderer/SimpleRenderer.h"
#include "core/scene/frustum/Frustum.h"

#include "core/scene/SceneLeaf.h"
#include "core/scene/frustum/SphereCheck.h"

#include "core/shaders/UnderWaterShader.h"

#include <time.h>
#ifdef _WIN32
#include <windows.h>
#endif
#ifdef __linux
#include <unistd.h>
#endif

#ifdef HORROR_GAME_ENABLE_DEBUG
// Debug callback function for OpenGL.
void APIENTRY DebugCallbackFunction(GLenum source, GLenum type, GLuint id, GLenum severity, GLsizei length, const GLchar *message, GLvoid *userParam)
{
	std::stringstream ss;
	ss << message << std::endl;
#ifdef _WIN32
	OutputDebugString(ss.str().c_str());
#else
	std::cout << ss.str() << std::endl;
#endif
}
#endif


void sleepMain(int ms)
{
#ifdef __linux
    usleep(ms * 1000);   // usleep takes sleep time in us (1 millionth of a second)
#endif
#ifdef _WIN32
    Sleep(ms);
#endif
}

#include "demo/TestScene1.h"
#include "demo/FlatDemo.h"
#include "demo/LoaderExample.h"
#include "demo/PLFDemo.h"
#include "demo/FBXLoaderDemo.h"
#include "demo/GUITester.h"
#include "demo/VolumetricLightDemo.h"
#include "demo/ParticleTest.h"
#include "demo/ZombieHorde.h"
#include "demo/AnimationTest.h"
#include "demo/FallingBlock.h"

#include "demo/Pandora.h"
//#include "demo/PandoraPillarExperiment.h"
#include "demo/pandora/AUV.h"
#include "demo/pandora/OpportunityGenerator.h"
#include "demo/RegionTest/RegionWorld.h"

#include "demo/shooter/ArmedPlayer.h"
#include "core/light/Light.h"
#include "core/scene/frustum/Frustum.h"
#include "core/scene/portal/Portal.h"

#include <glm/gtx/rotate_vector.hpp>
#include <glm/gtc/matrix_transform.hpp> 
#include <glm/gtc/quaternion.hpp>
#include <glm/gtx/quaternion.hpp>

#include "core/entities/Player.h"

#include "core/scene/SceneManager.h"
#include "core/models/AnimatedModel.h"
#include "core/models/Animation.h"

#include "core/math/BoundedBox.h"
#include "core/math/Plane.h"

#include "core/entities/Monster.h"
#include "core/texture/Texture.h"

///
// GUI stuff.
///
#include "core/gui/Frame.h"
#include "core/gui/Button.h"
#include "core/gui/GUIElement.h"
#include "core/renderer/GUIRenderer.h"
#include "core/texture/TargaTexture.h"
#include "core/gui/themes/MyGUITheme.h"
#include "core/gui/GUIManager.h"
#include "core/math/Math.h"

#include "demo/ApplicationInterface.h"

float speedup = 1.0f;
bool is_planning = false;

#ifdef _WIN32
double getWallTime()
{
    LARGE_INTEGER time,freq;
    if (!QueryPerformanceFrequency(&freq))
    {
        //  Handle error
        return 0;
    }
    if (!QueryPerformanceCounter(&time))
    {
        //  Handle error
        return 0;
    }
    return (double)time.QuadPart / freq.QuadPart;
}

double getCPUTime()
{
    FILETIME a,b,c,d;
    if (GetProcessTimes(GetCurrentProcess(),&a,&b,&c,&d) != 0)
    {
        //  Returns total user time.
        //  Can be tweaked to include kernel times as well.
        return
            (double)(d.dwLowDateTime |
            ((unsigned long long)d.dwHighDateTime << 32)) * 0.0000001;
    }
    else
    {
        //  Handle error
        return 0;
    }
}
//  Posix/Linux
#else
#include <sys/time.h>
double getWallTime()
{
    struct timeval time;
    if (gettimeofday(&time,NULL))
    {
        //  Handle error
        return 0;
    }
    return (double)time.tv_sec + (double)time.tv_usec * .000001;
}

double getCPUTime()
{
    return (double)clock() / CLOCKS_PER_SEC;
}
#endif

ApplicationInterface* example_ = NULL;
SimpleRenderer* renderer_ = NULL;
GUIRenderer* gui_renderer_ = NULL;
int width_ = 0;
int height_ = 0;

void GLFWCALL onResize(int width, int height)
{
	width_ = width;
	height_ = height;
	glViewport(0, 0, width_, height_);
	if (example_ != NULL)
	{
		example_->onResize(width, height);
	}
	if (renderer_ != NULL)
	{
		renderer_->onResize(width, height);
	}
	if (gui_renderer_ != NULL)
	{
		gui_renderer_->onResize(width, height);
	}
	GUIManager::getInstance().onResize(width, height);
}

void receiveStatus(const std_msgs::StringConstPtr& msg)
{
	is_planning = "planning" == msg->data;
}

#ifdef _WIN32
int WINAPI WinMain(HINSTANCE hInstance,
                   HINSTANCE hPrevInstance,
                   LPSTR cmdLine,
                   int cmdShow)
{
	int argc = 0;
	char** argv = NULL;
#else
int main(int argc, char** argv)
{
	ros::init(argc, argv, "PlannerVisualisation");
	ros::NodeHandle ros_nh;
	
	ros::Subscriber sub = ros_nh.subscribe("/planning_system/state", 1, &receiveStatus);
	
#endif
	if (!glfwInit())
	{
#ifdef _WIN32
        MessageBox(NULL, "Unable to initialise GLFW.", "An error occurred", MB_ICONERROR | MB_OK);
#endif
		return 1;
	}
	std::cout << "GLFW loaded!" << std::endl;

#ifdef HORROR_GAME_ENABLE_DEBUG
	// Enable debuging.
	glfwOpenWindowHint(GLFW_OPENGL_DEBUG_CONTEXT, GL_TRUE);
#endif
	
	if (!glfwOpenWindow(1024, 768, 16, 16, 16, 32, 24, 8, GLFW_WINDOW))
	{
#ifdef _WIN32
        	MessageBox(NULL, "Could not open window.", "An error occurred", MB_ICONERROR | MB_OK);
#endif
		glfwTerminate();
		return 1;
	}
	{
		std::stringstream ss;
		ss << glGetString(GL_VERSION);
		glfwSetWindowTitle(ss.str().c_str());
	}
	std::cout << "Opened the screen!" << std::endl;
	std::cout << "Stencil: " << glfwGetWindowParam(GLFW_STENCIL_BITS) << std::endl;

	glfwSetWindowSizeCallback(onResize);
	//glfwDisable(GLFW_MOUSE_CURSOR);

	// Initialise the extention library.
	GLenum err = glewInit();
	if (err != GLEW_OK)
	{
		std::stringstream ss;
		ss << glewGetErrorString(err);
#ifdef _WIN32
		MessageBox(NULL, ss.str().c_str(), "An error occurred", MB_ICONERROR | MB_OK);
#endif
		fprintf(stderr, "Error %s\n", glewGetErrorString(err));
		exit(1);
	}
	std::cout << "GLEW loaded!" << std::endl;

	// Initialise the texture framework.
	Texture::initialise();

#ifdef HORROR_GAME_ENABLE_DEBUG
	if (glewIsSupported("GL_ARB_debug_output"))
	{
		glEnable(GL_DEBUG_OUTPUT_SYNCHRONOUS_ARB);
		glEnable(GL_DEBUG_OUTPUT);

		if (glDebugMessageCallback != 0)
		{
			glDebugMessageCallback(DebugCallbackFunction, NULL);
		}
		else if (glDebugMessageCallbackARB != 0)
		{
			glDebugMessageCallbackARB(DebugCallbackFunction, NULL);
		}
		else
		{
#ifdef _WIN32
			MessageBox(NULL, "NO DEBUG :(!", "An error occurred", MB_ICONERROR | MB_OK);
#else
			std::cout << "No debug :(!" << std::endl;
#endif
		}
		// Enable all messages.
		glDebugMessageControlARB(GL_DONT_CARE, GL_DONT_CARE, GL_DEBUG_SEVERITY_HIGH, 0, NULL, GL_TRUE);
	}
	else
	{
#ifdef _WIN32
		MessageBox(NULL, "NO DEBUG :(!", "An error occurred", MB_ICONERROR | MB_OK);
#else
		std::cout << "NO DEBUG :(!" << std::endl;
#endif
	}
#endif

	SceneManager scene_manager;
	renderer_ = new SimpleRenderer(scene_manager);
	//example_ = new TestScene(scene_manager);
	//example_ = new FlatDemo(scene_manager);
	//example_ = new AnimationTest(scene_manager);
	//example_ = new PLFDemo(scene_manager);
	//example_ = new FallingBlock(scene_manager);
	//example_ = new RegionWorld(scene_manager);
	//example_ = new FBXLoaderDemo(scene_manager);
	example_ = new Pandora(scene_manager);
	//example_ = new OpportunityGenerator(scene_manager);
	//example_ = new PandoraPillarExperiment(scene_manager);
	//example_ = new VolumetricLightDemo(scene_manager);
	//example_ = new GUITester(scene_manager);
	//example_ = new ParticleTest(scene_manager);
	//example_ = new ZombieHorde(scene_manager);
	
	if (!example_->init(argc, argv)) //Initialize our example
	{
#ifdef _WIN32
		MessageBox(NULL, "Could not initialize the application", "An error occurred", MB_ICONERROR | MB_OK);
#endif
		glfwTerminate();
		return 1;
	}
	std::cout << "Example initialised!" << std::endl;

	std::vector<const SceneNode*> excluded_nodes;
	scene_manager.getRoot().initialiseBoundedBoxes(excluded_nodes);

	if (!example_->postInit())
	{
#ifdef _WIN32
        MessageBox(NULL, "Could not post initialize the application", "An error occurred", MB_ICONERROR | MB_OK);
#endif
        glfwTerminate();
		return 1;
	}
	std::cout << "Example post initialised!" << std::endl;

	
	// Initialise the frustrum culling geometries.
	//std::vector<const SceneNode*> excluded_nodes;
	scene_manager.getRoot().initialiseBoundedBoxes(excluded_nodes);

	scene_manager.getRoot().compress();

	bool running = true;

	double m_lastTime = getWallTime();

	float pre_render_time = 0;
	float render_time = 0;
	float final_pre_render_time = 0;
	float final_render_time = 0;
	float player_prepare_time = 0;
	float player_detection_time = 0;

	glfwGetWindowSize(&width_, &height_);
	glfwSetMousePos(width_ / 2, height_ / 2);

	double last_time = getWallTime();
	double last_fps_time = getWallTime();
	
	unsigned int frame_rendered = 0;
	unsigned int last_frames_rendered = 0;

	bool paused = false;
	bool show_collision_boxes = false;
	bool enable_post_processing = false;
	
	// Initialise the GUI.
	Texture* frame_texture = TargaTexture::loadTexture("data/textures/gui.tga");

	GUIManager& gui_manager = GUIManager::getInstance();
	gui_renderer_ = new GUIRenderer(renderer_->getFramebufferId());
	
	//glm::mat4 perspective_matrix = glm::ortho(0.0f, 1024.0f, 0.0f, 768.0f, -1.0f, 1.0f);
	
	while (running)
	{
		if (glfwGetKey('P') == GLFW_PRESS)
		{
			paused = !paused;
		}
		if (glfwGetKey('C') == GLFW_PRESS)
		{
			show_collision_boxes = !show_collision_boxes;
			renderer_->setShowCollisionBoxes(show_collision_boxes);
		}
		if (glfwGetKey('H') == GLFW_PRESS)
		{
			renderer_->setRenderMode(GL_LINE);
		}
		if (glfwGetKey('F') == GLFW_PRESS)
		{
			renderer_->setRenderMode(GL_FILL);
		}
		if (glfwGetKey('K') == GLFW_PRESS)
		{
			enable_post_processing = true;
		}
		if (glfwGetKey('L') == GLFW_PRESS)
		{
			enable_post_processing = false;
		}
		
		/*
		if (glfwGetKey(GLFW_KEY_LALT) == GLFW_PRESS)
		{
			glfwEnable(GLFW_MOUSE_CURSOR);
		}
		else
		{
			// Reset mouse pointer.
			glfwDisable(GLFW_MOUSE_CURSOR);
		}
		*/

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
				
				if (glfwGetKey('1') == GLFW_PRESS)
				{
					speedup = 1;
				}
				if (glfwGetKey('2') == GLFW_PRESS)
				{
					speedup = 2;
				}
				if (glfwGetKey('3') == GLFW_PRESS)
				{
					speedup = 4;
				}
				if (glfwGetKey('4') == GLFW_PRESS)
				{
					speedup = 8;
				}
				if (glfwGetKey('5') == GLFW_PRESS)
				{
					speedup = 16;
				}
				if (glfwGetKey('6') == GLFW_PRESS)
				{
					speedup = 32;
				}
				if (glfwGetKey('7') == GLFW_PRESS)
				{
					speedup = 64;
				}
				if (glfwGetKey('8') == GLFW_PRESS)
				{
					speedup = 128;
				}
				if (glfwGetKey('9') == GLFW_PRESS)
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
/*
		// Test the picking module.
		Entity* picked_entity = NULL;
		if (glfwGetMouseButton(GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS)
		{
			int mouseX, mouseY;
			glfwGetMousePos(&mouseX, &mouseY);

			picked_entity = scene_manager.pickEntity(example_->getCamera(), mouseX, mouseY);
		}
*/
		const Camera& camera = example_->getCamera();
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
		/*
		if (picked_entity != NULL)
		{
			ss << "Picked entity: " << picked_entity->getName() << "|";
		}
		else
		{
			ss << "No picked entity." << std::endl;
		}
		*/

		/*
		ss << last_frames_rendered;// << "|";
		ss << std::endl;
#ifdef _WIN32
		OutputDebugString(ss.str().c_str());
#else
		std::cout << ss.str() << std::endl;
#endif
		*/
		glfwSwapBuffers();
		running = !glfwGetKey( GLFW_KEY_ESC ) && glfwGetWindowParam( GLFW_OPENED );
	}

	glfwTerminate();
	return EXIT_SUCCESS;
}
