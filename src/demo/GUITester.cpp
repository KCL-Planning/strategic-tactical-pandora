#include "GUITester.h"

#ifdef _WIN32
#include <Windows.h>
#endif

#include <iostream>
#include <cstdlib>
#include <time.h>
#include <fstream>

#include <glm/gtc/matrix_transform.hpp> 
#include <GL/glew.h>

#include "../core/texture/TargaTexture.h"
#include "../core/entities/camera/Camera.h"
#include "../core/entities/camera/DynamicCamera.h"
#include "../core/entities/camera/FreeMovingCamera.h"

#include "../core/scene/SceneManager.h"

#include "../core/gui/themes/MyGUITheme.h"
#include "../core/gui/Frame.h"
#include "../core/gui/GUIManager.h"
#include "../core/gui/Label.h"
#include "../core/gui/Button.h"
#include "../core/gui/Scrollbar.h"
#include "../core/gui/CheckBox.h"
#include "../core/gui/TextBox.h"
#include "../core/gui/Canvas.h"
#include "../core/gui/fonts/TexturedFont.h"

#include "gui_demo/FPSLabel.h"

#include "gui_demo/OmniReporter.h"

GUITester::GUITester(SceneManager& scene_manager)
	: scene_manager_(&scene_manager)
{
	srand (time(NULL));
}

bool GUITester::init(int argc, char** argv)
{
#ifdef _WIN32
	if (!GLEW_VERSION_4_3)
	{
		std::cerr << "Right OpenGL version is not suppored!" << std::endl;
		std::ofstream file("example_init_errors.txt");
		file << "VBOs are not supported by your graphics card." << std::endl;
		file.close();
        	std::cerr << "VBOs are not supported by your graphics card" << std::endl;
	        MessageBox(NULL, "VBOs are not supported by your graphics card!", "An error occurred", MB_ICONERROR | MB_OK);
        	return false;
	}
	std::cout << "Right version is supported! :D" << std::endl;
#endif
	glEnable(GL_DEPTH_TEST);
	glClearColor(0.0f, 1.0f, 0.0f, 1.0f);
	
	// Initialise the camera and make sure it is constantely been updated.
	camera_ = new FreeMovingCamera(*scene_manager_, &scene_manager_->getRoot(), glm::mat4(1.0f), 90.0f, 1024, 768, 0.1f, 300.0f);
	scene_manager_->addUpdateableEntity(*camera_);

	// Construct a demo GUI.
	MyGUITheme* theme = new MyGUITheme();
	Texture* font_texture = TargaTexture::loadTexture("data/textures/fonts/test_font.tga");
	
	Font* font = new TexturedFont(*font_texture);
	GUIManager& gui_manager = GUIManager::getInstance();
	
	Frame* frame = new Frame(*theme, *font, 100, 100, 500, 200);
	gui_manager.addFrame(*frame);
	
	Container* horizontal_scroll_container = new Container(*theme, font->clone(), 10, 10, 200, 80, false);
	frame->addElement(*horizontal_scroll_container, 35, -25);

	Container* container = new Container(*theme, font->clone(), 0, 0, 45000, 80, false);
	horizontal_scroll_container->addElement(*container, 0, 0);

	Scrollbar* scrollbar = new Scrollbar(*theme, font->clone(), 0, 0, 180, 10, *container, false);
	horizontal_scroll_container->addElement(*scrollbar, 10, -50);

	for (unsigned int i = 0; i < 100; ++i)
	{
		Label* label = new Label(*theme, 40, 20, "TEST", 12);
		container->addElement(*label, i * 80, -20);
	}

	Container* vertical_scroll_container = new Container(*theme, font->clone(), 250, 10, 80, 160, false);
	frame->addElement(*vertical_scroll_container, 250, -10);

	Container* container2 = new Container(*theme, font->clone(), 0, 0, 80, 4500, false);
	vertical_scroll_container->addElement(*container2, 0, 0);

	Scrollbar* vertical_scrollbar = new Scrollbar(*theme, font->clone(), 0, 0, 10, 150, *container2, true);
	vertical_scroll_container->addElement(*vertical_scrollbar, 70, -5);
	
	for (int i = 0; i < 100; ++i)
	{
		Label* label = new Label(*theme, 70, 20, "TEST2", 12);
		container2->addElement(*label, 0, i * -40);
	}
	
	// Add a button.
	Button* button = new Button(*theme, 100, 20, "Click", 12);
	frame->addElement(*button, 400, -50);
	
	Button* button2 = new Button(*theme, 100, 20, "123", 12);
	frame->addElement(*button2, 400, -100);

	Button* button3 = new Button(*theme, 100, 20, "456", 12);
	frame->addElement(*button3, 400, -150);

	// Create a second frame:
	Frame* frame2 = new Frame(*theme, *font, 400, 100, 500, 200);
	gui_manager.addFrame(*frame2);
	
	CheckBox* checkbox = new CheckBox(*theme, 20, 20);
	Label* checkbox_label1 = new Label(*theme, 200, 20, "Check box 1", 12);

	CheckBox* checkbox2 = new CheckBox(*theme, 20, 20);
	Label* checkbox_label2 = new Label(*theme, 200, 20, "Check box 2", 12);
	
	CheckBox* checkbox3 = new CheckBox(*theme, 20, 20);
	Label* checkbox_label3 = new Label(*theme, 200, 20, "Check box 3", 12);
	
	frame2->addElement(*checkbox, 20, -20);
	frame2->addElement(*checkbox_label1, 60, -20);

	frame2->addElement(*checkbox2, 20, -60);
	frame2->addElement(*checkbox_label2, 60, -60);

	frame2->addElement(*checkbox3, 20, -100);
	frame2->addElement(*checkbox_label3, 60, -100);

	Button* buttonf2 = new Button(*theme, 100, 20, "Click", 12);
	frame2->addElement(*buttonf2, 400, -50);
	
	Button* buttonf21 = new Button(*theme, 100, 20, "123", 12);
	frame2->addElement(*buttonf21, 400, -100);
	
	
	// Third frame.
	Frame* frame3 = new Frame(*theme, *font, 10, 600, 400, 100);
	gui_manager.addFrame(*frame3);
	
	TextBox* text_box = new TextBox(*theme, font->clone(), "Some text", 200, 30, 12);
	frame3->addElement(*text_box, 10, 0);

	// Forth frame.
	Frame* frame4 = new Frame(*theme, *font, 300, 600, 500, 60);
	gui_manager.addFrame(*frame4);

	Label* diagnostic_label = new Label(*theme, 500, 30, "No events as of yet", 12);
	frame4->addElement(*diagnostic_label, 10, -20);

	OmniReporter* omni_reporter = new OmniReporter(*diagnostic_label);
	button->addListener(*omni_reporter);
	button2->addListener(*omni_reporter);
	button3->addListener(*omni_reporter);
	buttonf2->addListener(*omni_reporter);
	buttonf21->addListener(*omni_reporter);
	checkbox->addStateChangeListener(*omni_reporter);
	checkbox2->addStateChangeListener(*omni_reporter);
	checkbox3->addStateChangeListener(*omni_reporter);
	
	Texture* icon_texture = TargaTexture::loadTexture("data/textures/icons.tga");
	
	Frame* frame5 = new Frame(*theme, *font, 0, 0, 400, 400);
	gui_manager.addFrame(*frame5);
	
	Canvas* canvas = new Canvas(*theme, font->clone(), 100, -400, 200, 200, *icon_texture);
	gui_manager.addFrame(*canvas);
	
	//frame5->addElement(*canvas, 0, 0);

	Container* test_container = new Container(*theme, font->clone(), 100, -400, 200, 200, false);
	//gui_manager.addFrame(*test_container);
	
	
	
	// FPS frame.
	Container* fps_container = new Container(*theme, font->clone(), 10, 10, 120, 20, false);
	Label* fps_label = new Label(*theme, 120, 20, "", 12);
	fps_container->addElement(*fps_label, 0, -20);
	fps_label_ = new FPSLabel(*fps_label);

	gui_manager.addFrame(*fps_container);

	return true;
}

bool GUITester::postInit()
{
	return true;
}

void GUITester::tick(float dt)
{
	//light_node_->setTransformation(glm::translate(light_node_->getLocalTransformation(), glm::vec3(dt, 0, 0)));
}

GLuint GUITester::postProcess(Texture& color_texture, Texture& depth_texture, float dt)
{
	fps_label_->frameRendered();
	return 0;
}

void GUITester::onResize(int width, int height)
{
	camera_->onResize(width, height);
}
