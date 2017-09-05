#include "example.h"

#ifdef _WIN32
#include <windows.h>
#endif
#include <iostream>
#include <cstdlib>
#include <time.h>
#include <fstream>
#include "core/loaders/targa.h"
#include "core/Camera.h"
#include "shapes/terrain.h"
#include "shapes/Water.h"
#include "shapes/Tree.h"
#include "shapes/sphere.h"
#include "shapes/Piramid.h"
#include "effects/Reflection.h"
#include "core/light/PointLight.h"
#include "core/light/Light.h"
#include "effects/GodRays.h"
#include "core/freetypefont.h"

Example::Example()
{
    m_rotationAngle = 0.0f;
	m_lightPosZ = 0.0f;
	position = -2.0f;
	moveForward = true;
	moveLightForward = true;
	srand (time(NULL));
	light_manager_ = new LightManager();
}

bool Example::init()
{
	if (!GL_VERSION_3_0)
	{
		std::ofstream file("example_init_errors.txt");
		file << "VBOs are not supported by your graphics card." << std::endl;
		file.close();
        std::cerr << "VBOs are not supported by your graphics card" << std::endl;
        return false;
	}

	// Initialise the default shader.
	m_GLSLProgram = new LightShader("shaders/basic-shadow.vert", "shaders/basic-shadow.frag");

	// Create shaders.
	if (!m_GLSLProgram->initialize())
	{
		std::ofstream file("example_init_errors.txt");
		file << "Failed to initialise the shaders." << std::endl;
		file.close();
		return false;
	}

	checkError("Shader constructor.");

	//Bind the attribute locations.
	m_GLSLProgram->bindAttrib(0, "a_Vertex");
	checkError("Shader loaded attribute 0.");
    m_GLSLProgram->bindAttrib(1, "a_TexCoord");
	checkError("Shader loaded attribute 2.");
	m_GLSLProgram->bindAttrib(2, "a_Normal");
	checkError("Shader loaded attribute 3.");

	//Re link the program.
	m_GLSLProgram->linkProgram();
	m_GLSLProgram->resolveUniformNames();
	checkError("Shader linked.");

	m_GLSLProgram->bindShader(); //Enable our shader.

	Terrain* terrain = new Terrain();
	if (!terrain->loadHeightmap("data/heightmaps/heightmap.raw", 65))
	{
#ifdef _WIN32
		MessageBox(NULL, "Could not load the terrain heightmap", "Error", MB_OK);
#endif
		return false;
	}
	entities_.push_back(terrain);

	Piramid* light_cone_1 = new Piramid(*m_GLSLProgram, 7.0f, 8.0f, -2.0f);
	light_cone_1->initialise();
	light_entities_.push_back(light_cone_1);

	Piramid* light_cone_2 = new Piramid(*m_GLSLProgram, 7.0f, 8.0f, -2.0f);
	light_cone_2->initialise();
	light_entities_.push_back(light_cone_2);

	PointLight* light = new PointLight(*this, Vertex(-4.5f, 10.0f, 2.0f), Vertex(1.0f, -1.0, 0.0f), 20.0f, *light_cone_1);
	PointLight* light2 = new PointLight(*this, Vertex(-4.5f, 10.0f, 2.0f), Vertex(1.0f, -1.0, 0.0f), 20.0f, *light_cone_2);
	light_manager_->addLight(*light);
	light_manager_->addLight(*light2);

	god_rays_ = new GodRays(*this, *light, 1024, 768);

	Sphere* sphere_ = new Sphere(*m_GLSLProgram, 2.0f, 12.0f, 1.0f);
	if (!sphere_->initialize(50, 50, 1.0f))
	{
#ifdef _WIN32
		MessageBox(NULL, "Could not initialise the sphere", "Error", MB_OK);
#endif
		return false;
	}
	//entities_.push_back(sphere_);

	reflection = new Reflection(*this, *sphere_, 256, 256);
	reflection->initialise();

	Cube* cube = new Cube(*m_GLSLProgram, 1.0f, 1.0f, 8.0f, -5.0f);
	entities_.push_back(cube);

	Cube* cube2 = new Cube(*m_GLSLProgram, 1.0f, 1.0f, 8.0f, 2.0f);
	entities_.push_back(cube2);

	//shadowMapCube = new Cube(*m_GLSLProgram, .1f, 0.0f, 2.0f, 0.0f);

	Tree* trees = new Tree(*terrain, *m_GLSLProgram);
	trees->initalise();
	for (unsigned int i = 0; i < 20; )
	{
		float treePosX = (rand() / (float)RAND_MAX) * 64;
		float treePosZ = (rand() / (float)RAND_MAX) * 64;
		if (terrain->getHeight(treePosX, treePosZ) > 4.0f)
		{
			++i;
			trees->addTree(treePosX, treePosZ);
		}
	}
	entities_.push_back(trees);

	Water* water = new Water(65, 65, 4.0f);
	if (!water->initWater())
	{
#ifdef _WIN32
		MessageBox(NULL, "Could not initialise the water", "Error", MB_OK);
#endif
		return false;
	}
	transparent_entities_.push_back(water);

	glEnable(GL_DEPTH_TEST);
	glClearColor(.0f, .0f, 1.0f, 1.0f);

	// Load the textures.
	glEnable(GL_TEXTURE_2D);
	glEnable(GL_TEXTURE_CUBE_MAP);
	// Load some textures.
	// * Water texture.
	// * Grass texture.
	glGenTextures(5, textures);

	// Water texture.
	glActiveTexture(GL_TEXTURE0);
	glBindTexture(GL_TEXTURE_2D, textures[0]);

	TargaImage targa_loader;
	targa_loader.load("data/textures/water.tga");
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB8, targa_loader.getWidth(), targa_loader.getHeight(), 0, GL_RGB, GL_UNSIGNED_BYTE, targa_loader.getImageData());
	
	// Grass texture.
	glActiveTexture(GL_TEXTURE1);
	glBindTexture(GL_TEXTURE_2D, textures[1]);

	TargaImage targa_loader2;
	targa_loader2.load("data/textures/grass.tga");
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB8, targa_loader2.getWidth(), targa_loader2.getHeight(), 0, GL_RGB, GL_UNSIGNED_BYTE, targa_loader2.getImageData());

	// Height texture.
	glActiveTexture(GL_TEXTURE2);
	glBindTexture(GL_TEXTURE_1D, textures[2]);

	TargaImage targa_loader3;
	targa_loader3.load("data/textures/height.tga");
	glTexParameteri(GL_TEXTURE_1D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	glTexImage1D(GL_TEXTURE_1D, 0, GL_RGB8, targa_loader3.getWidth(), 0, GL_RGB, GL_UNSIGNED_BYTE, targa_loader3.getImageData());
	
	// Tree texture.
	glActiveTexture(GL_TEXTURE3);
	glBindTexture(GL_TEXTURE_2D, textures[3]);
	
	TargaImage targa_loader4;
	targa_loader4.load("data/textures/beech.tga");
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA8, targa_loader4.getWidth(), targa_loader4.getHeight(), 0, GL_RGBA, GL_UNSIGNED_BYTE, targa_loader4.getImageData());
	
	// Light texture.
	glActiveTexture(GL_TEXTURE28);
	glBindTexture(GL_TEXTURE_2D, textures[4]);
	
	TargaImage targa_loader5;
	if (targa_loader5.load("data/textures/light_point.tga"))
	{
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
		glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA8, targa_loader5.getWidth(), targa_loader5.getHeight(), 0, GL_RGBA, GL_UNSIGNED_BYTE, targa_loader5.getImageData());
	}
	else
	{
		MessageBox(NULL, "Could not initialise the point light!", "Error", MB_OK);
	}
	glActiveTexture(GL_TEXTURE10);
	checkError("Loaded textures.");

    m_rotationAngle = 0.0f;

    glEnable(GL_LINE_SMOOTH);
    glEnable(GL_POINT_SMOOTH);
	glEnable(GL_POLYGON_SMOOTH);

	glEnableVertexAttribArray(0); // Vertex.
	glEnableVertexAttribArray(1); // Colour.
	glEnableVertexAttribArray(2); // Texture.
	glEnableVertexAttribArray(3); // Normals.

	// Lights.
	light_manager_->initialise();
	checkError("Enabled vertexes.");

	// Post effects.
	god_rays_->initialise();

	// Setup the off screen framebuffer.
	glGenTextures(1, &texture_id_);
	glBindTexture(GL_TEXTURE_2D, texture_id_);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, 1024, 768, 0, GL_RGB, GL_FLOAT, NULL);

	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
	
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);

	glGenTextures(1, &depth_id_);
	glBindTexture(GL_TEXTURE_2D, depth_id_);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_DEPTH_COMPONENT32, 1024, 768, 0, GL_DEPTH_COMPONENT, GL_FLOAT, NULL);

	glGenFramebuffers(1, &fbo_id_);
	glBindFramebuffer(GL_FRAMEBUFFER, fbo_id_);

	// Attach the rgb texture to it.
	glFramebufferTexture(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, texture_id_, 0);
	glFramebufferTexture(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, depth_id_, 0);

	glActiveTexture(GL_TEXTURE14);
	glBindTexture(GL_TEXTURE_2D, texture_id_);
	glActiveTexture(GL_TEXTURE30);

	glBindFramebuffer(GL_FRAMEBUFFER, 0);
/*
	font = new FreeTypeFont("data/fonts/LiberationSans-Regular.ttf", 1024, 768, 16);
	if (!font->initialize())
	{
#ifdef _WIN32
        MessageBox(NULL, "Failed to load font.", "An error occurred", MB_ICONERROR | MB_OK);
#endif
	}
*/
    return true;
}

void Example::prepare(float dt, const Camera& cam)
{
	for (std::vector<Entity*>::const_iterator ci = entities_.begin(); ci != entities_.end(); ++ci)
	{
		(*ci)->prepare(dt);
	}
	
	for (std::vector<Entity*>::const_iterator ci = transparent_entities_.begin(); ci != transparent_entities_.end(); ++ci)
	{
		(*ci)->prepare(dt);
	}
	light_manager_->prepareLights(dt);
	reflection->prepare(dt);
	god_rays_->prepare(cam);
}

void Example::render(const Camera& cam, bool renderReflection, bool renderShadow, bool drawOpaqueSurfaces, bool drawTransparentSurfaces, bool drawLightEntities, bool drawToDefaultFrameBuffer, GLSLProgram* shader)
{
	glEnable(GL_CULL_FACE);
	if (drawToDefaultFrameBuffer)
	{
		glBindFramebuffer(GL_FRAMEBUFFER, fbo_id_);
	}
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	if (drawOpaqueSurfaces)
	{
		for (std::vector<Entity*>::const_iterator ci = entities_.begin(); ci != entities_.end(); ++ci)
		{
			(*ci)->render(cam, renderShadow, *light_manager_, shader);
		}
	}
	else
	{
		GLSLProgram* point_light_shader = PointLight::getShader();
		for (std::vector<Entity*>::const_iterator ci = entities_.begin(); ci != entities_.end(); ++ci)
		{
			(*ci)->render(cam, renderShadow, *light_manager_, point_light_shader);
		}
	}

	// We always use the standard shader for the lightning.
	if (drawLightEntities)
	{
		for (std::vector<Entity*>::const_iterator ci = light_entities_.begin(); ci != light_entities_.end(); ++ci)
		{
			(*ci)->render(cam, renderShadow, *light_manager_, NULL);
		}
	}

	if (drawTransparentSurfaces)
	{
		for (std::vector<Entity*>::const_iterator ci = transparent_entities_.begin(); ci != transparent_entities_.end(); ++ci)
		{
			(*ci)->render(cam, renderShadow, *light_manager_, shader);
		}
	}

	if (renderReflection)
	{
		reflection->render(cam, renderShadow, *light_manager_, shader);
	}
	if (drawToDefaultFrameBuffer)
	{
		std::stringstream ss;
		//ss << "CAM: (" << cam.getPosition().x << " " << cam.getPosition().y << " " << cam.getPosition().z <<") - Yaw=" << cam.getYaw() << "; Pitch=" << cam.getPitch() << "; Roll=" << cam.getRoll();
		ss << "Light position: (" << god_rays_->getLightLocation().x << ", " << god_rays_->getLightLocation().y << ", " << god_rays_->getLightLocation().z << ", " << god_rays_->getLightLocation().w;
		std::string camStr(ss.str());
		font->printString(camStr, 10, 10);
		glBindFramebuffer(GL_FRAMEBUFFER, 0);
	}
}

void Example::postProcess(const Camera& cam)
{
	god_rays_->postProcess();
	
	//glBindFramebuffer(GL_READ_FRAMEBUFFER, fbo_id_);
	glBindFramebuffer(GL_READ_FRAMEBUFFER, god_rays_->getFrameBufferId());
	glBindFramebuffer(GL_DRAW_FRAMEBUFFER, 0);

	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glBlitFramebuffer(0, 0, 1024, 768, 0, 0, 1024, 768, GL_COLOR_BUFFER_BIT, GL_NEAREST);
	glBindFramebuffer(GL_FRAMEBUFFER, 0);
	
	onResize(1024, 768);
}

bool Example::checkError(const std::string& description) const
{
	
	GLenum error = glGetError();
	if (error != GL_NO_ERROR)
	{
		std::ofstream file("errors2.txt");
		file << description << std::endl;
		file << "Error:" << glGetError() << std::endl;
		file.close();
#ifdef _WIN32
        MessageBox(NULL, description.c_str(), "An error occurred", MB_ICONERROR | MB_OK);
#endif
		exit(1);
		return false;
	}
	
	return true;
}

void Example::shutdown()
{
/*
	GLenum error = glGetError();
	if (error != GL_NO_ERROR)
	{
		std::ofstream file("errors.txt");
		file << "Error:" << glGetError() << std::endl;
		file.close();
	}
	glDeleteBuffers(1, &buffer_index);
*/
}

void Example::onResize(int width, int height)
{
    glViewport(0, 0, width, height);

    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(45.0f, float(width) / float(height), 0.1f, 100.0f);

    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
}

std::vector<std::string> Example::getSupportedExtensions() const
{
	PFNGLGETSTRINGIPROC glGetStringi = NULL;
	glGetStringi = (PFNGLGETSTRINGIPROC)wglGetProcAddress("glGetStringi");

	std::ofstream file("extensions.txt");

	std::vector<std::string> extensions;
	if (glGetStringi == NULL)
	{
		file << "No extensions are supported! :(((" << std::endl;
		file.close();
		return extensions;
	}

	GLint numExtensions = 0;
	glGetIntegerv(GL_NUM_EXTENSIONS, &numExtensions);
	for (int i = 0; i < numExtensions; ++i)
	{
		std::string extension = (const char*)glGetStringi(GL_EXTENSIONS, i);
		extensions.push_back(extension);
		file << extension.c_str() << std::endl;
	}
	file.close();
	return extensions;
}
