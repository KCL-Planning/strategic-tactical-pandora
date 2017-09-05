#include "Reflection.h"

#include <vector>
#include <sstream>
#include "../shapes/Sphere.h"
#include "../core/Camera.h"
#include "../example.h"
#include "GL/glfw.h"
#include "../core/shaders/glslshader.h"
//#include "../core/freetypefont.h"
#include "../core/light/Light.h"

Reflection::Reflection(Example& example, Sphere& entity, float width, float height)
	: example(&example), entity(&entity), width(width), height(height)
{
	cubeMapInitialised = false;
	shader_ = new GLSLProgram("shaders/reflection.vert", "shaders/reflection.frag");
}

void Reflection::initialise()
{
	// Create a single texture which contains all 6 faces.
	glActiveTexture(GL_TEXTURE4);
	glGenTextures(1, &facesTexturesId);
	glBindTexture(GL_TEXTURE_CUBE_MAP, facesTexturesId);
	glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_MAG_FILTER,GL_LINEAR);
    glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_R, GL_CLAMP_TO_EDGE);
	
	// Render the scene from 6 different perspectives.
	Camera* camX = new Camera(90, 0, 180, entity->getPos().x, entity->getPos().y, entity->getPos().z);
	cameras[0] = camX; // Pos X.
	Camera* camY = new Camera(0, 90, entity->getPos().x, entity->getPos().y, entity->getPos().z);
	cameras[2] = camY; // Pos Y.
	Camera* camZ = new Camera(180, 0, 180, entity->getPos().x, entity->getPos().y, entity->getPos().z);
	cameras[4] = camZ; // Pos Z.

	Camera* camMinX = new Camera(-90, 0, 180, entity->getPos().x, entity->getPos().y, entity->getPos().z);
	cameras[1] = camMinX; // Min X.
	Camera* camMinY = new Camera(0, -90, entity->getPos().x, entity->getPos().y, entity->getPos().z);
	cameras[3] = camMinY; // Min Y.
	Camera* camMinZ = new Camera(0, 0, 180, entity->getPos().x, entity->getPos().y, entity->getPos().z);
	cameras[5] = camMinZ; // Min Z.

	// Prepare the shader.
	if (!shader_->initialize())
	{
		exit(1);
	}

	//Bind the attribute locations
	shader_->bindAttrib(0, "a_Vertex");
	shader_->bindAttrib(1, "a_Normal");

	//Re link the program
	shader_->linkProgram();

    //Viewport[2] stores the width of the viewport, vieport[3] stores the height
    //We pass these into our font so the ortho mode can set the resolution for the window
    //font = new FreeTypeFont("data/LiberationSans-Regular.ttf", viewport[2], viewport[3], 36);
//	font = new FreeTypeFont("data/fonts/LiberationSans-Regular.ttf", 1024, 768, 16);
//	if (!font->initialize())
//	{
//#ifdef _WIN32
//        MessageBox(NULL, "Failed to load font.", "An error occurred", MB_ICONERROR | MB_OK);
//#endif
//	}
}

void Reflection::prepare(float dt)
{
	glViewport(0, 0, width, height);
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	gluPerspective(90.0f, float(width) / float(height), 0.1f, 100.0f);
	glMatrixMode(GL_MODELVIEW);
	for (unsigned int i = 0; i < 6; ++i)
	{
		Camera* cam = cameras[i];

		// Render the scene with this camera.
		example->render(*cam, false, true, true, true, false);

		// Map the rendered scene onto the texture.
		glBindTexture(GL_TEXTURE_CUBE_MAP, facesTexturesId);
		if (!cubeMapInitialised)
			glCopyTexImage2D(GL_TEXTURE_CUBE_MAP_POSITIVE_X + i, 0, GL_RGB8, 0, 0, width, height, 0);
		else
			glCopyTexSubImage2D(GL_TEXTURE_CUBE_MAP_POSITIVE_X + i, 0, 0, 0, 0, 0, width, height);
	}

	// Reset the framebuffer and viewport so things are back to normal.
	int orgWidth, orgHeight;
	glfwGetWindowSize(&orgWidth, &orgHeight);
	example->onResize(orgWidth, orgHeight);
	cubeMapInitialised = true;
}

void Reflection::render(const Camera& cam, bool render_shadow, const LightManager& light_manager, GLSLProgram* shader)
{
	if (render_shadow)
	{
		shader_->bindShader();
		float modelMatrix[16];
		float modelviewMatrix[16];
		float projectionMatrix[16];

		glLoadIdentity();
		glTranslatef(entity->getPos().x, entity->getPos().y, entity->getPos().z);
		glGetFloatv(GL_MODELVIEW_MATRIX, modelMatrix);

		glLoadIdentity();
		cam.updateCamera();
		glTranslatef(entity->getPos().x, entity->getPos().y, entity->getPos().z);

		// Send info to the shaders.
		glGetFloatv(GL_MODELVIEW_MATRIX, modelviewMatrix);
		glGetFloatv(GL_PROJECTION_MATRIX, projectionMatrix);
		glBindTexture(GL_TEXTURE_CUBE_MAP, facesTexturesId);
	
		//Send the modelview and projection matrices to the shaders
		shader_->sendUniform4x4("modelview_matrix", modelviewMatrix);
		shader_->sendUniform4x4("projection_matrix", projectionMatrix);
		shader_->sendUniform4x4("model_matrix", modelMatrix);
		Vertex camLoc = cam.getPosition();
		shader_->sendUniform("camPosition", camLoc.x, camLoc.y, camLoc.z);
		shader_->sendUniform("texture0", 4);
		
		//Bind the vertex array and set the vertex pointer to point at it
		glBindBuffer(GL_ARRAY_BUFFER, entity->getVertexBufferId());
		glVertexAttribPointer((GLint)0, 3, GL_FLOAT, GL_FALSE, 0, 0);

		glBindBuffer(GL_ARRAY_BUFFER, entity->getNormalBufferId());
		glVertexAttribPointer((GLint)1, 3, GL_FLOAT, GL_FALSE, 0, 0);

		entity->plainRender();

		glLoadIdentity();
//		std::stringstream ss;
//		ss << "CAM: (" << camLoc.x << " " << camLoc.y << " " << camLoc.z <<") - Yaw=" << cam.getYaw() << "; Pitch=" << cam.getPitch() << "; Roll=" << cam.getRoll();
//		std::string camStr(ss.str());
//		font->printString(camStr, 10, 10);
	}
	else
	{
		entity->render(cam, render_shadow, light_manager, NULL);
	}
}
