#ifndef CORE_RENDERER_SHADOW_RENDERER_H
#define CORE_RENDERER_SHADOW_RENDERER_H

#include <sstream>
#include <vector>

#include <GL/glew.h>

#include "Renderer.h"

/**
 * In order to render a scene, we use the visitor pattern and deal with geometry and lights 
 * in different ways.
 */
class Entity;
class Light;
class PointLight;
class LightManager;
class SceneLeafLight;
class SceneLeafModel;
class SceneManager;
class SceneNode;
class Frustum;
class FrustumCaster;
class Texture;

class ShadowRenderer : public Renderer
{
public:
	/**
	 * Create a shadow shader for the given @ref{light}.
	 * @param scene_manager The scene manager.
	 * @param depth_texture_size The size of the depth texture. The texture is square.
	 * @param cull_mode The culling mode used while rendering, can be GL_BACK, GL_FRONT, GL_BOTH, GL_NONE.
	 * @param texture_compare_mode The compare method used for creating the depth map.
	 * @param texture_compare_function The compare function used.
	 */
	ShadowRenderer(SceneManager& scene_manager, unsigned int depth_texture_size, GLenum cull_mode = GL_FRONT, GLint texture_compare_mode = GL_COMPARE_REF_TO_TEXTURE, GLint texture_compare_function = GL_LEQUAL);

	~ShadowRenderer();

	void render(const FrustumCaster& cam);

	GLuint getFramebufferId() const { return fbo_id_; }
	//GLuint getTextureId() const { return texture_id_; }
	Texture& getTexture() const { return *depth_texture_; }

	void visit(const SceneLeafLight& light);
	void visit(const RenderableSceneLeaf& model);
	const std::stringstream& getDebugString() const { return ss_; }
	
	/**
	 * Set the way the shadow renderer culls faces, the options are GL_FRONT, GL_BACK, and GL_FRONT_AND_BACK.
	 */
	void setCullMode(GLenum cull_mode) { cull_mode_ = cull_mode; }

	enum SHADOW_TYPE { NO_SHADOW, STATIC_SHADOW, ANIMATED_SHADOW };

	void onResize(int width, int height);
private:

	void initialise();

	std::vector<const RenderableSceneLeaf*> active_entities_;
	std::vector<const SceneLeafLight*> active_lights_;

	SceneManager* scene_manager_;
	unsigned int depth_texture_size_;

	// Off screen buffer.
	GLuint fbo_id_;
	//GLuint texture_id_;//, depth_id_;
	
	Texture* depth_texture_;

	Frustum* frustum_;
	//LightManager* light_manager_;
	
	GLenum cull_mode_; // The method that is used to cull (default is back).
	GLint texture_compare_mode_;
	GLint texture_compare_function_;

	// Debug.
	std::stringstream ss_;
};

#endif