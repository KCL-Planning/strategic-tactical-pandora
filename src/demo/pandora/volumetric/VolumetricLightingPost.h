#ifndef DEMO_VOLLUMETRIC_POST_PROCESS_RENDERER_H
#define DEMO_VOLLUMETRIC_POST_PROCESS_RENDERER_H

#include <memory>

#include <dpengine/renderer/PostProcessRenderer.h>
#include <dpengine/renderer/WindowEventListener.h>

namespace DreadedPE
{
	class Material;
	class SpotLight;
	class SceneLeafLight;
	class SceneLeafModel;
	class SceneManager;
	class SceneNode;
	class ShadowRenderer;
	class Texture;
};

class VolumetricLightingPost : public DreadedPE::PostProcessRenderer, public DreadedPE::WindowEventListener
{
public:
	
	VolumetricLightingPost(DreadedPE::SceneManager& scene_manager, DreadedPE::SceneNode& parent, std::shared_ptr<DreadedPE::Material> terrain_material);
	
	/**
		* @param color_texture The colour texture of the current frame buffer.
		* @param depht_ext The depth texture of the current depht_texture.
		* @param dt The time that has elapsed since the last render command.
		* @return The ID of the framebuffer that needs to be rendered. If NULL is returned then the original framebuffer is rendered.
		*/
	GLuint postProcess(DreadedPE::Texture& color_texture, DreadedPE::Texture& depth_texture, float dt);
	
	/**
	 * Callback function when the window gets resized.
	 * @param width The new width of the window.
	 * @param height The new height of the window.
	 */
	void windowResized(DreadedPE::Window& window, int width, int height);
	
private:
	DreadedPE::SceneManager* scene_manager_;

	DreadedPE::ShadowRenderer* shadow_renderer_;

	DreadedPE::SceneLeafModel* light_volume_leaf_;

	DreadedPE::SpotLight* volumetric_light_point_;
	DreadedPE::SceneLeafLight* volumetric_light_leaf_;

	GLuint fbo_id_;
	DreadedPE::Texture* texture_;
};

#endif
