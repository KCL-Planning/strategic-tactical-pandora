#ifndef CORE_SCENE_SCENE_VISITOR_H
#define CORE_SCENE_SCENE_VISITOR_H

class SceneLeafLight;
class RenderableSceneLeaf;

/**
 * Interface implemented by any class that uses the visitor pattern to visit the scene
 * graph.
 */
class SceneVisitor
{
public:
	virtual void visit(const SceneLeafLight& light) = 0;
	virtual void visit(const RenderableSceneLeaf& model) = 0;
};

#endif
