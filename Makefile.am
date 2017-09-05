## Process this file with automake to produce Makefile.in
##
## Copyright (C) 2002-2004 Carnegie Mellon University
## Written by H�kan L. S. Younes.
##
## Permission is hereby granted to distribute this software for
## non-commercial research purposes, provided that this copyright
## notice is included with any such distribution.

## THIS SOFTWARE IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
## EITHER EXPRESSED OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, THE
## IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
## PURPOSE.  THE ENTIRE RISK AS TO THE QUALITY AND PERFORMANCE OF THE
## SOFTWARE IS WITH YOU.  SHOULD THE PROGRAM PROVE DEFECTIVE, YOU
## ASSUME THE COST OF ALL NECESSARY SERVICING, REPAIR OR CORRECTION.
##
## $Id: Makefile.am,v 1.6 2003/03/01 18:56:34 lorens Rel lorens $
#src/core/entities/Bridge.cpp src/core/entities/Entity.cpp src/core/entities/Lever.cpp src/core/entities/Monster.cpp src/core/entities/Player.cpp src/core/entities/WavingWater.cpp 
#src/core/entities/behaviours/Behaviour.cpp src/core/entities/behaviours/MoveBehaviour.cpp src/core/entities/behaviours/RotateBehaviour.cpp

AUTOMAKE_OPTIONS = subdir-objects
#SUBDIRS = FF-X

bin_PROGRAMS = engine engined
engine_SOURCES = src/Main.cpp \
               src/core/ai/pathfinding/ConvexNavigationArea.cpp src/core/ai/pathfinding/NavMeshAStar.cpp src/core/ai/pathfinding/NavigationMesh.cpp \
               src/core/collision/BoxCollision.cpp src/core/collision/CollisionInfo.cpp \
               src/core/entities/Entity.cpp src/core/entities/Monster.cpp src/core/entities/Player.cpp src/core/entities/WavingWater.cpp src/core/entities/HeightMap.cpp \
               src/core/entities/behaviours/Behaviour.cpp src/core/entities/behaviours/MoveBehaviour.cpp src/core/entities/behaviours/RotateBehaviour.cpp \
               src/core/entities/behaviours/HoverBehaviour.cpp \
               src/core/entities/camera/Camera.cpp src/core/entities/camera/DynamicCamera.cpp \
               src/core/gui/Button.cpp src/core/gui/Container.cpp src/core/gui/Frame.cpp src/core/gui/FreeTypeFont.cpp src/core/gui/GUIManager.cpp src/core/gui/Label.cpp src/core/gui/TextBox.cpp src/core/gui/CheckBox.cpp \
               src/core/gui/themes/MyGUITheme.cpp \
               src/core/light/Light.cpp src/core/light/PointLight.cpp src/core/light/DirectedLight.cpp \
               src/core/loaders/AssimpLoader.cpp src/core/loaders/PortalLevelFormatLoader.cpp src/core/loaders/Texture.cpp src/core/loaders/WavefrontLoader.cpp src/core/loaders/targa.cpp \
               src/core/math/BoundedBox.cpp src/core/math/Math.cpp src/core/math/Plane.cpp \
               src/core/models/AnimatedModel.cpp src/core/models/Animation.cpp src/core/models/AnimationNode.cpp src/core/models/Bone.cpp src/core/models/BoneNode.cpp \
               src/core/renderer/Renderer.cpp src/core/renderer/ShadowRenderer.cpp src/core/renderer/SimpleRenderer.cpp src/core/renderer/GUIRenderer.cpp \
               src/core/scene/Material.cpp src/core/scene/SceneLeaf.cpp src/core/scene/SceneLeafLight.cpp src/core/scene/SceneLeafModel.cpp src/core/scene/SceneManager.cpp src/core/scene/SceneNode.cpp src/core/scene/SkyBoxLeaf.cpp \
               src/core/scene/frustum/Frustum.cpp src/core/scene/frustum/SphereCheck.cpp \
               src/core/scene/portal/Portal.cpp src/core/scene/portal/Region.cpp \
               src/core/shaders/AnimatedShadowShader.cpp src/core/shaders/BasicShadowShader.cpp src/core/shaders/CreateAnimatedShadowMapShader.cpp src/core/shaders/LightShader.cpp src/core/shaders/ShadowShader.cpp src/core/shaders/SkyBoxShader.cpp src/core/shaders/TerrainShader.cpp src/core/shaders/ParticleComputerShader.cpp src/core/shaders/ParticleShader.cpp src/core/shaders/UnderWaterShader.cpp src/core/shaders/LineShader.cpp src/core/shaders/ShaderParser.cpp src/core/shaders/WaterShader.cpp src/core/shaders/GUIShader.cpp src/core/shaders/ToonShader.cpp \
               src/demo/FlatDemo.cpp src/demo/FrustumCullingExample.cpp src/demo/LoaderExample.cpp src/demo/PLFDemo.cpp \
               src/demo/flat/Wall.cpp src/demo/Pandora.cpp src/demo/pandora/AUV.cpp src/demo/pandora/Propeller.cpp src/demo/pandora/RRT.cpp src/demo/pandora/PlanningGUI.cpp \
               src/demo/pandora/ontology/Ontology.cpp \
               src/demo/pandora/controllers/FollowWaypointController.cpp \
               src/demo/pandora/sensors/Sonar.cpp src/demo/pandora/sensors/Odometry.cpp \
               src/demo/pandora/structures/ValvePanel.cpp \
               src/demo/shooter/ArmedPlayer.cpp src/demo/shooter/Bullet.cpp \
               src/shapes/AnimatedShape.cpp src/shapes/Cube.cpp src/shapes/Piramid.cpp src/shapes/Shape.cpp src/shapes/SkyBox.cpp src/shapes/Tree.cpp src/shapes/sphere.cpp src/shapes/terrain.cpp src/shapes/Line.cpp src/shapes/Water.cpp \
               src/core/particles/ParticleEmitter.cpp
#               src/core/particles/ParticleSystem.cpp
               

#engine_LDADD = @LIBOBJS@
engine_LDADD = -lglfw -lGLEW -lGL -lGLU -lassimp -lfreetype -L/usr/lib64 -L/opt/ros/hydro/lib -lroscpp -lroscpp_serialization -lrostime -ltf -lboost_system
engine_CXXFLAGS = -O3 -DNDEBUG -I/usr/include/freetype2 -I../libs/glm/

engined_SOURCES = src/Main.cpp \
               src/core/ai/pathfinding/ConvexNavigationArea.cpp src/core/ai/pathfinding/NavMeshAStar.cpp src/core/ai/pathfinding/NavigationMesh.cpp \
               src/core/collision/BoxCollision.cpp src/core/collision/CollisionInfo.cpp \
               src/core/entities/Entity.cpp src/core/entities/Monster.cpp src/core/entities/Player.cpp src/core/entities/WavingWater.cpp src/core/entities/HeightMap.cpp \
               src/core/entities/behaviours/Behaviour.cpp src/core/entities/behaviours/MoveBehaviour.cpp src/core/entities/behaviours/RotateBehaviour.cpp \
               src/core/entities/behaviours/HoverBehaviour.cpp \
               src/core/entities/camera/Camera.cpp src/core/entities/camera/DynamicCamera.cpp \
               src/core/gui/Button.cpp src/core/gui/Container.cpp src/core/gui/Frame.cpp src/core/gui/FreeTypeFont.cpp src/core/gui/GUIManager.cpp src/core/gui/Label.cpp src/core/gui/TextBox.cpp src/core/gui/CheckBox.cpp \
               src/core/gui/themes/MyGUITheme.cpp \
               src/core/light/Light.cpp src/core/light/PointLight.cpp src/core/light/DirectedLight.cpp \
               src/core/loaders/AssimpLoader.cpp src/core/loaders/PortalLevelFormatLoader.cpp src/core/loaders/Texture.cpp src/core/loaders/WavefrontLoader.cpp src/core/loaders/targa.cpp \
               src/core/math/BoundedBox.cpp src/core/math/Math.cpp src/core/math/Plane.cpp \
               src/core/models/AnimatedModel.cpp src/core/models/Animation.cpp src/core/models/AnimationNode.cpp src/core/models/Bone.cpp src/core/models/BoneNode.cpp \
               src/core/renderer/Renderer.cpp src/core/renderer/ShadowRenderer.cpp src/core/renderer/SimpleRenderer.cpp src/core/renderer/GUIRenderer.cpp \
               src/core/scene/Material.cpp src/core/scene/SceneLeaf.cpp src/core/scene/SceneLeafLight.cpp src/core/scene/SceneLeafModel.cpp src/core/scene/SceneManager.cpp src/core/scene/SceneNode.cpp src/core/scene/SkyBoxLeaf.cpp \
               src/core/scene/frustum/Frustum.cpp src/core/scene/frustum/SphereCheck.cpp \
               src/core/scene/portal/Portal.cpp src/core/scene/portal/Region.cpp \
               src/core/shaders/AnimatedShadowShader.cpp src/core/shaders/BasicShadowShader.cpp src/core/shaders/CreateAnimatedShadowMapShader.cpp src/core/shaders/LightShader.cpp src/core/shaders/ShadowShader.cpp src/core/shaders/SkyBoxShader.cpp src/core/shaders/TerrainShader.cpp src/core/shaders/ParticleComputerShader.cpp src/core/shaders/ParticleShader.cpp src/core/shaders/UnderWaterShader.cpp src/core/shaders/LineShader.cpp src/core/shaders/ShaderParser.cpp src/core/shaders/WaterShader.cpp src/core/shaders/GUIShader.cpp src/core/shaders/ToonShader.cpp \
               src/demo/FlatDemo.cpp src/demo/FrustumCullingExample.cpp src/demo/LoaderExample.cpp src/demo/PLFDemo.cpp \
               src/demo/flat/Wall.cpp src/demo/Pandora.cpp src/demo/pandora/AUV.cpp src/demo/pandora/Propeller.cpp src/demo/pandora/RRT.cpp src/demo/pandora/PlanningGUI.cpp \
               src/demo/pandora/ontology/Ontology.cpp \
               src/demo/pandora/controllers/FollowWaypointController.cpp \
               src/demo/pandora/sensors/Sonar.cpp src/demo/pandora/sensors/Odometry.cpp \
               src/demo/pandora/structures/ValvePanel.cpp \
               src/demo/shooter/ArmedPlayer.cpp src/demo/shooter/Bullet.cpp \
               src/shapes/AnimatedShape.cpp src/shapes/Cube.cpp src/shapes/Piramid.cpp src/shapes/Shape.cpp src/shapes/SkyBox.cpp src/shapes/Tree.cpp src/shapes/sphere.cpp src/shapes/terrain.cpp src/shapes/Line.cpp src/shapes/Water.cpp \
               src/core/particles/ParticleEmitter.cpp
#               src/core/particles/ParticleSystem.cpp
               

#engine_LDADD = @LIBOBJS@
engined_LDADD = -lglfw -lGLEW -lGL -lGLU -lassimp -lfreetype -L/usr/lib64 -L/opt/ros/hydro/lib -lroscpp -lroscpp_serialization -lrostime -ltf -lboost_system
engined_CXXFLAGS = -g -DNDEBUG -I/usr/include/freetype2 -I../libs/glm/
