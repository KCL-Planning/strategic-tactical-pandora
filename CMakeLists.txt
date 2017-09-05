cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Load custom modules.
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake/Modules/")

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

# OCTOMAP
find_package(octomap REQUIRED)
include_directories(${OCTOMAP_INCLUDE_DIRS})
link_libraries(${OCTOMAP_LIBRARIES})

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DNDEBUG -O3 -Wno-unknown-pragmas")
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DNDEBUG -g -Wno-unknown-pragmas")

# visualisation
find_package(OpenGL REQUIRED)
find_package(GLUT REQUIRED)
#find_package(ASSIMP3 REQUIRED)
#find_package(ASSIMP REQUIRED)
find_package(octomap 1.6 REQUIRED)

#find_package(FreeImage REQUIRED)
#include_directories(${OPENGL_INCLUDE_DIRS} ${GLUT_INCLUDE_DIRS} ${ASSIMP3_INCLUDE_DIRS} /usr/include/freetype2)
include_directories(${OPENGL_INCLUDE_DIRS} ${GLUT_INCLUDE_DIRS} /usr/include/freetype2)

# executables
rosbuild_add_executable(visualiser src/Main.cpp 
               src/core/animation/BouncingBox.cpp src/core/animation/LinearAnimation.cpp
               src/core/ai/pathfinding/ConvexNavigationArea.cpp src/core/ai/pathfinding/NavMeshAStar.cpp src/core/ai/pathfinding/NavigationMesh.cpp 
               src/core/collision/BoxCollision.cpp src/core/collision/CollisionInfo.cpp 
               src/core/entities/Entity.cpp src/core/entities/Monster.cpp src/core/entities/Player.cpp src/core/entities/WavingWater.cpp src/core/entities/HeightMap.cpp src/core/entities/Bridge.cpp src/core/entities/Lever.cpp
               src/core/entities/behaviours/Behaviour.cpp src/core/entities/behaviours/MoveBehaviour.cpp src/core/entities/behaviours/RotateBehaviour.cpp 
               src/core/entities/behaviours/HoverBehaviour.cpp 
               src/core/entities/camera/Camera.cpp src/core/entities/camera/DynamicCamera.cpp src/core/entities/camera/FreeMovingCamera.cpp
               src/core/gui/Button.cpp src/core/gui/Container.cpp src/core/gui/Frame.cpp src/core/gui/GUIManager.cpp src/core/gui/Label.cpp src/core/gui/TextBox.cpp src/core/gui/CheckBox.cpp src/core/gui/Scrollbar.cpp src/core/gui/ScrollbarUnit.cpp src/core/gui/GUIElement.cpp src/core/gui/Canvas.cpp 
               src/core/gui/fonts/Font.cpp src/core/gui/fonts/TexturedFont.cpp
               src/core/gui/themes/MyGUITheme.cpp 
               src/core/light/Light.cpp src/core/light/PointLight.cpp src/core/light/DirectedLight.cpp 
               src/core/loaders/AssimpLoader.cpp src/core/loaders/PortalLevelFormatLoader.cpp src/core/loaders/WavefrontLoader.cpp src/core/loaders/targa.cpp src/core/loaders/PLFCube.cpp 
               src/core/texture/Texture.cpp src/core/texture/TargaTexture.cpp src/core/texture/RandomTexture.cpp src/core/texture/FreeImageLoader.cpp
               src/core/math/BoundedBox.cpp src/core/math/Math.cpp src/core/math/Plane.cpp 
               src/core/models/AnimatedModel.cpp src/core/models/Animation.cpp src/core/models/AnimationNode.cpp src/core/models/Bone.cpp src/core/models/BoneNode.cpp 
               src/core/renderer/Renderer.cpp src/core/renderer/ShadowRenderer.cpp src/core/renderer/SimpleRenderer.cpp src/core/renderer/GUIRenderer.cpp 
               src/core/scene/Material.cpp src/core/scene/SceneLeaf.cpp src/core/scene/SceneLeafLight.cpp src/core/scene/SceneLeafModel.cpp src/core/scene/SceneManager.cpp src/core/scene/SceneNode.cpp src/core/scene/SkyBoxLeaf.cpp 
               src/core/scene/frustum/Frustum.cpp src/core/scene/frustum/SphereCheck.cpp 
               src/core/scene/portal/Portal.cpp src/core/scene/portal/Region.cpp 
               src/core/shaders/glslshader.cpp src/core/shaders/AnimatedShadowShader.cpp src/core/shaders/BasicShadowShader.cpp src/core/shaders/CreateAnimatedShadowMapShader.cpp src/core/shaders/LightShader.cpp src/core/shaders/ShadowShader.cpp src/core/shaders/SkyBoxShader.cpp src/core/shaders/TerrainShader.cpp src/core/shaders/UnderWaterShader.cpp src/core/shaders/LineShader.cpp src/core/shaders/ShaderParser.cpp src/core/shaders/WaterShader.cpp src/core/shaders/GUIShader.cpp src/core/shaders/ToonShader.cpp  src/core/shaders/MergeFBOShader.cpp
               src/demo/FlatDemo.cpp src/demo/LoaderExample.cpp src/demo/PLFDemo.cpp src/demo/VolumetricLightDemo.cpp src/demo/ParticleTest.cpp src/demo/ZombieHorde.cpp src/demo/GUITester.cpp src/demo/AnimationTest.cpp
               src/demo/flat/Wall.cpp src/demo/Pandora.cpp src/demo/pandora/AUV.cpp src/demo/pandora/Propeller.cpp src/demo/pandora/RRT.cpp src/demo/pandora/Waypoint.cpp src/demo/pandora/structures/PipeNetwork.cpp 
               src/demo/pandora/ontology/OntologyInterface.cpp src/demo/pandora/ontology/Ontology.cpp src/demo/pandora/ontology/OctomapBuilder.cpp src/demo/pandora/ontology/Goal.cpp src/demo/pandora/ontology/InspectionGoal.cpp src/demo/pandora/ontology/ValveGoal.cpp src/demo/pandora/ontology/ChainGoal.cpp src/demo/pandora/ontology/Pose.cpp
               src/demo/pandora/ontology/filter/Filter.cpp src/demo/pandora/ontology/filter/ConnectionFilter.cpp src/demo/pandora/ontology/filter/PillarFilter.cpp src/demo/pandora/ontology/filter/ChainFilter.cpp
               src/demo/pandora/controllers/FollowWaypointController.cpp src/demo/pandora/controllers/ActionController.cpp src/demo/pandora/controllers/ObserveController.cpp src/demo/pandora/controllers/ExaminePanelController.cpp src/demo/pandora/controllers/ValveTurnController.cpp src/demo/pandora/controllers/IlluminateController.cpp src/demo/pandora/controllers/ChainFollowController.cpp src/demo/pandora/controllers/RechargeController.cpp
               src/demo/pandora/sensors/Sonar.cpp src/demo/pandora/sensors/SliceSonar.cpp src/demo/pandora/sensors/Odometry.cpp 
               src/demo/pandora/structures/Valve.cpp src/demo/pandora/structures/Pillar.cpp src/demo/pandora/structures/Chain.cpp src/demo/pandora/structures/ChainLink.cpp src/demo/pandora/structures/Structure.cpp
               src/demo/pandora/gui/PlanVisualiser.cpp src/demo/pandora/gui/StrategicPlanVisualiser.cpp src/demo/pandora/gui/WaypointLabeler.cpp src/demo/pandora/gui/PlanLine.cpp src/demo/pandora/gui/PlanGraph.cpp src/demo/pandora/gui/ActionLabel.cpp src/demo/pandora/gui/BillBoard.cpp src/demo/pandora/gui/PlanningGUI.cpp src/demo/pandora/gui/EditorTool.cpp src/demo/pandora/gui/StrategicPlanGUIElement.cpp src/demo/pandora/gui/AUVStatusIcon.cpp
               src/demo/pandora/level/MissionSite.cpp src/demo/pandora/level/Mission.cpp
               src/demo/pandora/models/Shark.cpp src/demo/pandora/models/Seal.cpp src/demo/pandora/models/UnderWaterVolcano.cpp src/demo/pandora/models/RobotHand.cpp
               src/demo/pandora/shaders/CausticShader.cpp src/demo/pandora/shaders/CausticTerrainShader.cpp src/demo/pandora/shaders/CausticTexture.cpp 
               src/demo/pandora/editor/XMLLevelLoader.cpp
               src/demo/pandora/ROSTimer.cpp
               src/demo/pandora/3rdparty/pugixml/pugixml.cpp
               src/demo/gui_demo/OmniReporter.cpp src/demo/gui_demo/FPSLabel.cpp
               src/demo/shooter/ArmedPlayer.cpp src/demo/shooter/Bullet.cpp 
               src/demo/volumetric/LightVolumeShape.cpp src/demo/volumetric/ShadowVolumeShader.cpp
               src/shapes/AnimatedShape.cpp src/shapes/Cube.cpp src/shapes/Piramid.cpp src/shapes/Shape.cpp src/shapes/SkyBox.cpp src/shapes/Tree.cpp src/shapes/sphere.cpp src/shapes/terrain.cpp src/shapes/Line.cpp src/shapes/Water.cpp src/shapes/FrustumShape.cpp src/shapes/TextBanner.cpp src/shapes/Cylinder.cpp
               src/core/particles/GPUParticleEmitter.cpp src/core/particles/GPUParticleComputerShader.cpp src/core/particles/GPUParticleDrawShader.cpp
               src/demo/ZombieHorde.cpp src/demo/instance_rendering/InstanceRenderedShape.cpp src/demo/instance_rendering/AnimatedInstanceShader.cpp src/demo/instance_rendering/InstanceShader.cpp src/demo/instance_rendering/SceneLeafInstanced.cpp src/demo/instance_rendering/InstanceRenderedSceneNode.cpp src/demo/physical_sim/PhysicsCube.cpp src/demo/FallingBlock.cpp
               src/demo/RegionTest/RegionWorld.cpp src/demo/RegionTest/FrustumEntity.cpp
)

target_link_libraries(visualiser ${GLUT_LIBRARIES} ${OPENGL_LIBRARIES} -lglfw -lGLEW -lGL -lGLU -lassimp  -lfreeimage -L/usr/lib64 -L/opt/ros/hydro/lib -lroscpp -lroscpp_serialization -lrostime -ltf -lboost_system)
#target_link_libraries(visualiser ${GLUT_LIBRARIES} ${OPENGL_LIBRARIES} -lfbxsdk -lglfw -lGLEW -lGL -lGLU -lassimp3  -lfreeimage -L/usr/lib64 -L/opt/ros/hydro/lib -L/usr/local/lib/gcc4/x86/release -lroscpp -lroscpp_serialization -lrostime -ltf -lboost_system)
#target_link_libraries(visualiser ${GLUT_LIBRARIES} ${OPENGL_LIBRARIES} -lfbxsdk -lglfw -lGLEW -lGL -lGLU -lassimp_latest  -lfreeimage -L/usr/lib64 -L/opt/ros/hydro/lib -L/usr/local/lib/gcc4/x86/release -lroscpp -lroscpp_serialization -lrostime -ltf -lboost_system)

# explicit boost link for picky systems (my netbook only so far)
rosbuild_add_boost_directories(../libs/glm /usr/include/freetype2)
rosbuild_link_boost(visualiser filesystem system)

#rosbuild_add_executable(test src/demo/pandora/test_scenarios/DoubleAUVPillarDetection.cpp)
#target_link_libraries(test ${GLUT_LIBRARIES} ${OPENGL_LIBRARIES} -lglfw -lGLEW -lGL -lGLU -lassimp3  -lfreeimage -L/usr/lib64 -L/opt/ros/hydro/lib -lroscpp -lroscpp_serialization -lrostime -ltf -lboost_system)
