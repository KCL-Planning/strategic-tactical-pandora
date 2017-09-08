cmake_minimum_required(VERSION 2.8)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Enable C++11
SET(CMAKE_CXX_FLAGS "-std=c++0x")

# Load custom modules.
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake/Modules/")

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)
set(DPE_INCLUDE_PATH /home/bram/projects/horror-flash-thing/trunk/include)
set(DPE_LIBRARY_PATH /home/bram/projects/horror-flash-thing/trunk/lib)

# OCTOMAP
find_package(octomap REQUIRED)
include_directories(${OCTOMAP_INCLUDE_DIRS} include)
link_libraries(${OCTOMAP_LIBRARIES})

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DNDEBUG -O3 -Wno-unknown-pragmas -Wno-unused-variable")
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DNDEBUG -g -Wno-unknown-pragmas -Wno-unused-variable")

# visualisation
find_package(OpenGL REQUIRED)
find_package(GLUT REQUIRED)
find_package(ASSIMP3 REQUIRED)
find_package(glfw3 3.2 REQUIRED)
find_package(octomap 1.6 REQUIRED)

#find_package(FreeImage REQUIRED)
#include_directories(${OPENGL_INCLUDE_DIRS} ${GLUT_INCLUDE_DIRS} ${ASSIMP3_INCLUDE_DIRS} /usr/include/freetype2)
include_directories(lib/glm-0.9.8.4 ${OPENGL_INCLUDE_DIRS} ${GLUT_INCLUDE_DIRS} ${DPE_INCLUDE_PATH} /usr/include/freetype2)

# executables
rosbuild_add_executable(visualiser src/Main.cpp 
               src/demo/Pandora.cpp src/demo/pandora/AUV.cpp src/demo/pandora/Propeller.cpp src/demo/pandora/RRT.cpp src/demo/pandora/Waypoint.cpp src/demo/pandora/structures/PipeNetwork.cpp 
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
               src/demo/gui_demo/FPSLabel.cpp
               src/demo/pandora/volumetric/LightVolumeShape.cpp src/demo/pandora/volumetric/ShadowVolumeShader.cpp src/demo/pandora/volumetric/VolumetricLightingPost.cpp
)

target_link_libraries(visualiser ${GLUT_LIBRARIES} ${OPENGL_LIBRARIES} -L${DPE_LIBRARY_PATH} -ldreaded_spectre_engine -lglfw3 -lGLEW -lGL -lGLU -lassimp  -lfreeimage -L/usr/lib64 -L/opt/ros/hydro/lib -lroscpp -lroscpp_serialization -lrostime -ltf -lboost_system)
#target_link_libraries(visualiser ${GLUT_LIBRARIES} ${OPENGL_LIBRARIES} -lfbxsdk -lglfw -lGLEW -lGL -lGLU -lassimp3  -lfreeimage -L/usr/lib64 -L/opt/ros/hydro/lib -L/usr/local/lib/gcc4/x86/release -lroscpp -lroscpp_serialization -lrostime -ltf -lboost_system)
#target_link_libraries(visualiser ${GLUT_LIBRARIES} ${OPENGL_LIBRARIES} -lfbxsdk -lglfw -lGLEW -lGL -lGLU -lassimp_latest  -lfreeimage -L/usr/lib64 -L/opt/ros/hydro/lib -L/usr/local/lib/gcc4/x86/release -lroscpp -lroscpp_serialization -lrostime -ltf -lboost_system)

# explicit boost link for picky systems (my netbook only so far)
rosbuild_add_boost_directories(../libs/glm /usr/include/freetype2)
rosbuild_link_boost(visualiser filesystem system)

#rosbuild_add_executable(test src/demo/pandora/test_scenarios/DoubleAUVPillarDetection.cpp)
#target_link_libraries(test ${GLUT_LIBRARIES} ${OPENGL_LIBRARIES} -lglfw -lGLEW -lGL -lGLU -lassimp3  -lfreeimage -L/usr/lib64 -L/opt/ros/hydro/lib -lroscpp -lroscpp_serialization -lrostime -ltf -lboost_system)
