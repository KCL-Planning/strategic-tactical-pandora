cmake_minimum_required(VERSION 2.8.3)
project(planning_system)

find_package(catkin REQUIRED COMPONENTS rospy roscpp std_msgs diagnostic_msgs planning_msgs knowledge_msgs geometry_msgs auv_msgs nav_msgs actionlib)
#find_package(Eigen3 REQUIRED)
find_package(PkgConfig)
pkg_search_module(Eigen3 REQUIRED eigen3)

###################################
## catkin specific configuration ##
###################################

catkin_package(
  DEPENDS system_lib std_msgs diagnostic_msgs rospy roscpp planning_msgs knowledge_msgs geometry_msgs auv_msgs nav_msgs actionlib
)
###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  ${catkin_INCLUDE_DIRS}
  #${Boost_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIRS}
  /usr/include/eigen3
)

# executables
add_executable(planner src/PlanningLoop.cpp src/VisualiserTimer.cpp)
add_executable(chainSearcher src/ChainSearcher.cpp)
#add_executable(valveTester src/util/ValveTester.cpp)

target_link_libraries(planner ${catkin_LIBRARIES})
target_link_libraries(chainSearcher ${catkin_LIBRARIES})
#target_link_libraries(valveTester ${catkin_LIBRARIES})

# explicit boost link for picky systems (my netbook only so far)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(planner filesystem system)
