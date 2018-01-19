cmake_minimum_required(VERSION 2.8.3)
project(navi_mdp)	

SET(SOURCES src/MDPmanager.cpp src/srBSpline.cpp)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  actionlib
  roscpp
  rospy
  geometry_msgs
  sensor_msgs
  trajectory_msgs
  control_msgs
  message_generation
  std_msgs
  tf
)

find_package(Eigen3 QUIET)
if(NOT EIGEN3_FOUND)
    find_package(PkgConfig REQUIRED)
    pkg_check_modules(EIGEN3 REQUIRED eigen3)
    set(EIGEN3_INCLUDE_DIR ${EIGEN3_INCLUDE_DIRS})
endif()

include_directories(include src)

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES classifier
#  CATKIN_DEPENDS roscpp geometry_msgs
  DEPENDS
  	roscpp
  	geometry_msgs
  	sensor_msgs
  	message_runtime
        eigen
    tf
 )

include_directories(
  ${catkin_INCLUDE_DIRS}
)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

add_executable(mdpplanner src/global_planner.cpp ${SOURCES})
target_link_libraries(mdpplanner ${catkin_LIBRARIES})

add_executable(dynamic_planner src/dynamic_planner.cpp ${SOURCES})
target_link_libraries(dynamic_planner ${catkin_LIBRARIES})

add_executable(base_actionlib_mk src/base_actionlib.cpp)
target_link_libraries(base_actionlib_mk ${catkin_LIBRARIES})

add_executable(conti_base_actionlib src/base_conti_actionlib.cpp)
target_link_libraries(conti_base_actionlib ${catkin_LIBRARIES})
