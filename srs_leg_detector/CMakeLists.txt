cmake_minimum_required(VERSION 2.8.3)
project(srs_leg_detector)

SET(SOURCES src/laser_processor.cpp src/srs_leg_detector.cpp src/calc_leg_features.cpp)

find_package(catkin REQUIRED COMPONENTS
  roslib
  roscpp
  rospy
  geometry_msgs
  sensor_msgs
  std_srvs
  std_msgs
  tf
  pcl_ros
  pcl_conversions
  cv_bridge
  message_generation
)

find_package(PCL 1.7 REQUIRED)
find_package(Boost REQUIRED COMPONENTS thread)
find_package(OpenCV REQUIRED)

catkin_package(
  CATKIN_DEPENDS roscpp rospy sensor_msgs geometry_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

add_service_files(
   FILES
   DetectLeg.srv
)

generate_messages(
   DEPENDENCIES
   geometry_msgs
   std_msgs
   std_srvs
)

include_directories(include)
include_directories(src)
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS 
  DEPENDS Boost
  roscpp
  geometry_msgs
  sensor_msgs
  std_msgs
  people_msgs
  message_runtime
  tf
 )

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")


#add_executable(srs_leg_detector ${SOURCES})
#target_link_libraries(srs_leg_detector ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBS})
#add_dependencies(srs_leg_detector ${catkin_EXPORTED_TARGETS})

add_executable(train_leg_detector ${SOURCES})
target_link_libraries(train_leg_detector ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBS})
add_dependencies(train_leg_detector ${catkin_EXPORTED_TARGETS})


add_executable(legListener  src/legListener.cpp)


