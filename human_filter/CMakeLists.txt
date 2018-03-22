cmake_minimum_required(VERSION 2.8.3)
project(human_filter)

SET(LEGSOURCES src/laser_processor.cpp src/srs_leg_detector.cpp src/calc_leg_features.cpp)
SET(BACKUP_LEGSOURCES src/laser_processor.cpp src/backup_srs_leg_detector.cpp src/calc_leg_features.cpp)

# Look for bfl (Bayesian Filtering Library)
find_package(PkgConfig)
pkg_check_modules(BFL REQUIRED orocos-bfl)
pkg_check_modules(bullet REQUIRED bullet)

include_directories(${BFL_INCLUDE_DIRS})
link_directories(${BFL_LIBRARY_DIRS})

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
  std_msgs
  std_srvs
  geometry_msgs
  tf
  tf2
  message_filters
  people_msgs
  sensor_msgs
  nav_msgs
  actionlib_msgs
  keyboard
  actionlib
  message_generation
  cv_bridge
  pcl_ros
  pcl_conversions
  dynamic_reconfigure
)


## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS thread)
find_package(PCL 1.7 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(orocos_kdl REQUIRED)

generate_dynamic_reconfigure_options(
  cfg/LegDetection.cfg
)


add_service_files(
   FILES
   set_target_to_follow.srv
   DetectLegs.srv
)

add_action_files(
  DIRECTORY action
  FILES FollowMe.action
)

generate_messages(
   DEPENDENCIES
   geometry_msgs
   sensor_msgs
   std_msgs
   std_srvs
   actionlib_msgs
)


## Declare things to be passed to dependent projects
catkin_package(
INCLUDE_DIRS include
CATKIN_DEPENDS
    roscpp
    sensor_msgs
    std_msgs
    geometry_msgs
    people_msgs
    nav_msgs
    actionlib_msgs
    actionlib
    message_runtime
LIBRARIES human_filter
)

## Specify additional locations of header files
include_directories(
  include ${catkin_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    ${orocos_kdl_INCLUDE_DIRS}
    ${BFL_INCLUDE_DIRS}
    ${BFL_INCLUDE_DIRS}/bfl
)

link_directories(${BFL_LIBRARY_DIRS})

## Declare a cpp library
add_library(human_filter 
            src/uniform_vector.cpp 
            src/gaussian_vector.cpp 
            src/gaussian_pos_vel.cpp 
            src/mcpdf_pos_vel.cpp
            src/mcpdf_vector.cpp
            src/sysmodel_pos_vel.cpp
            src/sysmodel_vector.cpp
            src/measmodel_pos.cpp
            src/measmodel_vector.cpp
            src/tracker_particle.cpp 
            src/tracker_kalman.cpp 
            src/detector_particle.cpp           
)

## Declare a cpp executable
add_executable(kalman_tracker src/people_tracking_node.cpp)
add_executable(edge_midfilter src/edge_leg_filter_node.cpp)
add_executable(opleg_filter src/leg_op_filter_node.cpp)
add_executable(follow_service_test src/service_test.cpp)


add_executable(srs_leg_detector ${LEGSOURCES})
target_link_libraries(srs_leg_detector human_filter ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBS} ${BFL_LIBRARIES} ${Boost_LIBRARIES} ${orocos_kdl_LIBRARIES})
add_dependencies(srs_leg_detector ${catkin_EXPORTED_TARGETS})

add_executable(backup_srs_leg_detector ${BACKUP_LEGSOURCES})
target_link_libraries(backup_srs_leg_detector human_filter ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBS} ${BFL_LIBRARIES} ${Boost_LIBRARIES} ${orocos_kdl_LIBRARIES})
add_dependencies(backup_srs_leg_detector ${catkin_EXPORTED_TARGETS})



## Add cmake target dependencies of the executable/library
add_dependencies(kalman_tracker human_filter ${cakin_EXPORTED_TARGETS})
add_dependencies(edge_midfilter human_filter ${cakin_EXPORTED_TARGETS})
add_dependencies(follow_service_test human_filter ${cakin_EXPORTED_TARGETS})



## Specify libraries to link a library or executable target against
target_link_libraries(kalman_tracker human_filter ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${BFL_LIBRARIES}
)

target_link_libraries(edge_midfilter human_filter ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${BFL_LIBRARIES}
)

target_link_libraries(opleg_filter human_filter ${catkin_LIBRARIES}
    ${Boost_LIBRARIES} ${BFL_LIBRARIES})

target_link_libraries(follow_service_test human_filter ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${BFL_LIBRARIES})


install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS kalman_tracker
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS edge_midfilter
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS opleg_filter
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(TARGETS follow_service_test
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS srs_leg_detector
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

#install(TARGETS backup_srs_leg_detector
  #ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  #LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  #RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#)


install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(FILES config/trained_leg_detector.yaml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
)


install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
