cmake_minimum_required(VERSION 2.8.3)
project(yumi_manager)

## Compile as C++11, supported in ROS Kinetic and newer
 add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  perception_manager
  rospy
  std_msgs
  cv_bridge
  yumi_actions
  actionlib
  yumi_eneroth_bridge
  message_generation
  trajectory_msgs
  moveit_msgs
  tf
  ros_cpp_utils
)

find_package(OpenCV 3 REQUIRED)

## Generate messages in the 'msg' folder
add_message_files(
   FILES
   SceneObjects.msg

 )


## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   perception_manager  
   std_msgs
 )


###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
   INCLUDE_DIRS include
   CATKIN_DEPENDS geometry_msgs perception_manager rospy std_msgs ros_cpp_utils
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/yumi_manager.cpp
# )


#add_executable(lego_manager_node src/yumi_manager/lego_manager_node.cpp)
#add_executable(leap_manager_node src/yumi_manager/leap_manager_node.cpp)
#add_executable(unknown_object_manager_node src/yumi_manager/unkown_object_manager_node.cpp)
add_executable(test_manager_node src/yumi_manager/test_node.cpp include/yumi_manager/yumi_manager.h src/yumi_manager/yumi_manager.cpp)
add_executable(leap_manager_node src/yumi_manager/leap_manager.cpp include/yumi_manager/yumi_manager.h src/yumi_manager/yumi_manager.cpp)



## Add cmake target dependencies of the executable
## same as for the library above
#add_dependencies(lego_manager_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(leap_manager_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
#add_dependencies(unknown_object_manager_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(test_manager_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} )

## Specify libraries to link a library or executable target against
target_link_libraries(leap_manager_node
   ${catkin_LIBRARIES} ${OpenCV_LIBRARIES}
 )

target_link_libraries(test_manager_node
   ${catkin_LIBRARIES} ${OpenCV_LIBRARIES}
 )

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_yumi_manager.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
