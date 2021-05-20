cmake_minimum_required(VERSION 3.0.2)
project(ros_gym_bridge_gazebo)

# Check for c++11 / c++0x support
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "-std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "-std=c++0x")
else()
    message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()


## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
	rospy
	roscpp
	gazebo_ros
	std_msgs
	message_generation
  ros_gym_core
)

# Depend on system install of Gazebo
find_package(gazebo REQUIRED)

catkin_python_setup()

# Declare ROS service
add_service_files(FILES SetInt.srv)
generate_messages(DEPENDENCIES std_msgs)

link_directories(${GAZEBO_LIBRARY_DIRS})
include_directories(include ${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS})

catkin_package(
	INCLUDE_DIRS include
  DEPENDS
    roscpp
    gazebo_ros
    message_runtime
    std_msgs
)

install(DIRECTORY include/
   DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}
   FILES_MATCHING PATTERN "*.h"
)

add_library(${PROJECT_NAME} src/ros_gym_bridge_gazebo/gazebo_plugin/step_world.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
