cmake_minimum_required(VERSION 3.0.2)
project(eager_bridge_real)


## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  eager_core
  rospy
)

catkin_python_setup()
