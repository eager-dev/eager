cmake_minimum_required(VERSION 2.8.3)
project(action_server)

find_package(catkin REQUIRED COMPONENTS rospy std_msgs)

catkin_package()

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
