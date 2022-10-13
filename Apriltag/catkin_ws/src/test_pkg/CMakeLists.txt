cmake_minimum_required(VERSION 3.0.2)
project(test_pkg)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES my_ros_pkg
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(publish src/publish.cpp)
target_link_libraries(publish ${catkin_LIBRARIES})
# add_dependencies(publish ${PROJECT_NAME}_generate_messages_cpp)

add_executable(subscribe src/subscribe.cpp)
target_link_libraries(subscribe ${catkin_LIBRARIES})
# add_dependencies(subscribe ${PROJECT_NAME}_generate_messages_cpp)

