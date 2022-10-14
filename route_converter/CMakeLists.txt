cmake_minimum_required(VERSION 3.0.2)
project(route_converter)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)


find_package(catkin REQUIRED COMPONENTS
  autoware_planning_msgs
  autoware_lanelet2_msgs
  geometry_msgs
  roscpp
  lanelet2_extension

)

# find_package(Boost REQUIRED COMPONENTS system)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES route_converter
  CATKIN_DEPENDS autoware_planning_msgs geometry_msgs roscpp
#  DEPENDS system_lib
)


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)



add_library(route_converter src/route_converter.cpp)
target_link_libraries(route_converter
  ${catkin_LIBRARIES}
)

add_executable(route_converter_node
  src/route_converter_node.cpp
)
target_link_libraries(route_converter_node
  route_converter
)

