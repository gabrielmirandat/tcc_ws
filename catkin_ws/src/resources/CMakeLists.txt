cmake_minimum_required(VERSION 2.8.3)
project(resources)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  sensor_msgs
  p2os_msgs
  image_transport
  cv_bridge
)

catkin_package(
 INCLUDE_DIRS nodes/include
 CATKIN_DEPENDS roscpp geometry_msgs sensor_msgs p2os_msgs image_transport cv_bridge
)

include_directories(nodes/include ${catkin_INCLUDE_DIRS})

add_executable(joystick_node nodes/src/joystick_node.cc nodes/src/joystick.cc)
add_executable(camera_node nodes/src/camera.cc)
target_link_libraries(joystick_node ${catkin_LIBRARIES})
target_link_libraries(camera_node ${catkin_LIBRARIES})
