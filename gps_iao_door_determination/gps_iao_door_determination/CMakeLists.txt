cmake_minimum_required(VERSION 3.5)
project(gps_iao_door_determination)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(gps_iao_door_msgs REQUIRED)

find_package(sensor_msgs REQUIRED)

include_directories(
  include
  include/${PROJECT_NAME}
)

add_executable(gps_iao_door_determination_node 
  src/main.cpp
  src/Determination.cpp
)

ament_target_dependencies(gps_iao_door_determination_node 
  rclcpp
  gps_iao_door_msgs

  sensor_msgs
)
install(TARGETS
  gps_iao_door_determination_node
  DESTINATION lib/${PROJECT_NAME}
)
ament_package()
