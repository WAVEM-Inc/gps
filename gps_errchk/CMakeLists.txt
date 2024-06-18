cmake_minimum_required(VERSION 3.8)
project(gps_errchk)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
include_directories(
  include
  include/${PROJECT_NAME}
  #${EXTERNAL_INCLUDE_DIRS}
)

set(source_set
  src/main.cpp 
  src/gps_errchk.cpp
)

set(dependencies 
  rclcpp
  sensor_msgs
  nav_msgs
)

set(node_name
  gps_errchk_node
)

add_executable(${node_name}
  ${source_set}
)

ament_target_dependencies(${node_name}
  ${dependencies}
)

install(TARGETS
  ${node_name}
  DESTINATION lib/${PROJECT_NAME}
)
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
