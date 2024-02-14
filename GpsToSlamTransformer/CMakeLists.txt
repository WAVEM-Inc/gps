cmake_minimum_required(VERSION 3.5)
project(gps_to_slam_transformer)

if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

include_directories(
  include
  include/${PROJECT_NAME}
  /usr/local/include
  /usr/local/include/proj
)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(gps_navigation_msgs REQUIRED)
find_package(PROJ REQUIRED CONFIG)

add_executable(
  transformer
  include/math/math.hpp
  src/gps_to_slam_transformer/gps_to_slam_transformer.cpp
  src/main.cpp
)

ament_target_dependencies(
  transformer
  rclcpp
  rclcpp_action
  sensor_msgs
  geometry_msgs
  nav2_msgs
  gps_navigation_msgs
  PROJ
)

add_definitions(-DACCEPT_USE_OF_DEPRECATED_PROJ_API_H)

target_link_libraries(
  transformer
  PROJ::proj
)

install(TARGETS
  transformer
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()