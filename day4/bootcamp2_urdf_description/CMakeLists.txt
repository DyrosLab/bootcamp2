cmake_minimum_required(VERSION 3.8)
project(bootcamp2_urdf_description)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)

include_directories(
  include
  ${rclcpp_INCLUDE_DIRS}
)

link_directories(
  /usr/local/lib
  /usr/include
)

install(DIRECTORY
  launch meshes urdf
  DESTINATION share/${PROJECT_NAME}
)

ament_package()