cmake_minimum_required(VERSION 3.8)
project(bootcamp2_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

include_directories(
  include
  /usr/local/include
)

link_directories(
  /usr/local/lib
)

find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rclcpp REQUIRED)


add_executable(${PROJECT_NAME}_comm src/bootcamp_comm.cpp)
ament_target_dependencies(${PROJECT_NAME}_comm 
  rclcpp 
  std_msgs 
  sensor_msgs
)
target_link_libraries(${PROJECT_NAME}_comm 
  dxl_x64_cpp
)

add_executable(bootcamp2_dxl_sync src/bootcamp_dxl_sync.cpp)
ament_target_dependencies(bootcamp2_dxl_sync
  rclcpp 
  std_msgs 
  sensor_msgs
)
target_link_libraries(bootcamp2_dxl_sync
  dxl_x64_cpp
)

install(TARGETS
  ${PROJECT_NAME}_comm
  bootcamp2_dxl_sync
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()