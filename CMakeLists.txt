cmake_minimum_required(VERSION 3.5)
project(ros2_realsense)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(create_image_periodic src/create_image_periodic.cpp)
target_link_libraries(create_image_periodic realsense2)
ament_target_dependencies(create_image_periodic rclcpp std_msgs)

add_executable(create_image_command src/create_image_command.cpp)
target_link_libraries(create_image_command realsense2)
ament_target_dependencies(create_image_command rclcpp std_msgs)

include_directories(./common ./third-party)

install(TARGETS
  create_image_periodic
  create_image_command
  DESTINATION lib/${PROJECT_NAME})

ament_package()