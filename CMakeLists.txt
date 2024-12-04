cmake_minimum_required(VERSION 3.5)
project(gazebo_custom_plugins)

set(CMAKE_BUILD_TYPE Debug)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(gazebo_dev REQUIRED)
find_package(gazebo_msgs REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(rclcpp REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(gazebo_custom_msg REQUIRED)

add_library(gazebo_ros2_to_gz_traznsport SHARED
  src/ros2_to_gz_transport.cpp
)

message(STATUS "Include dirs GAZEBO_CUSTOM_MSG: ${gazebo_custom_msg_INCLUDE_DIRS}")

target_include_directories(gazebo_ros2_to_gz_traznsport PUBLIC ${gazebo_custom_msg_INCLUDE_DIRS})
target_include_directories(gazebo_ros2_to_gz_traznsport PUBLIC include)

ament_target_dependencies(gazebo_ros2_to_gz_traznsport
  gazebo_dev
  gazebo_msgs
  gazebo_ros
  rclcpp
  px4_msgs
  gazebo_custom_msg
)

# add_dependencies(gazebo_ros2_to_gz_traznsport gazebo_custom_msg)
# target_link_libraries(gazebo_ros2_to_gz_traznsport PUBLIC gazebo_custom_msg::sknr_msgs)
ament_export_libraries(gazebo_ros2_to_gz_traznsport)

ament_package()

install(DIRECTORY include/
  DESTINATION include)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

install(TARGETS
  gazebo_ros2_to_gz_traznsport
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

# Inject compiled .so plugin into PX4-Autopilot simulation structure
install(TARGETS
  gazebo_ros2_to_gz_traznsport
  DESTINATION ~/PX4-Autopilot/build/px4_sitl_default/build_gazebo-classic)

install(DIRECTORY
  worlds
  DESTINATION share/${PROJECT_NAME}/
)