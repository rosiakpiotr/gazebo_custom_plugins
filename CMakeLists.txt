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

# further dependencies manually.
# find_package(<dependency> REQUIRED)
link_directories(${gazebo_dev_LIBRARY_DIRS})

add_library(gazebo_motor_model_vpp SHARED
  src/gazebo_motor_model_vpp.cpp
)
target_include_directories(gazebo_motor_model_vpp PUBLIC include)
ament_target_dependencies(gazebo_motor_model_vpp
  gazebo_dev
  gazebo_msgs
  gazebo_ros
  rclcpp
)
ament_export_libraries(gazebo_motor_model_vpp)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)

  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  # set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  # set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

install(DIRECTORY include/
  DESTINATION include)

install(TARGETS
  gazebo_motor_model_vpp
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)