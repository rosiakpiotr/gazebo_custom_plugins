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

# XXX this approach is extremely error prone
# it would be preferable to either depend on the
# compiled headers from Gazebo directly
# or to have something entirely independent.
#
set(PROTOBUF_IMPORT_DIRS "")

foreach(ITR ${GAZEBO_INCLUDE_DIRS})
  if(ITR MATCHES ".*gazebo-[0-9.]+$")
    set(PROTOBUF_IMPORT_DIRS "${ITR}/gazebo/msgs/proto")
  endif()
endforeach()

# PROTOBUF_IMPORT_DIRS has to be set before
# find_package is called
find_package(Protobuf REQUIRED)
pkg_check_modules(PROTOBUF protobuf)

if("${PROTOBUF_VERSION}" VERSION_LESS "2.5.0")
  message(FATAL_ERROR "protobuf version: ${PROTOBUF_VERSION} not compatible, must be >= 2.5.0")
endif()

if("${GAZEBO_VERSION}" VERSION_LESS "6.0")
  message(FATAL_ERROR "You need at least Gazebo 6.0. Your version: ${GAZEBO_VERSION}")
else()
  message(STATUS "Gazebo version: ${GAZEBO_VERSION}")
endif()

# --------------------#
# Message Generation #
# --------------------#
set(mav_msgs
  msgs/CommandPitchAngle.proto
  msgs/VppState.proto
)

PROTOBUF_GENERATE_CPP(MAV_PROTO_SRCS MAV_PROTO_HDRS ${mav_msgs})

include_directories(${CMAKE_CURRENT_BINARY_DIR})

add_library(mav_msgs SHARED ${MAV_PROTO_SRCS})
target_link_libraries(mav_msgs ${PROTOBUF_LIBRARY})

# further dependencies manually.
# find_package(<dependency> REQUIRED)
link_directories(${gazebo_dev_LIBRARY_DIRS})

# install the generated protobuf headers
install(FILES
  ${MAV_PROTO_HDRS}
  DESTINATION
  include/${PROJECT_NAME}
)

add_library(gazebo_ros2_to_gz_traznsport SHARED
  src/ros2_to_gz_transport.cpp
)
target_include_directories(gazebo_ros2_to_gz_traznsport PUBLIC include)
ament_target_dependencies(gazebo_ros2_to_gz_traznsport
  gazebo_dev
  gazebo_msgs
  gazebo_ros
  rclcpp
  px4_msgs
)
add_dependencies(gazebo_ros2_to_gz_traznsport mav_msgs)
target_link_libraries(gazebo_ros2_to_gz_traznsport mav_msgs)
ament_export_libraries(gazebo_ros2_to_gz_traznsport)

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