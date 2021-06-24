cmake_minimum_required(VERSION 3.5 FATAL_ERROR)
project(ros2_px4_gazebo)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# WIN32 compatibilities
if(WIN32)
  add_compile_definitions(
    # For math constants
    _USE_MATH_DEFINES
    # Minimize Windows namespace collision
    NOMINMAX
    WIN32_LEAN_AND_MEAN
  )
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(gazebo REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(ros2_px4_interfaces REQUIRED)

include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})

# UWB plugin
add_library(ros2_px4_gazebo_uwb SHARED src/ros2_px4_gazebo_uwb.cpp)
target_include_directories(ros2_px4_gazebo_uwb PUBLIC include)
ament_target_dependencies(ros2_px4_gazebo_uwb
  "rclcpp"
  "gazebo"
  "gazebo_ros"
  "ros2_px4_interfaces"
)
ament_export_libraries(ros2_px4_gazebo_uwb)

# Ament export
ament_export_include_directories(include)
ament_export_dependencies(rclcpp)
ament_export_dependencies(gazebo)
ament_export_dependencies(gazebo_ros)
ament_export_dependencies(ros2_px4_interfaces)

ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/env-hooks/${PROJECT_NAME}.dsv.in")

ament_package()

# Install
install(TARGETS
  ros2_px4_gazebo_uwb
  DESTINATION lib
)

install(DIRECTORY
  models
  DESTINATION share/${PROJECT_NAME}/
)

install(DIRECTORY
  worlds
  DESTINATION share/${PROJECT_NAME}/
)

install(DIRECTORY
  media
  DESTINATION share/${PROJECT_NAME}/
)
