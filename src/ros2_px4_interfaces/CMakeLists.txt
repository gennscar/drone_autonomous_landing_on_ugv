cmake_minimum_required(VERSION 3.5)
project(ros2_px4_interfaces)

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
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

set(msg_files
  "msg/DistanceStamped.msg"
  "msg/DistanceStampedArray.msg"
  "msg/Error.msg"
  "msg/UnitVector.msg"
  "msg/UnitVectorArray.msg"
  "msg/UwbSensor.msg"
  "msg/Yaw.msg"
  "msg/VelocityVector.msg"
  "msg/Uwb.msg"
)

set(srv_files
  "srv/ControlMode.srv"
  "srv/DroneCustomCommand.srv"
  "srv/GpsEnable.srv"
  "srv/SwarmCommand.srv"
  "srv/TargetCustomCommand.srv"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
  DEPENDENCIES std_msgs geometry_msgs
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
