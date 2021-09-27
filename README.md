# ROS2 PX4 WS
This is a set of ROS2 packages that work with PX4 Drone Autopilot

## Authors
1. Cosimo Conte [@cosmo97](https://github.com/cosmo97)
2. Gennaro Scarati [@gennscar](https://github.com/gennscar)
3. Matteo Celada [@MatteoCelada](https://github.com/MatteoCelada)

## Installation
1. Follow [this guide](https://docs.px4.io/master/en/ros/ros2_comm.html) to set up ROS2 and PX4 environment
2. Clone this repository
3. Build all the packages with the command `colcon build --symlink-install` inside the ros2-px4-ws folder
4. Source the workspace with `source ros2_px4_ws/install/setup.bash`

## Packages
1. [ros2_px4_control](src/ros2_px4_control) This package contains nodes to control the drone interfacing with PX4
2. [ros2_px4_estimation](src/ros2_px4_estimation) This package contains state estimation nodes for the drone or other components
3. [ros2_px4_functions](src/ros2_px4_functions) This package contains all functions that are not directly ROS2 nodes used by nodes
4. [ros2_px4_interfaces](src/ros2_px4_interfaces) This package contains all interfaces required from the other packages
of this repo
5. [ros2_px4_launch_files](src/ros2_px4_launch_files) This package contains all the launch files to perform automatic action with multiple nodes
