# ROS2 PX4 WS
This is a set of ROS2 packages that work with PX4 Drone Autopilot

## Authors
1. Cosimo Conte [@cosmo97](https://github.com/cosmo97)
2. Gennaro Scarati [@gennscar](https://github.com/gennscar)
3. Matteo Celada [@MatteoCelada](https://github.com/MatteoCelada)

## Installation
### PC-side
1. Install [Ubuntu 20.04](https://ubuntu.com/server/docs/installation)
2. Clone [this repository](https://github.com/PIC4SeRCentre/ros2_px4_ws) running `git clone https://github.com/PIC4SeRCentre/ros2_px4_ws.git`
3. Clone [this version](https://github.com/PX4/PX4-Autopilot/tree/v1.12.0-beta5) of the PX4 firmware (`git clone --recurse-submodules https://github.com/PX4/PX4-Autopilot/tree/v1.12.0-beta5`)
4. Substitute file _ros2_px4_ws/utils/PX4-Autopilot/uorb_rtps_message_ids.yaml_ in folder _PX4-Autopilot/msg/tools_
5. Build PX4-Autopilot running `bash ./PX4-Autopilot/Tools/setup/ubuntu.sh`
6. Make PX4-Autopilot running `cd PX4-Autopilot && make px4_sitl_rtps gazebo`
7. (Optional) Install [QGroundControl](https://docs.qgroundcontrol.com/master/en/getting_started/download_and_install.html)
8. Follow [this guide](https://docs.px4.io/master/en/ros/ros2_comm.html) to set up ROS2 Foxy
9. After cloning the ROS2 bridge packages as described [here](https://docs.px4.io/master/en/ros/ros2_comm.html#build-ros-2-workspace), checkout to [this version](https://github.com/PX4/px4_ros_com/tree/3b577d6e50c1f1d31f4ffa218722093a41d166fc) running `cd $HOME/px4_ros_com_ros2/src/px4_ros_com && git checkout 3b577d6` and [this version](https://github.com/PX4/px4_msgs/commit/cb455c2914b3b49d65b449caaa268116e27edccb) running `cd $HOME/px4_ros_com_ros2/src/px4_msgs && git checkout cb455c2`
10. Before building the code, as described [here](https://docs.px4.io/master/en/ros/ros2_comm.html#build-ros-2-workspace), substitute file _ros2_px4_ws/utils/px4_ros_com/uorb_rtps_message_ids.yaml_ in folder _px4_ros_com_ros2/src/px4_ros_com/templates_
11. Install _gazebo_ros_pkgs_ running `sudo apt install ros-foxy-gazebo-ros-pkgs`
12. Build the workspace running `cd ros2_px4_ws && colcon build --symlink-install`

#### Solutions to possible problems
If needed:
- install Java jdk-11 with command `sudo apt-get install openjdk-11-jdk`
- install SDKMAN with `curl -s "https://get.sdkman.io" | bash` and initialize it with `source "$HOME/.sdkman/bin/sdkman-init.sh"`
- be careful to install the suggested version of Gradle (at present time it is version 6.3)
- install cmake with `sudo apt install cmake`
- during Fast DDS installation from source, install `sudo apt-get install libssl-dev`
- after installing ROS2 Foxy, run `sudo apt-get install python3-launchpadlib`
- when building px4_ros_com_ros2, if the terminal crashes running `source build_ros2_workspace.bash`, try running `./build_ros2_workspace.bash` instead
- install _scipy_ if required with `sudo apt-get install -y python3-scipy`
