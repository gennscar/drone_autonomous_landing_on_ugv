# ROS2 PX4 WS
This is a set of ROS2 packages that work with PX4 Drone Autopilot

## Authors
1. Cosimo Conte [@cosmo97](https://github.com/cosmo97)
2. Gennaro Scarati [@gennscar](https://github.com/gennscar)
3. Matteo Celada [@MatteoCelada](https://github.com/MatteoCelada)
4. Giovanni Fantin []()

## Installation
### PC-side
1. Install [Ubuntu 20.04](https://ubuntu.com/server/docs/installation)
2. Clone [this repository](https://github.com/PIC4SeRCentre/ros2_px4_ws) running `git clone https://github.com/PIC4SeRCentre/ros2_px4_ws.git`
3. Clone the PX4 firmware with (`git clone https://github.com/PX4/PX4-Autopilot.git --recursive`)
4. Checkout to the correct version of the PX4 firmware running `cd PX4-Autopilot && git checkout f15eefc`
5. Substitute file [_ros2_px4_ws/utils/PX4-Autopilot/uorb_rtps_message_ids.yaml_](utils/PX4-Autopilot/uorb_rtps_message_ids.yaml) in folder _PX4-Autopilot/msg/tools_
6. Build PX4-Autopilot running `bash ./PX4-Autopilot/Tools/setup/ubuntu.sh`
7. Update the submodules running `git submodule sync --recursive && git submodule update --init --recursive`
8. Make PX4-Autopilot running `cd PX4-Autopilot && make px4_sitl_rtps gazebo`
9. (Optional) Install [QGroundControl](https://docs.qgroundcontrol.com/master/en/getting_started/download_and_install.html)
10. Follow [this guide](https://docs.px4.io/master/en/ros/ros2_comm.html) to set up ROS2 Foxy
11. After cloning the ROS2 bridge packages as described [here](https://docs.px4.io/master/en/ros/ros2_comm.html#build-ros-2-workspace), checkout to [this version](https://github.com/PX4/px4_ros_com/tree/3b577d6e50c1f1d31f4ffa218722093a41d166fc) running `cd $HOME/px4_ros_com_ros2/src/px4_ros_com && git checkout 3b577d6` and [this version](https://github.com/PX4/px4_msgs/commit/cb455c2914b3b49d65b449caaa268116e27edccb) running `cd $HOME/px4_ros_com_ros2/src/px4_msgs && git checkout cb455c2`
12. Before building the code, as described [here](https://docs.px4.io/master/en/ros/ros2_comm.html#build-ros-2-workspace), substitute file [_ros2_px4_ws/utils/px4_ros_com/uorb_rtps_message_ids.yaml_](utils/px4_ros_com/uorb_rtps_message_ids.yaml) in folder _px4_ros_com_ros2/src/px4_ros_com/templates_
13. Install _gazebo_ros_pkgs_ running `sudo apt install ros-foxy-gazebo-ros-pkgs`
14. Build the workspace running `cd ros2_px4_ws && colcon build --symlink-install`
15. See [_ros2_px4_ws/src/ros2_px4_swarming/other/README.md_](src/ros2_px4_swarming/other/README.md) for more information on the swarming package

### Drone-side
1. Install [Ubuntu 20.04 server](https://ubuntu.com/download/server) on the RaspberryPi4
2. Clone [this repository](https://github.com/PIC4SeRCentre/ros2_px4_ws) running `git clone https://github.com/PIC4SeRCentre/ros2_px4_ws.git`
3. After changing the _.yaml_ file in the PX4 firmware on the PC, upload it on the autopilot via `make px4_fmu-v5_rtps upload`
4. Follow the steps described in the previous paragraph to install ROS2 Foxy and _px4_ros_com_, reminding also in this case to replace the _.yaml_ file before building 
5. Build the workspace running `cd ros2_px4_ws && colcon build --symlink-install --packages-skip ros2_px4_gazebo` (Gazebo package will not run on the RaspberryPi, and trying to build it will give an error)
6. Set up the service to automatically launch the code on the drone following [this guide](https://nxp.gitbook.io/8mmnavq/navq-developer-guide/software-support/installing-ros2-foxy/autostart-micrortps-client-via-systemd-on-navq). Use the files stored in [_ros2_px4_ws/utils/RaspberryPi4_](utils/RaspberryPi4), and remember to change the parameters as described in the file itself

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
- `sudo apt install python3-testresources`

## Packages
1. [ros2_px4_control](src/ros2_px4_control) This package contains nodes to control the drone interfacing with PX4
2. [ros2_px4_estimation](src/ros2_px4_estimation) This package contains state estimation nodes for the drone or other components
3. [ros2_px4_functions](src/ros2_px4_functions) This package contains all functions that are not directly ROS2 nodes used by nodes
4. [ros2_px4_gazebo](src/ros2_px4_gazebo) 
5. [ros2_px4_interfaces](src/ros2_px4_interfaces) This package contains all interfaces required from the other packages
of this repo
6. [ros2_px4_launch_files](src/ros2_px4_launch_files) This package contains all the launch files to perform automatic action with multiple nodes
7. [ros2_px4_swarming](src/ros2_px4_swarming) This package contains nodes and launch files of the swarming algorithm
8. [ros2_px4_testing](src/ros2_px4_testing)
