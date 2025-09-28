# Description
An autonomous drone landing system achieving centimeter-level precision on moving UGVs, using an EKF-based localization pipeline integrating drone, UGV, and Ultra-Wideband (UWB) data.

## Installation
### PC-side
1. Install [Ubuntu 20.04](https://ubuntu.com/server/docs/installation)
2. Clone this repository running `git clone git@github.com:gennscar/drone_autonomous_landing_on_ugv.git`
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

## Autonomous landing package guide

In order to have a basic understanding of the autonomous landing system you can read my thesis [here](https://webthesis.biblio.polito.it/21180/). Note that in the real implementation the user has to manually send the takeoff first and only then the autonomous landing command. This has been done in order to debug the system but it can easily be automatized. 
### Autonomous landing simulation guide
1. Insert [iris](other_material/iris) (substitute the original one) and [rover_uwb](other_material/rover_uwb) folders inside the folder `PX4-Autopilot/Tools/sitl_gazebo/models`. Substitute [empty.world](other_material/empty.world) inside the folder `PX4-Autopilot/Tools/sitl_gazebo/worlds`.
2. Open 4 terminals.
3. In the first type: `ros2 launch ros2_px4_launch_files drone_rover_clients_agents.launch.py`. This will launch the Gazebo simulation with a drone and a rover. Make sure to change the `gazebo_sitl_multiple_run.sh` path inside the file [drone_rover_clients_agents.launch.py](src/ros2_px4_launch_files/launch/drone_rover_clients_agents.launch.py) according to the one in your ubuntu environment.
4. In the second one type: `ros2 launch ros2_px4_launch_files drone_rover_positioning.launch.py`. This will enable the uwb-based positioning.
5. In the third one type: `ros2 launch ros2_px4_launch_files drone_controller_sim.launch.py`. This will launch the drone controller algorithm. Now it is possible to send commands to the drone. 

Note that this first three terminal need always to be running. Without the second one the drone can be still controlled but it won't be able to follow on land on the rover. If the second terminal crashes or it is closed during a chase or an autonomous landing, a custom failsafe will make the drone stop and abort the operation. Without the third one the drone won't be able to receive any commands and therefore it will land. 

6. In the fourth you can send commands to the drone or the rover. A comprehensive list of them is shown below:
    Drone commands:
    - Send the takeoff command to the drone: `ros2 service call /drone/ControlMode_Service ros2_px4_interfaces/srv/ControlMode '{control_mode: 'takeoff'}'` 
    - Send the land command to the drone: `ros2 service call /drone/ControlMode_Service ros2_px4_interfaces/srv/ControlMode '{control_mode: 'land'}'`
    - Make the drone follow the rover: `ros2 service call /drone/ControlMode_Service ros2_px4_interfaces/srv/ControlMode '{control_mode: 'follow_target'}'`
    - Make the drone follow and autonomously land on the rover: `ros2 service call /drone/ControlMode_Service ros2_px4_interfaces/srv/ControlMode '{control_mode: 'land_on_target'}'`
    Rover commands:
    - Make the rover move forward with velocity 1m/s: `ros2 topic pub /rover/cmd_vel geometry_msgs/Twist '{linear: {x: 1.0}, angular: {z: 0.0}}' -1`
    - Make the rover move back with velocity 1m/s: `ros2 topic pub /rover/cmd_vel geometry_msgs/Twist '{linear: {x: -1.0}, angular: {z: 0.0}}' -1`
    - Make the rover stop: `ros2 topic pub /rover/cmd_vel geometry_msgs/Twist '{linear: {x: 0}, angular: {z: 0.0}}' -1`
    Note that you can change both the linear and angular value. Changing the second one will make the rover steer.

It is also possible to automatize the rover movement by opening a fifth terminal and typing: `ros2 run ros2_px4_control rover_controller`. This will launch the script [rover_controller.py](src/ros2_px4_control/rover_controller.py). It will make the rover move randomly. You can add noise and allow the rover steering by changing the parameters `noise` and `curvature` in the code. 

A possible way to simulate the landing would be sending the takeoff command in the fourth terminal, waiting the drone to be at the right altitude, then making the rover move in the fifth terminal and sending the land on target command in the fourth terminal. 

In order to measure the performance of the positioning algorithm, you can type `ros2 launch ros2_px4_launch_files drone_rover_positioning_error.launch.py` in another terminal. This will publish the positioning error on certain topics.

Note that the drone will abort the chase and autonomous landing if the rover is too far away. This is triggered by a custom failsafe.

### Autonomous landing real drone guide

In order to make the real drone autonomously land on the rover you need to:

1. Connect the drone and rover raspberry to the same wifi network.
2. Connect the arduino and compass to the rover raspberry.
3. Launch the rover attitude positioning by sending the following command to the rover raspberry: `ros2 launch ros2_px4_launch_files real_rover_positioning.launch.py`
4. Launch the positioning algorithms by sending the following command to the drone raspberry: `ros2 launch ros2_px4_launch_files real_drone_positioning.launch.py`
5. Start the drone controller by sending the following command to the drone raspberry: `ros2 launch ros2_px4_launch_files drone_controller.launch.py`. 

Make sure to change `drone_namespace_arg` and `vehicle_number` in the file [real_drone_positioning.launch.py](src/ros2_px4_launch_files/launch/real_drone_positioning.launch.py) and `drone_name` inside [drone_controller.launch.py](src/ros2_px4_launch_files/launch/drone_controller.launch.py) according to the drone number. Example: in the case of the drone X500_0 you should set `default_value="/X500_0"` in `drone_namespace_arg` and `"vehicle_number": 1` in the first file and `drone_name = "/X500_0"` in the second one.

6. You can now send any of the commands listed above to the drone. Example: Send the takeoff command to the drone: `ros2 service call /drone/ControlMode_Service ros2_px4_interfaces/srv/ControlMode '{control_mode: 'takeoff'}'`. 
