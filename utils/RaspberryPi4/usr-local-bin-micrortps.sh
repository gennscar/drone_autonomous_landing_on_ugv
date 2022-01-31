#!/bin/bash

#####
## Uncomment the following code to test the swarming application
#####

#####
## Before using this file, remember to
## 	- change the ip address of the discovery server (if needed)
## 	- change the MAVSYS_ID. This must match the MAVSYS_ID on the relative Pixhawk4 autopilot
## 	- change the namespace (-n X500_"drone number") of the drone, and the first parameter of function "launchdrone" accordingly
##    notice that it is not mandatory that the id corresponds to the one indicated on the drone's label
#####

source /opt/ros/foxy/setup.bash
source /home/ubuntu/px4_ros_com_ros2/install/setup.bash
source /home/ubuntu/ros2_px4_ws/install/setup.bash
source /home/ubuntu/ros2_px4_ws/src/ros2_px4_swarming/other/ros2_px4_swarming_aliases.sh

MAVSYS_ID=2
export ROS_DISCOVERY_SERVER=192.168.1.99:11811          # comment this line to launch the drone without the DDS server
micrortps_agent -b 921600 -d /dev/ttyAMA0 -n X500_1 &

sleep 5

launchdrone 1 $MAVSYS_ID                                # to get info on this function, see "ros2_px4_ws/src/ros2_px4_swarming/other/README.md"


###############################


#####
## Uncomment the following code to test automatic landing application
#####

# source /opt/ros/foxy/setup.bash
# source /home/ubuntu/px4_ros_com_ros2/install/setup.bash
# source /home/ubuntu/ros2_px4_ws/install/setup.bash
# source /home/ubuntu/ros2_px4_ws/src/ros2_px4_swarming/other/ros2_px4_swarming_aliases.sh
#
# micrortps_agent -b 921600 -d /dev/ttyAMA0 -n X500_1 &
#
# sleep 5
#
# ros2 launch ros2_px4_launch_files drone_controller.launch.py &
# ros2 launch ros2_px4_launch_files real_drone_positioning.launch.py
