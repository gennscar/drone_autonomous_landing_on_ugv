**ros2_px4_swarming**
===

---
***Important***
---

For the simulation and the described commands to work, the ROS2 workspace *ros2_px4_ws* should be in the `$HOME` directory of the pc, as well as the *PX4-Autopilot*, *px4_ros_com_ros2*, and *gazebo_ros_pkgs* folders. For commodity, it is suggested to add the command `source ~/ros2_px4_ws/src/ros2_px4_swarming/other/ros2_px4_swarming_aliases.sh` to the `~/.bashrc` file.

---
***ros2_px4_swarming***
---

##### **anchorDrone&#46;py**
It is the code running on each drone of the swarm. It initializes the parameteres from *params.yaml* and it is responsible of reacting to the requests for the vehicle. When run, it connects to the Pixhawk and it puts the drone in an idle state, where it waits for external commands. When takeoff is commanded, the vehicle waits for its peers to be ready as well, then reaches a predefined altitude and, when the whole formation is ready, the swarming algorithm is run. If the swarm loses contact with one of its members, it is not considered anymore in the swarming algorithm and a new formation is generated.

##### **GPSSimulator&#46;py**
It continuously publishes dummy data on the visual odometry topic of every drone. In this way, the vehicle thinks to know its global position. It is useful to run the code indoor, without the propellers mounted.

##### **numAnchorsNode&#46;py**
It continuously publishes the number of vehicles in the formation.

##### **performanceAnalyzer&#46;py**
It publishes information about the performance of the system (tracking error, inter-anchors distances, synchronization error, ...), so that the architecture could be analyzed through graphs and plots.

##### **plotBag&#46;py**
It plots the data stored in a specific bag file (tracking error, inter-anchors distances, target UWB distances, trajectories, altitudes, tracking velocity, velocities, positions, synchronization error, ...).

##### **printInformationNode&#46;py**
It reads a topic on which every drone publishes information for debug, and it consequently prints every message to screen.

##### **targetRover&#46;py**
It is the node through which the followed target is commanded in simulation. Like *anchorDrone&#46;py*, it subscribes to a service to receive the inputs from the user.

##### **topicsPlotter&#46;py**
It plots the data stored in the .csv files by *topicsRecorder&#46;py*.

##### **topicsRecorder&#46;py**
It records the data published on some chosen topics, together with a timestamp. It is needed because, at present time, it is not possible to use the *rosbag* command while using the DDS server.

##### **trackingVelocityCalculator&#46;py**
It is the brain of the formation: it applies the PID controller to the data received from the anchors and it publishes the resulting tracking velocity, that is equal for every member of the swarm.

---
***launch***
---

##### **launch_drone&#46;launch&#46;py**
It is the script launching the node running on the physical drone for testing. It runs *anchorDrone&#46;py*.

##### **launch_simulation&#46;launch&#46;py**
It is the script launching all the needed pieces of the simulation. It runs *numAnchorsNode&#46;py*, *targetRover&#46;py*, *anchorDrone&#46;py*, *trackingVelocityCalculator&#46;py*, and *performanceAnalyzer&#46;py*.

##### **launch_test&#46;launch&#46;py**
It is the script launching the central nodes needed to test the system on hardware. It runs *numAnchorsNode&#46;py*, *targetRover&#46;py*, *trackingVelocityCalculator&#46;py* and *performanceAnalyzer&#46;py*.

---
***models***
---

##### **iris**
The model of the drone used in simulation.

##### **r1_rover**
The model of the rover used in simulation.

##### **uwb_anchor**
The model of the UWB anchor used in simulation.

##### **uwb_sensor**
The model of the UWB sensor used in simulation.

---
***other***
---

##### **gazebo_sitl_multiple_run&#46;sh**
It opens gazebo, spawning the desired number and type of vehicles in the chosen world for the simulation.

##### **jinja_gen&#46;py**
It initializes some parameters, including the UWB anchor's identifier.

##### **launchedVehicles&#46;txt**
It stores the number of drones and the information about the presence of the target vehicle. These data are retrieved by the proper scripts to give everybody the needed information for the system to work properly.

##### **README&#46;md**
This document.

##### **ros2_px4_swarming_aliases&#46;sh**
This file, which should be sourced in the *.bashrc*, generates some aliases, useful to easily launch simulations and tests of this package.
* `buildws`: builds the whole *ros2_px4_ws* workspace with colcon
* `buildswarming`: builds only the *ros2_px4_swarming* package, included in the *ros2_px4_ws* workspace
* `killgazebosimulation`: kills px4, gazebo, the microRTPS agent and ROS2
* `agent 0`: generates the microRTPS agent, with namespace *X500_0*
* `gazebosimulation 4 1`: launches a gazebo simulation with 4 drones and 1 rover
* `launchsimulation`: runs *launch_simulation&#46;launch&#46;py*
* `launchtest 4 1`: runs *launch_test&#46;launch&#46;py*, for a test with 4 drones and 1 rover
+ `swarmtakeoff`: commands the formation to takeoff
+ `swarmhover`: commands the formation to hover at the current location
+ `swarmswarming`: commands the formation to start the swarming algorithm
+ `swarmgoto 45.06 7.66`: commands the formation to move to latitude 45.06 and longitude 7.66
+ `swarmreturn`: commands each drone to return to its starting point, that is the one where it concluded the takeoff (it could be **dangerous**, because the drones move independently and not as a whole)
+ `swarmland`: commands the formation to land at the current location
+ `swarmrestart`: commands each drone to restart, that is reset the parameters and get ready to fly again
* `dronetakeoff 0 -3`: commands drone 0 to takeoff to an altitude of 3 meters
* `dronehover 0`: commands drone 0 to hover at its current location
* `droneswarming 0`: commands drone 0 to start the swarming algorithm
* `dronegoto 0 1 2 -3`: commands drone 0 to go to position (1, 2, -3) with respect to its boot position, considering a NED reference system
* `dronesethorizontalvelocity 0 1 2`: commands drone 0 to set a velocity of (1, 2), considering a NED reference system
* `dronereturnhome 0`: commands drone 0 to return to its starting point, that is the one where it concluded the takeoff
* `droneland 0`: commands drone 0 to land at its current location
* `dronerestart 0`: commands drone 0 to restart, that is reset the parameters and get ready to fly again
+ `targetroverhold`: commands the target to stand still
+ `targetroverstraight 1`: commands the target to move straight with a speed of 1 m/s
+ `targetrovercircle 1 2`: commands the target to move describing a circle, with throttle 1 m/s and yaw rate 2 rad/s
+ `targetroverrandom`: commands the target to move randomly
+ `targetrovergoto 1 2`: commands the target to move to (1, 2) with respect to its spawning location
+ `targetroverrotate 1`: commands the target to rotate with yaw rate 1 rad/s
+ `targetroverturn 90`: commands the target to rotate of 90° on its z axis 
+ `targetroversquare`: commands the target to move describing a square
* `ddsserver`: opens a DDS server at 192.168.1.99:11811
* `rds`: exports ROS\_DISCOVERY\_SERVER
* `sendworkspace 0`: sends the *ros2_px4_ws* workspace to drone 0
* `sendpackage 0`: sends the *ros2_px4_swarming* package to drone 0
* `sendparams 0`: sends *drone_params&#46;yaml* to drone 0
* `connectssh 0`: connects via ssh to drone 0
* `launchdrone 0 1`: command to launch *anchorDrone&#46;py* on the RaspberryPi, with ID=0 and MAVSYS_ID=1
* `srvstatus`: command to check the status of the service on the RaspberryPi
* `srvrestart`: command to restart the service on the RaspberryPi
* `srvstop`: command to stop the service on the RaspberryPi
+ `plotbag rosbag2_2021_09_02-11_33_25`: plots the data stored in *rosbag2_2021_09_02-11_33_25*
+ `plotlastbag`: plots the data stored in the last bag created
+ `plotcsv 2021_09_02-11_33_25`: plots the data stored in *2021_09_02-11_33_25*
+ `plotlastcsv`: plots the data stored in the last csv created

So for example, to launch a simulation with 4 drones and 1 target, the commands should be:
-  in every used terminal, run `source ~/ros2_px4_ws/src/ros2_px4_swarming/other/ros2_px4_swarming_aliases.sh`
1. in a terminal, run `gazebosimulation 4 1`
2. in a second terminal, run `launchsimulation`
3. in a third terminal, run `swarmtakeoff`
4. in a fourth terminal, run `targetroverrandom`

---

To launch a real world test, do:
1. turn on the drones, the command `launchdrone` is executed automatically at startup

On the computer:
- in every used terminal, run `source ~/ros2_px4_ws/src/ros2_px4_swarming/other/ros2_px4_swarming_aliases.sh`
2. in a terminal, run `ddsserver`
3. in a second terminal, run `rds && launchtest 2 0` to initialize the test with two drones and no target rover
4. in a third terminal, run `rds && swarmtakeoff`
5. in the third terminal, run `rds && swarmland`
