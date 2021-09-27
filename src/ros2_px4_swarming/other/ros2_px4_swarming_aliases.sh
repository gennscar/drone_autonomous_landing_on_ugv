source /opt/ros/foxy/setup.bash
if [[ -f "/usr/share/gazebo/setup.sh" ]]; then
  source /usr/share/gazebo/setup.sh
fi
source $HOME/px4_ros_com_ros2/install/setup.bash
if [[ -f "$HOME/gazebo_ros_pkgs/install/setup.bash" ]]; then
  source $HOME/gazebo_ros_pkgs/install/setup.bash
fi
source $HOME/ros2_px4_ws/install/setup.bash

# region Build, launch and other
function buildws() {
  cd $HOME/ros2_px4_ws || exec $SHELL
  colcon build --symlink-install
  cd || exec $SHELL
}

function buildswarming() {
  cd $HOME/ros2_px4_ws || exec $SHELL
  colcon build --symlink-install --packages-select ros2_px4_swarming
  cd || exec $SHELL
}

function killgazebosimulation() {
  echo ""
  echo "Stopping the RTPS agents, gazebo, PX4."
  pkill px4
  pkill gzclient
  pkill gzserver
  pkill micrortps_agent
  eval "ros2 daemon stop"
  exec $SHELL
}

# RTPS agent alias
function agent() {
  n=${1:-0}
  rPort=$((2020 + 2 * n))
  sPort=$((2019 + 2 * n))

  namespace="X500_$n"
  echo "RTPS agent for $namespace"
  eval "micrortps_agent -t UDP -r $rPort -s $sPort -n $namespace"
}

function gazebosimulation() {
  trap "killgazebosimulation" SIGINT SIGTERM EXIT

  eval "ros2 daemon stop"

#  Export the number of drones and target to spawn
  numDrones=${1:-4}
  numTarget=${2:-1}

  if [ $numTarget -gt 0 ]
  then
    numTarget=1
  fi

  export NUM_DRONES=$numDrones
  export NUM_TARGET=$numTarget

#  Save the number of spawned drones and target in a file
  fileName="$HOME/ros2_px4_ws/src/ros2_px4_swarming/other/launchedVehicles.txt"
  echo "$numDrones" > $fileName
  echo "$numTarget" >> $fileName

#  cd ~/PX4-Autopilot
#  DONT_RUN=1 make px4_sitl_rtps gazebo
#  cd

eval ". $HOME/ros2_px4_ws/src/ros2_px4_swarming/other/gazebo_sitl_multiple_run.sh -t px4_sitl_rtps -s \"iris:$numDrones,r1_rover:$numTarget\" -l rtps &"

#  Launch the RTPS agents
  n=0
  while [ $n -lt $numDrones ]; do
    agent $n &
    n=$((n + 1))
  done
}

# ROS2 nodes alias
function launchsimulation() {
#  cd $HOME/ros2_px4_ws || exec $SHELL
#  colcon build --packages-select ros2_px4_swarming
#  cd || exec $SHELL

#  Count the vehicles spawned
  fileName="$HOME/ros2_px4_ws/src/ros2_px4_swarming/other/launchedVehicles.txt"
  n=0
  while read -r val; do
    if [ $n -eq 0 ]
    then
      numDrones=$val
    elif [ $n -eq 1 ]
    then
      numTarget=$val
    fi
    n=$((n + 1))
  done < $fileName

  export NUM_DRONES=$numDrones
  export NUM_TARGET=$numTarget

  ros2 launch ros2_px4_swarming launch_simulation.launch.py &

  sleep 10
  cd $HOME/ros2_px4_ws/src/ros2_px4_swarming/bagfiles || exec $SHELL
  fileName=$(ls -1 | tail -n 1)
  cp $HOME/ros2_px4_ws/src/ros2_px4_swarming/parameters/params.yaml $HOME/ros2_px4_ws/src/ros2_px4_swarming/bagfiles/$fileName
  cd || exec $SHELL

  fg 1
}

function launchtest() {
  numDrones=${1:-1}
  numTarget=${2:-0}

  export NUM_DRONES=$numDrones
  export NUM_TARGET=$numTarget

#  Save the number of drones and target in a file
  fileName="$HOME/ros2_px4_ws/src/ros2_px4_swarming/other/launchedVehicles.txt"
  echo "$numDrones" > $fileName
  echo "$numTarget" >> $fileName

#  n=0
#  while [ $n -lt $numDrones ]; do
#    dronerestart $n &
#    n=$((n + 1))
#  done

#  sleep 5

  ros2 launch ros2_px4_swarming launch_test.launch.py &

  sleep 10
  cd $HOME/ros2_px4_ws/src/ros2_px4_swarming/bagfiles || exec $SHELL
  fileName=$(ls -1 | tail -n 1)
  cp $HOME/ros2_px4_ws/src/ros2_px4_swarming/parameters/params.yaml $HOME/ros2_px4_ws/src/ros2_px4_swarming/bagfiles/$fileName
  cd || exec $SHELL

  fg 1
}

# Plot data from bag
function plotbag() {
  bagFileName=${1:fileName}
  cd $HOME/ros2_px4_ws/src/ros2_px4_swarming/bagfiles || exec $SHELL
  python3 $HOME/ros2_px4_ws/src/ros2_px4_swarming/ros2_px4_swarming/plotBag.py $bagFileName
}

function plotlastbag() {
  cd $HOME/ros2_px4_ws/src/ros2_px4_swarming/bagfiles || exec $SHELL
  fileName=$(ls -1 | tail -n 1)
  python3 $HOME/ros2_px4_ws/src/ros2_px4_swarming/ros2_px4_swarming/plotBag.py $fileName
  cd || exec $SHELL
}
# endregion

# region Swarming commands
function swarmtakeoff() {
#  Count the vehicles spawned
  fileName="$HOME/ros2_px4_ws/src/ros2_px4_swarming/other/launchedVehicles.txt"
  n=0
  while read -r val; do
    if [ $n -eq 0 ]
    then
      numDrones=$val
    fi
    n=$((n + 1))
  done < $fileName

  n=0
  while [ $n -lt $numDrones ]; do
    dronetakeoff $n &
    n=$((n + 1))
  done
}

function swarmhover() {
  eval "ros2 service call /trackingVelocityCalculator/SwarmCommand ros2_px4_interfaces/srv/SwarmCommand '{operation: 'hover'}'"
}

function swarmswarming() {
#  Count the vehicles spawned
  fileName="$HOME/ros2_px4_ws/src/ros2_px4_swarming/other/launchedVehicles.txt"
  n=0
  while read -r val; do
    if [ $n -eq 0 ]
    then
      numDrones=$val
    fi
    n=$((n + 1))
  done < $fileName

  n=0
  while [ $n -lt $numDrones ]; do
    droneswarming $n &
    n=$((n + 1))
  done
}

function swarmgoto() {
  lat=${1:-0.0}
  lon=${2:-0.0}
  eval "ros2 service call /trackingVelocityCalculator/SwarmCommand ros2_px4_interfaces/srv/SwarmCommand '{operation: 'goTo', lat: $lat, lon: $lon}'"
}

function swarmreturn() {
#  Count the vehicles spawned
  fileName="$HOME/ros2_px4_ws/src/ros2_px4_swarming/other/launchedVehicles.txt"
  n=0
  while read -r val; do
    if [ $n -eq 0 ]
    then
      numDrones=$val
    fi
    n=$((n + 1))
  done < $fileName

  n=0
  while [ $n -lt $numDrones ]; do
    dronereturnhome $n &
    n=$((n + 1))
  done

  fg 1
}

function swarmland() {
#  Count the vehicles spawned
  fileName="$HOME/ros2_px4_ws/src/ros2_px4_swarming/other/launchedVehicles.txt"
  n=0
  while read -r val; do
    if [ $n -eq 0 ]
    then
      numDrones=$val
    fi
    n=$((n + 1))
  done < $fileName

  n=0
  while [ $n -lt $numDrones ]; do
    droneland $n &
    n=$((n + 1))
  done

  fg 1
}

function swarmrestart() {
#  Count the vehicles spawned
  fileName="$HOME/ros2_px4_ws/src/ros2_px4_swarming/other/launchedVehicles.txt"
  n=0
  while read -r val; do
    if [ $n -eq 0 ]
    then
      numDrones=$val
    fi
    n=$((n + 1))
  done < $fileName

  n=0
  while [ $n -lt $numDrones ]; do
    dronerestart $n &
    n=$((n + 1))
  done

  fg 1
}
# endregion

# region Drones commands
function dronetakeoff() {
  n=${1:-0}
  z=${2:--1}
  eval "ros2 service call /X500_$n/DroneCustomCommand ros2_px4_interfaces/srv/DroneCustomCommand '{operation: 'takeoff', z: $z}'"
}

function dronehover() {
  n=${1:-0}
  eval "ros2 service call /X500_$n/DroneCustomCommand ros2_px4_interfaces/srv/DroneCustomCommand '{operation: 'hover'}'"
}

function droneswarming() {
  n=${1:-0}
  eval "ros2 service call /X500_$n/DroneCustomCommand ros2_px4_interfaces/srv/DroneCustomCommand '{operation: 'swarming'}'"
}

function dronegoto() {
  n=${1:-0}
  x=${2:-0}
  y=${3:-0}
  z=${4:--3}
  eval "ros2 service call /X500_$n/DroneCustomCommand ros2_px4_interfaces/srv/DroneCustomCommand '{operation: 'goTo', x: $x, y: $y, z: $z}'"
}

function dronesethorizontalvelocity() {
  n=${1:-0}
  vx=${2:-0}
  vy=${3:-0}
  eval "ros2 service call /X500_$n/DroneCustomCommand ros2_px4_interfaces/srv/DroneCustomCommand '{operation: 'setHorizontalVelocity', vx: $vx, vy: $vy}'"
}

function dronereturnhome() {
  n=${1:-0}
  eval "ros2 service call /X500_$n/DroneCustomCommand ros2_px4_interfaces/srv/DroneCustomCommand '{operation: 'returnHome'}'"
}

function droneland() {
  n=${1:-0}
  eval "ros2 service call /X500_$n/DroneCustomCommand ros2_px4_interfaces/srv/DroneCustomCommand '{operation: 'land'}'"
}

function dronerestart() {
  n=${1:-0}
  eval "ros2 service call /X500_$n/DroneCustomCommand ros2_px4_interfaces/srv/DroneCustomCommand '{operation: 'restart'}'"
}
# endregion

# region Target commands
function targetroverhold() {
  eval "ros2 service call /targetRover/targetCustomCommand ros2_px4_interfaces/srv/TargetCustomCommand '{operation: 'hold'}'"
}

function targetroverstraight() {
  throttle=${1:-0.5}
  eval "ros2 service call /targetRover/targetCustomCommand ros2_px4_interfaces/srv/TargetCustomCommand '{operation: 'straight', throttle: $throttle}'"
}

function targetrovercircle() {
  throttle=${1:-0.5}
  yawrate=${2:-1}
  eval "ros2 service call /targetRover/targetCustomCommand ros2_px4_interfaces/srv/TargetCustomCommand '{operation: 'circle', throttle: $throttle, yawrate: $yawrate}'"
}

function targetroverrandom() {
  eval "ros2 service call /targetRover/targetCustomCommand ros2_px4_interfaces/srv/TargetCustomCommand '{operation: 'random'}'"
}

function targetrovergoto() {
  x=${1:-0}
  y=${2:-0}
  eval "ros2 service call /targetRover/targetCustomCommand ros2_px4_interfaces/srv/TargetCustomCommand '{operation: 'goTo', x: $x, y: $y}'"
}

function targetroverrotate() {
  yawrate=${1:-1}
  eval "ros2 service call /targetRover/targetCustomCommand ros2_px4_interfaces/srv/TargetCustomCommand '{operation: 'rotate', yawrate: $yawrate}'"
}

function targetroverturn() {
  angle=${1:-90}
  eval "ros2 service call /targetRover/targetCustomCommand ros2_px4_interfaces/srv/TargetCustomCommand '{operation: 'turn', angle: $angle}'"
}

function targetroversquare() {
  eval "ros2 service call /targetRover/targetCustomCommand ros2_px4_interfaces/srv/TargetCustomCommand '{operation: 'square'}'"
}
# endregion

# region Test commands
function ddsserver() {
  fastdds discovery -i 0 -l 192.168.1.99 -p 11811
#  cd $HOME || exec $SHELL
#  export FASTRTPS_DEFAULT_PROFILES_FILE=discovery_server_configuration_file.xml
#  ros2 daemon stop
#  ros2 daemon start
}

function rds() {
#  export FASTRTPS_DEFAULT_PROFILES_FILE=$HOME/ros2_px4_ws/utils/super_client_configuration_file.xml
  export ROS_DISCOVERY_SERVER=192.168.1.99:11811
}

function superclient() {
  export FASTRTPS_DEFAULT_PROFILES_FILE=$HOME/ros2_px4_ws/utils/super_client_configuration_file.xml
}

function sendpackage() {
  n=${1:-0}
  ipEnd=$((10 + n))
  ip="192.168.1.$ipEnd"
  rsync -vrh --exclude={'build','install','log','rosbags','.git','.gitignore','src/ros2_px4_gazebo','src/ros2_px4_swarming/bagfiles'} $HOME/ros2_px4_ws ubuntu@$ip:/home/ubuntu
}

function sendparams() {
  n=${1:-0}
  ipEnd=$((10 + n))
  ip="192.168.1.$ipEnd"
  rsync -vrh $HOME/ros2_px4_ws/src/ros2_px4_swarming/parameters/drone_params.yaml ubuntu@$ip:/home/ubuntu/ros2_px4_ws/src/ros2_px4_swarming/parameters
}

function connectssh() {
  n=${1:-0}
  ipEnd=$((10 + n))
  ip="192.168.1.$ipEnd"
  user="ubuntu"
  password="swarm1234"
  sshpass -p $password ssh $user@$ip
}

function launchdrone() {
  id=${1:-0}
  export DRONE_ID=$id
  ros2 launch ros2_px4_swarming launch_drone.launch.py
}
# endregion
