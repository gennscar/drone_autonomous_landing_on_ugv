#!/bin/bash
# run multiple instances of the 'px4' binary, with the gazebo SITL simulation
# It assumes px4 is already built, with 'make px4_sitl_default gazebo'

# The simulator is expected to send to TCP port 4560+i for i in [0, N-1]
# For example gazebo can be run like this:
#./ros2_px4_ws/src/ros2_px4_swarming/other/gazebo_sitl_multiple_run.sh -n 3 -m iris

function cleanup() {
	pkill -x px4
	pkill gzclient
	pkill gzserver
}

function posX() {
  echo "scale=2; 3 * c(360 * 0.017453293 / $NUM_DRONES * $2)" | bc -l
}

function posY() {
  echo "scale=2; 3 * s(360 * 0.017453293 / $NUM_DRONES * $2)" | bc -l
}

function spawn_model() {
	MODEL=$1
	N=$2 #Instance Number
	X=$3
	Y=$4
	echo "NUM_DRONES = $NUM_DRONES, N = $N"
	X=${X:=$(posX $num_vehicles $N)}
	Y=${Y:=$(posY $num_vehicles $N)}

	SUPPORTED_MODELS=("r1_rover" "iris")
	if [[ " ${SUPPORTED_MODELS[*]} " != *"$MODEL"* ]];
	then
		echo "ERROR: Currently only vehicle model $MODEL is not supported!"
		echo "       Supported Models: [${SUPPORTED_MODELS[@]}]"
		trap "cleanup" SIGINT SIGTERM EXIT
		exit 1
	fi

	working_dir="$build_path/instance_$n"
	[ ! -d "$working_dir" ] && mkdir -p "$working_dir"

	pushd "$working_dir" &>/dev/null
	echo "starting instance $N in $(pwd)"
	../bin/px4 -i $N -d "$build_path/etc" -w sitl_${MODEL}_${N} -s etc/init.d-posix/rcS >out.log 2>err.log &
  python3 ${ros2_package_path}/other/jinja_gen.py ${ros2_package_path}/models/${MODEL}/${MODEL}.sdf.jinja ${ros2_package_path} --mavlink_tcp_port $((4560+${N})) --mavlink_udp_port $((14560+${N})) --mavlink_id $((1+${N})) --gst_udp_port $((5600+${N})) --video_uri $((5600+${N})) --mavlink_cam_udp_port $((14530+${N})) --output-file /tmp/${MODEL}_${N}.sdf --anchor_id ${N} --namespace /X500_${N}/GroundTruth

  if [ $MODEL == "r1_rover" ]
	then
	  X=0
	  Y=0
	fi

	echo "Spawning ${MODEL}_${N} at ${X} ${Y}"

  gz model --spawn-file=/tmp/${MODEL}_${N}.sdf --model-name=${MODEL}_${N} -x ${X} -y ${Y} -z 0

	popd &>/dev/null

}

if [ "$1" == "-h" ] || [ "$1" == "--help" ]
then
	echo "Usage: $0 [-n <num_vehicles>] [-m <vehicle_model>] [-w <world>] [-s <script>]"
	echo "-s flag is used to script spawning vehicles e.g. $0 -s iris:3,r1_rover:1"
	exit 1
fi

while getopts n:m:w:s:t:l: option
do
	case "${option}"
	in
		n) NUM_VEHICLES=${OPTARG};;
		m) VEHICLE_MODEL=${OPTARG};;
		w) WORLD=${OPTARG};;
		s) SCRIPT=${OPTARG};;
		t) TARGET=${OPTARG};;
		l) LABEL=_${OPTARG};;
	esac
done

num_vehicles=${NUM_VEHICLES:=3}
world=${WORLD:=swarming_empty}
target=${TARGET:=px4_sitl_default}
vehicle_model=${VEHICLE_MODEL:="iris"}
export PX4_SIM_MODEL=${vehicle_model}${LABEL}

src_path="$HOME/PX4-Autopilot"
ros2_package_path="$HOME/ros2_px4_ws/src/ros2_px4_swarming"

build_path=${src_path}/build/${target}
mavlink_udp_port=14560
mavlink_tcp_port=4560

echo "killing running instances"
pkill -x px4 || true

sleep 1

source ${src_path}/Tools/setup_gazebo.bash ${src_path} ${src_path}/build/${target}

echo "Starting gazebo"
gzserver ${ros2_package_path}/worlds/${world}.world --verbose &
sleep 5

n=0
if [ -z ${SCRIPT} ]; then
	if [ $num_vehicles -gt 255 ]
	then
		echo "Tried spawning $num_vehicles vehicles. The maximum number of supported vehicles is 255"
		exit 1
	fi

	while [ $n -lt $num_vehicles ]; do
		spawn_model ${vehicle_model} $n
		n=$(($n + 1))
	done
else
	IFS=,
	for target in ${SCRIPT}; do
		target="$(echo "$target" | tr -d ' ')" #Remove spaces
		target_vehicle=$(echo $target | cut -f1 -d:)
		target_number=$(echo $target | cut -f2 -d:)
		target_x=$(echo $target | cut -f3 -d:)
		target_y=$(echo $target | cut -f4 -d:)

		if [ $n -gt 255 ]
		then
			echo "Tried spawning $n vehicles. The maximum number of supported vehicles is 255"
			exit 1
		fi

		m=0
		while [ $m -lt ${target_number} ]; do
			spawn_model ${target_vehicle} $n $target_x $target_y
			m=$(($m + 1))
			n=$(($n + 1))
		done
	done

fi
trap "cleanup" SIGINT SIGTERM EXIT

echo "Starting gazebo client"
gzclient
