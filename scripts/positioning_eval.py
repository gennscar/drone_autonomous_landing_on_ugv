import os
import time
import subprocess
import signal
import numpy as np

from ros2bag import BagFileParser

PX4_AUTOPILOT_PATH = "/home/cosimo/PX4-Autopilot"
ROSBAGS_PATH = "/home/cosimo/ros2_px4_ws/rosbags"
LOG_PATH = "/home/cosimo/ros2_px4_ws/scripts/log"


def run_evaluation(world_name, timeout_sec=60):
    subprocess.run(["killall", "-9", "-q", "micrortps_agent",
                   "gazebo", "gzclient", "gzserver"])

    os.environ['PX4_SITL_WORLD'] = world_name + ".world"

    with open(LOG_PATH + "/simulation_proc.txt", 'w') as output:
        simulation_proc = subprocess.Popen(
            ["make", "px4_sitl_rtps", "gazebo"],
            cwd=PX4_AUTOPILOT_PATH,
            stdout=output,
            stderr=subprocess.STDOUT,
            universal_newlines=True,
            preexec_fn=os.setsid
        )

    with open(LOG_PATH + "/rtpsagent_proc.txt", 'w') as output:
        rtpsagent_proc = subprocess.Popen(
            ["micrortps_agent", "-t", "UDP"],
            stdout=output,
            stderr=subprocess.STDOUT,
            universal_newlines=True,
            preexec_fn=os.setsid
        )

    with open(LOG_PATH + "/ros2nodes_proc.txt", 'w') as output:
        ros2nodes_proc = subprocess.Popen(
            ["ros2", "launch", "ros2_px4_launch_files" "uwb_position_eval.launch.py"],
            stdout=output,
            stderr=subprocess.STDOUT,
            universal_newlines=True,
            preexec_fn=os.setsid
        )

    with open(LOG_PATH + "/ros2bag_proc.txt", 'w') as output:
        ros2bag_proc = subprocess.Popen(
            ["ros2", "bag", "record", "-a", "-o", world_name],
            cwd=ROSBAGS_PATH,
            stdout=output,
            stderr=subprocess.STDOUT,
            universal_newlines=True,
            preexec_fn=os.setsid
        )

    processes_map = {
        "SIMULATION": simulation_proc,
        "RTPS_AGENT": rtpsagent_proc,
        "ROS2NODES": ros2nodes_proc,
        "ROS2BAG": ros2bag_proc
    }

    print("Evaluation...", timeout_sec, "seconds remaining")
    time.sleep(timeout_sec)

    print("Evaluation terminated, closing processes...")
    for proc in processes_map.values():
        os.killpg(os.getpgid(proc.pid), signal.SIGINT)
        proc.wait()


if __name__ == "__main__":
    WORLD_NAME = "4anchors"

    # run_evaluation(WORLD_NAME, timeout_sec=5)

    parser = BagFileParser(ROSBAGS_PATH + "/" +
                           WORLD_NAME + "/" + WORLD_NAME + "_0.db3")

    groundtruth_msgs = parser.get_messages("/ground_truth_Iris/odom")

    groundtruth_time = map(
        lambda x:  x[1].header.stamp.sec * 1e9 +
        x[1].header.stamp.nanosec,
        groundtruth_msgs
    )
    groundtruth_time = np.array(list(groundtruth_time), np.int)

    groundtruth_position = map(
        lambda x:  [
            x[1].pose.pose.position.x,
            x[1].pose.pose.position.y,
            x[1].pose.pose.position.z
        ],
        groundtruth_msgs
    )
    groundtruth_position = np.array(list(groundtruth_position))

    groundtruth_orientation = map(
        lambda x:  [
            x[1].pose.pose.orientation.x,
            x[1].pose.pose.orientation.y,
            x[1].pose.pose.orientation.z,
            x[1].pose.pose.orientation.w
        ],
        groundtruth_msgs
    )
    groundtruth_orientation = np.array(list(groundtruth_orientation))

    print(groundtruth_orientation)
