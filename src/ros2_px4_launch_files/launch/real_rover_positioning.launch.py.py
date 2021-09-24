from launch import LaunchDescription
from launch_ros.actions import Node


yaw_topic_name = "/yaw_estimator/estimated_yaw"

def generate_launch_description():
    ld = LaunchDescription()

    ld.add_entity(Node(
        package = "ros2_px4_estimation",
        executable = "magnetometer_yaw_estimator",
        name = "magnetometer_yaw_estimator",
        parameters = [
            {"yaw_publisher_topic": yaw_topic_name}
        ]
    ))

    return ld
