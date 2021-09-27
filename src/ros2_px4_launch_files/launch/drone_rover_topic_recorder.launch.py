
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    ld.add_entity(Node(
        package='ros2_px4_testing',
        executable='drone_rover_topic_recorder',
        namespace='drone_rover_topic_recorder',
        parameters=[
            {"LS_rot_topic": "/LS_drone_rover_uwb_estimator/estimated_pos"},
            {"LS_norot_topic": "/LS_drone_rover_uwb_estimator/norot_pos"},
            {"KF_0_topic": "/KF_pos_estimator_0/estimated_pos"},
            {"range_sensor_topic": "/DistanceSensor_PubSubTopic"},
            {"yaw_sensor_topic": "/yaw_sensor/estimated_yaw"},
            {"vehicle_namespace": "/drone"}
        ]
    ))

    return ld