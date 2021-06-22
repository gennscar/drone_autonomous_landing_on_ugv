from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ros2_px4_estimation',
            executable='uwb_driver',
            parameters=[
                {'topic_name': '/uwb_sensor/Iris'}
            ]
        ),
        Node(
            package='ros2_px4_estimation',
            executable='uwb_positioning',
            namespace='LS_uwb_estimator',
            parameters=[
                {"sensor_id": "Iris"},
                {"method": "LS"}
            ]
        )
    ])
