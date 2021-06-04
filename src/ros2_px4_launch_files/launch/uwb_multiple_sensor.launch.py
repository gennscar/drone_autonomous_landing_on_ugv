from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ros2_px4_estimation',
            executable='uwb_positioning',
            namespace='LS_uwb0_estimator',
            parameters=[
                {"sensor_id": "0"},
                {"method": "LS"}
            ]
        ),
        Node(
            package='ros2_px4_estimation',
            executable='uwb_positioning',
            namespace='LS_uwb1_estimator',
            parameters=[
                {"sensor_id": "10"},
                {"method": "LS"}
            ]
        ),
        Node(
            package='ros2_px4_estimation',
            executable='px4_positioning',
            namespace='PX4_estimator'
        ),
        Node(
            package='ros2_px4_testing',
            executable='positioning_error',
            namespace='positioning_error'
        )
    ])
