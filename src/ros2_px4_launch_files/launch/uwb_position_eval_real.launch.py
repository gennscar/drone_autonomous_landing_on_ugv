from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ros2_px4_estimation',
            executable='uwb_driver'
        ),
        Node(
            package='ros2_px4_testing',
            executable='uwb_positioning_real',
            namespace='LS_uwb_estimator',
            parameters=[
                {"method": "LS"}
            ]
        ),
        Node(
            package = 'ros2_px4_testing',
            executable='uwb_positioning_real',
            namespace = 'GN_10iter_uwb_estimator',
            parameters=[
                {"method": "GN"},
                {"iterations": 10}
            ]
        ),
        Node(
            package = 'ros2_px4_testing',
            executable='uwb_positioning_real',
            namespace = 'GN_1iter_uwb_estimator',
            parameters=[
                {"method": "GN"},
                {"iterations": 1}
            ]
        ),
    ])
