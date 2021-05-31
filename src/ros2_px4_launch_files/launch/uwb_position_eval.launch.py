from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ros2_px4_estimation',
            executable='uwb_positioning',
            namespace='LS_uwb_estimator',
            parameters=[
                {"sensor_id": "0"},
                {"method": "LS"}
            ]
        ),
        Node(
            package='ros2_px4_estimation',
            executable='uwb_positioning',
            namespace='GN_uwb_estimator',
            parameters=[
                {"sensor_id": "0"},
                {"method": "GN"}
            ]
        ),
        Node(
            package='ros2_px4_estimation',
            executable='uwb_positioning',
            namespace='GN_10it_uwb_estimator',
            parameters=[
                {"sensor_id": "0"},
                {"method": "GN"},
                {"iterations": 10}
            ]
        ),
        Node(
            package='ros2_px4_testing',
            executable='positioning_error',
            namespace='positioning_error'
        )
    ])
