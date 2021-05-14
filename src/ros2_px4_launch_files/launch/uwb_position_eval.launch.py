from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='estimation',
            executable='uwb_positioning',
            namespace='LS_uwb_estimator',
            parameters=[
                {"sensor_id": "0"},
                {"method": "LS"}
            ]
        ),
        Node(
            package='estimation',
            executable='uwb_positioning',
            namespace='GN_uwb_estimator',
            parameters=[
                {"sensor_id": "0"},
                {"method": "GN"}
            ]
        ),
        Node(
            package='estimation',
            executable='ins_positioning',
            namespace='ins_estimator'
        ),
        Node(
            package='testing',
            executable='positioning_error',
            namespace='positioning_error'
        )
    ])
