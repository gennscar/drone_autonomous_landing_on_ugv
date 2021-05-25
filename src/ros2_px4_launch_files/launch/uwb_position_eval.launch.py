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
            executable='uwb_positioning',
            namespace='GN10iter_uwb_estimator',
            parameters=[
                {"sensor_id": "0"},
                {"method": "GN"},
                {"iterations": 10}
            ]
        ),
        Node(
            package='estimation',
            executable='px4_positioning',
            namespace='PX4_estimator'
        ),
        Node(
            package='estimation',
            executable='kf_positioning',
            namespace='KF_estimator'
        ),
        Node(
            package="ros2_px4_control",
            executable="drone_controller",
            name="DroneController"
        ),
        Node(
            package='testing',
            executable='positioning_error',
            namespace='positioning_error'
        )
    ])
