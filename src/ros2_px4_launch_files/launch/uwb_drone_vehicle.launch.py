from launch import LaunchDescription
from launch_ros.actions import Node

estimation_mode = 'GN_10iter_uwb_estimator'
def generate_launch_description():
    return LaunchDescription([
        Node(
            package = 'estimation',
            executable='uwb_positioning',
            namespace = estimation_mode,
            parameters=[
                {"sensor_id": "0"},
                {"method": "GN"},
                {"iterations": 10}
            ]
        ),
        Node(
            package='testing',
            executable='chassis_to_drone_rotation',
            namespace='chassis_to_drone_rotation',
            parameters=[
                {"estimation_mode": estimation_mode},
            ]
        )
    ])
