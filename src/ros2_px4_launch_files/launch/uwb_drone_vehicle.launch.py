from launch import LaunchDescription
from launch_ros.actions import Node

estimation_mode = 'GN_10iter_uwb_estimator'
def generate_launch_description():
    return LaunchDescription([
        Node(
            package = 'ros2_px4_estimation',
            executable='uwb_positioning',
            namespace = estimation_mode,
            parameters=[
                {"sensor_id": "0"},
                {"method": "GN"},
                {"iterations": 10}
            ]
        ),
        Node(
            package='ros2_px4_testing',
            executable='drone_vehicle_uwb_positioning',
            namespace='drone_vehicle_uwb_positioning',
            parameters=[
                {"estimation_mode": estimation_mode},
            ]
        )
    ])
