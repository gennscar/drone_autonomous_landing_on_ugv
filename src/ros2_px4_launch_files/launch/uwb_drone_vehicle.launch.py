from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ros2_px4_estimation',
            executable='drone_vehicle_uwb_positioning',
            namespace='LS_uwb_estimator',
            parameters=[
                {"sensor_id": "0"},
                {"method": "LS"}
            ]
        ),
        Node(
            package='ros2_px4_estimation',
            executable='drone_vehicle_kf_loose',
            namespace='KF_loose'
        ),
        Node(
            package='ros2_px4_estimation',
            executable='drone_vehicle_px4_positioning',
            namespace='PX4_estimator'
        ),
        Node(
            package='ros2_px4_testing',
            executable='drone_vehicle_positioning_error',
            namespace='drone_vehicle_positioning_error'
        ),

    ])
