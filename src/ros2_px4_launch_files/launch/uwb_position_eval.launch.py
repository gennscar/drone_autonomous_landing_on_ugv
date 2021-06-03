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
            package='ros2_px4_estimation',
            executable='px4_positioning',
            namespace='PX4_estimator'
        ),
        Node(
            package='ros2_px4_estimation',
            executable='ukf_positioning',
            namespace='UKF_estimator',
            parameters=[{'deltaT': 1e-2},
                        {'R_uwb': 0.0002},
                        {'R_px4': 5.},
                        {'Q': 0.05},
                        {'AdaptG': 0.2}]
        ),
        Node(
            package='ros2_px4_testing',
            executable='positioning_error',
            namespace='positioning_error'
        )
    ])
