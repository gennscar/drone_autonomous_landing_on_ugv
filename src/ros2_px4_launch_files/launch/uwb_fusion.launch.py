from numpy.core.numeric import outer
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import Shutdown


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
            namespace='LS_estimator',
            parameters=[
                {"sensor_id": "Iris"},
                {"method": "LS"}
            ]
        ),
        Node(
            package='ros2_px4_estimation',
            executable='ukf_positioning',
            namespace='UKF_estimator'
        ),
        Node(
            package='ros2_px4_testing',
            executable='positioning_error',
            namespace='positioning_error'
        ),
        Node(
            package='ros2_px4_estimation',
            executable='uwb_estimate_2_px4',
            parameters=[
                {"estimator_name": "/UKF_estimator"},
            ]
        ),
        Node(
            package="ros2_px4_control",
            executable="drone_controller",
            name="drone_controller"
        )
    ])
