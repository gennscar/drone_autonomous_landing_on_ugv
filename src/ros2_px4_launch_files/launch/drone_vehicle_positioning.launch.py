from launch import LaunchDescription
from launch_ros.actions import Node

kf_params = [
    [{'deltaT': 1e-1}, {'R_apriltag': 1e-1}, {'R_px4_yaw': 1.}, {'Q': 1e2}, {'namespace_rover': "/rover"}]

]

def generate_launch_description():
    ld = LaunchDescription()

    ld.add_entity(Node(
        package='ros2_px4_estimation',
        executable='drone_vehicle_uwb_positioning',
        namespace='LS_uwb_estimator',
        parameters=[
            {"sensor_id": "Iris"},
            {"method": "LS"},
            {"vehicle_namespace": "/rover"}

        ]
    ))

    ld.add_entity(Node(
        package = "ros2_px4_estimation",
        executable = "apriltag_yaw_estimator",
        name = "ApriltagYawEstimator",
        parameters = [{"vehicle_namespace": "/drone"}]
    ))
    ld.add_entity(Node(
        package = "ros2_px4_estimation",
        executable = "px4_yaw_estimator",
        name = "PX4YawEstimator",
        parameters = [{"vehicle_namespace": "/rover"}]
    ))

    for i, param in enumerate(kf_params):
        ld.add_entity(Node(
            package='ros2_px4_estimation',
            executable='drone_vehicle_kf_yaw',
            namespace='KF_yaw_estimator_' + str(i),
            parameters=param
        ))

    """for i, param in enumerate(kf_params):
        ld.add_entity(Node(
            package='ros2_px4_estimation',
            executable='drone_vehicle_kf_yaw_bias',
            namespace='KF_yaw_bias_estimator_' + str(i),
            parameters=param
        ))"""
    ld.add_entity(Node(
        package='ros2_px4_testing',
        executable='drone_vehicle_positioning_error',
        namespace='drone_vehicle_positioning_error',
    ))

    ld.add_entity(Node(
        package='ros2_px4_testing',
        executable='yaw_error',
        namespace='yaw_error',
        parameters = [{"noise": 1}]
    ))


    return ld
