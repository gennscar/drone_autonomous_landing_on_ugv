from launch import LaunchDescription
from launch_ros.actions import Node

kf_params = [
    [{'deltaT': 5e-3}, {'R_uwb': 0.01}, {'R_px4': 0.1}, {'Q': 1e-2}]
]

def generate_launch_description():
    ld = LaunchDescription()

    ld.add_entity(Node(
        package='ros2_px4_estimation',
        executable='drone_vehicle_uwb_positioning',
        namespace='LS_uwb_estimator',
        parameters=[
            {"sensor_id": "0"},
            {"method": "LS"}
        ]
    ))

    for i, param in enumerate(kf_params):
        ld.add_entity(Node(
            package='ros2_px4_estimation',
            executable='drone_vehicle_kf_loose',
            namespace='KF_estimator_' + str(i),
            parameters=param
        ))

    ld.add_entity(Node(
        package='ros2_px4_estimation',
        executable='drone_vehicle_px4_positioning',
        namespace='PX4_estimator'
    ))

    ld.add_entity(Node(
        package='ros2_px4_testing',
        executable='drone_vehicle_positioning_error',
        namespace='drone_vehicle_positioning_error'
    ))

    return ld
