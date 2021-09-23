from launch import LaunchDescription
from launch_ros.actions import Node



kf_params_pos = [
    [{'deltaT': 1e-1}, {'R_uwb': 1e-3}, {'R_px4': 1e-1}, {'R_range_sensor': 6.25e-4},
    {'R_compass': 1e-3}, {'rng_sensor_fuse_radius': 0.30}, {'Q_drone': 1e-4}, 
    {'Q_rover': 1e-2}, {'Q_compass': 1e0}, {'Q_rover_z': 1e-8}, {'Q_drone_z': 1e-3}, 
    {'namespace_drone': "/drone"}, {'uwb_estimator': "/LS_uwb_estimator"}],
]

def generate_launch_description():
    ld = LaunchDescription()


    ld.add_entity(Node(
        package='ros2_px4_estimation',
        executable='drone_rover_uwb_positioning',
        namespace='LS_uwb_estimator',
        parameters=[
            {"sensor_id": "Iris"},
            {"allowed_delay_ns": 1e2},
            {"max_range": 100.0},
            {"vehicle_namespace": "/rover"},
            {"yaw_estimator": "/px4_estimator"}
        ]
    ))

    ld.add_entity(Node(
        package = "ros2_px4_estimation",
        executable = "px4_yaw_estimator",
        name = "px4_yaw_estimator",
        parameters = [
            {"vehicle_namespace": "/rover"},
            {"yaw_offset": 15.0},
            {"yaw_std_dev": 2.5}
        ]
    ))

    for i, param in enumerate(kf_params_pos):
        ld.add_entity(Node(
            package='ros2_px4_estimation',
            executable='drone_rover_kf_pos_theta',
            namespace='KF_pos_estimator_pos_' + str(i),
            parameters=param
        ))

        ld.add_entity(Node(
        package='ros2_px4_estimation',
        executable='range_sensor_positioning',
        namespace='range_sensor_positioning',
        parameters = [
            {"rng_sensor_min_height": 0.2},
            {"rng_sensor_max_height": 10.0}
        ]
        ))

    return ld
