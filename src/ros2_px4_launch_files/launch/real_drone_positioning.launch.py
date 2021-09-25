from launch import LaunchDescription
from launch_ros.actions import Node


yaw_topic_name = "/yaw_sensor/estimated_yaw"
drone_name = "/X500_2"



kf_params_pos = [
    [{'deltaT': 1e-1}, {'R_uwb': 1e-3}, {'R_px4': 1e-1}, {'R_range_sensor': 5.625e-5},
    {'R_compass': 5e-1}, {'Q_drone': 1e-4}, {'Q_rover': 1e-2}, {'Q_compass': 5e2},
    {'Q_rover_z': 1e-8}, {'Q_drone_z': 1e-1}, {'rng_sensor_fuse_radius': 0.30},

    {'namespace_drone': drone_name}, {'uwb_estimator': "/LS_uwb_estimator"},
    {"rng_sensor_topic": "/range_sensor_positioning/estimated_pos"},
    {"yaw_subscriber_topic": yaw_topic_name}, {"enable_watchdog": False}]
]

def generate_launch_description():
    ld = LaunchDescription()

    ld.add_entity(Node(
        package='ros2_px4_estimation',
        executable='drone_rover_uwb_positioning',
        namespace='LS_uwb_estimator',
        parameters=[
            {"sensor_id": "Iris"},
            {"allowed_delay_ns": 1e4},
            {"max_range": 30.0},
            {"yaw_subscriber_topic": yaw_topic_name}
        ]
    ))

    
    ld.add_entity(Node(
    package='ros2_px4_estimation',
    executable='range_sensor_positioning',
    namespace='range_sensor_positioning',
    parameters = [
        {"vehicle_namespace": drone_name},
        {"rng_sensor_min_height": 0.15},
        {"rng_sensor_max_height": 10.0}
    ]
    ))

    for i, param in enumerate(kf_params_pos):
        ld.add_entity(Node(
            package='ros2_px4_estimation',
            executable='drone_rover_kf_pos_theta',
            namespace='KF_pos_estimator_' + str(i),
            parameters=param
        ))


    return ld
