from launch import LaunchDescription
from launch_ros.actions import Node


yaw_topic_name = "/LS_drone_rover_uwb_estimator/estimated_yaw"
drone_name = "/drone"

kf_params_pos = [
    [{'deltaT': 1e-1}, {'R_uwb': 1e-2}, {'R_px4': 1e-1}, {'R_range_sensor': 5.625e-5},
    {'R_compass': 1e0}, {'Q_drone': 1e-4}, {'Q_rover': 1e-3}, {'Q_compass': 1e2},
    {'Q_rover_z': 1e-8}, {'Q_drone_z': 1e-1}, {'rng_sensor_fuse_radius': 0.40},

    {'vehicle_namespace': drone_name}, {'uwb_estimator': "/LS_drone_rover_uwb_estimator/norot_pos"},
    {"yaw_subscriber_topic": yaw_topic_name}, {"enable_watchdog": True}]
]

def generate_launch_description():
    ld = LaunchDescription()

    ld.add_entity(Node(
        package='ros2_px4_estimation',
        executable='drone_rover_uwb_positioning',
        namespace='LS_drone_rover_uwb_estimator',
        parameters=[
            {"sensor_id_0": "tag_0"},
            {"sensor_id_1": "tag_1"},
            {"allowed_delay_ns": 1e2},
            {"max_range": 50.0},
        ]
    ))

    
    for i, param in enumerate(kf_params_pos):
        ld.add_entity(Node(
            package='ros2_px4_estimation',
            executable='drone_rover_kf_new',
            namespace='KF_pos_estimator_' + str(i),
            parameters=param
        ))
    

    return ld
