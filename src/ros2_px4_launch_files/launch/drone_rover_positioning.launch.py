from launch import LaunchDescription
from launch_ros.actions import Node


yaw_topic_name = "/yaw_sensor/estimated_yaw"
drone_name = "/drone"

kf_params_pos = [

    [{'deltaT': 1e-1}, {'R_uwb': 1e-2}, {'R_px4': 5e-1}, {'R_range_sensor': 1e-2},
    {'R_compass': 1e0}, {'Q_drone': 2e-4}, {'Q_rover': 5e-2}, 
    {'Q_compass': 2e3}, {'Q_rover_z': 1e-8}, {'Q_drone_z': 1e-1}, 
    {'rng_sensor_fuse_radius': 0.40}, {'vehicle_namespace': drone_name}, {'uwb_estimator': "/LS_uwb_estimator/norot_pos"},
    {"yaw_subscriber_topic": yaw_topic_name}, {"enable_watchdog": True}],

]

kf_bias_params_pos = [

    [{'deltaT': 1e-1}, {'R_uwb': 1e-2}, {'R_px4': 5e-1}, {'R_range_sensor': 5e-2},
    {'R_compass': 1e0}, {'R_apriltag': 1e-2}, {'Q_drone': 1e-4}, {'Q_rover': 3e-2}, 
    {'Q_compass': 1e2}, {'Q_rover_z': 1e-8}, {'Q_drone_z': 1e-1}, {'Q_bias': 1e-5},
    {'rng_sensor_fuse_radius': 0.40}, {'vehicle_namespace': drone_name}, {'uwb_estimator': "/LS_uwb_estimator/norot_pos"},
    {"yaw_subscriber_topic": yaw_topic_name}, {"enable_watchdog": True}],

]

def generate_launch_description():
    ld = LaunchDescription()

    
    ld.add_entity(Node(
        package = "ros2_px4_estimation",
        executable = "gazebo_yaw_estimator",
        name = "gazebo_yaw_estimator",
        parameters = [
            {"yaw_publisher_topic": yaw_topic_name},
            {"yaw_offset": 15.0},
            {"yaw_std_dev": 0.75}
        ]
    ))

    ld.add_entity(Node(
        package='ros2_px4_estimation',
        executable='drone_rover_uwb_positioning',
        namespace='LS_uwb_estimator',
        parameters=[
            {"sensor_id": "tag_0"},
            {"allowed_delay_ns": 1e8},
            {"max_range": 15.0},
            {"yaw_subscriber_topic": yaw_topic_name},
            {"method": "LS"}
        ]
    ))

    for i, param in enumerate(kf_params_pos):
        ld.add_entity(Node(
            package='ros2_px4_estimation',
            executable='drone_rover_kf_pos_theta_sim',
            namespace='KF_pos_estimator_' + str(i),
            parameters=param
        ))

    for i, param in enumerate(kf_bias_params_pos):
        ld.add_entity(Node(
            package='ros2_px4_estimation',
            executable='drone_rover_kf_pos_theta_bias_sim',
            namespace='KF_pos_estimator_bias_' + str(i),
            parameters=param
        ))

    return ld
