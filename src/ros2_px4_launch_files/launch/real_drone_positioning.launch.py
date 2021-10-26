from launch import LaunchDescription
from launch_ros.actions import Node


yaw_topic_name = "/yaw_sensor/estimated_yaw"
drone_name = "/X500_0"



kf_params_pos = [

    [{'deltaT': 1e-1}, {'R_uwb': 1e-2}, {'R_px4': 5e-1}, {'R_range_sensor': 5e-2},
    {'R_compass': 1e0}, {'Q_drone': 2e-4}, {'Q_rover': 5e-2}, {'Q_compass': 2e3},
    {'Q_rover_z': 1e-8}, {'Q_drone_z': 1e-1}, {'rng_sensor_fuse_radius': 0.30},
    {'vehicle_namespace': drone_name}, {'uwb_estimator': "/LS_uwb_estimator/norot_pos"},
    {"yaw_subscriber_topic": yaw_topic_name}, {"enable_watchdog": True}],

]

def generate_launch_description():
    ld = LaunchDescription()


    ld.add_entity(Node(
        package='ros2_px4_estimation',
        executable='uwb_driver',
        namespace='uwb_driver',
        parameters=[
            {"topic_name": "tag_0"},
            {"uwbPort": '/dev/ttyACM0'},
            {"anchors_pos_file_path": '/home/ubuntu/ros2_px4_ws/json/anchors_landing_pad.json'},
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
            executable='drone_rover_kf_pos_theta',
            namespace='KF_pos_estimator_' + str(i),
            parameters=param
        ))


    return ld
