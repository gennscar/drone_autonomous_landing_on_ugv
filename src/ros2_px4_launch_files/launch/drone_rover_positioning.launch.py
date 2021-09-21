from launch import LaunchDescription
from launch_ros.actions import Node

kf_params_pos = [

    [{'deltaT': 1e-1}, {'R_uwb': 1e-3}, {'R_px4': 1e-1}, {'R_range_sensor': 6.25e-4}, 
    {'rng_sensor_fuse_radius': 0.30}, {'Q_drone': 1e-4}, {'Q_rover': 1e-2}, 
    {'Q_rover_z': 1e-8}, {'Q_drone_z': 1e-3}, {'include_drone': 1}, 
    {'namespace_drone': "/drone"}, {'uwb_estimator': "/LS_uwb_estimator"}],

]

kf_params_yaw= [

    [{'deltaT': 1e-1}, {'R_px4_yaw': 1e-5}, {'Q': 1e-1}]

]

def generate_launch_description():
    ld = LaunchDescription()



    ld.add_entity(Node(
        package='ros2_px4_estimation',
        executable='drone_rover_uwb_positioning',
        namespace='LS_uwb_estimator',
        parameters=[
            {"sensor_id": "Iris"},
            {"method": "LS"},
            {"vehicle_namespace": "/rover"},
            {"yaw_estimator": "/px4_estimator"}
        ]
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

    for i, param in enumerate(kf_params_pos):
        ld.add_entity(Node(
            package='ros2_px4_estimation',
            executable='drone_rover_kf_pos',
            namespace='KF_pos_estimator_pos_' + str(i),
            parameters=param
        ))

    ld.add_entity(Node(
        package = "ros2_px4_estimation",
        executable = "px4_yaw_estimator",
        name = "px4_yaw_estimator",
        parameters = [
            {"vehicle_namespace": "/rover"}
        ]
    ))

    ld.add_entity(Node(
        package='ros2_px4_testing',
        executable='drone_rover_positioning_error',
        namespace='drone_rover_positioning_error',
        parameters=[
            {"vehicle_namespace": "/rover"},
            {"sensor_id": "Iris"}
        ]
        ))

    for i, param in enumerate(kf_params_yaw):
        ld.add_entity(Node(
            package='ros2_px4_estimation',
            executable='drone_rover_kf_yaw',
            namespace='KF_yaw_estimator_' + str(i),
            parameters=param
        ))


    ld.add_entity(Node(
        package='ros2_px4_testing',
        executable='rover_yaw_error',
        namespace='rover_yaw_error',
        parameters=[
            {"vehicle_namespace": "/rover"}
        ]))


    """ld.add_entity(Node(
        package = "ros2_px4_estimation",
        executable = "apriltag_yaw_estimator",
        name = "ApriltagYawEstimator",
        parameters = [{"vehicle_namespace": "/drone"}]))
    """
    
    return ld
