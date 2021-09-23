from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    ld = LaunchDescription()

    drone_controller_node = Node(
        package = "ros2_px4_control",
        executable = "drone_controller_old",
        name = "drone_controller_old",
        parameters = [
        {"control_mode": 1},
        {"vehicle_namespace": "/drone"},
        {"vehicle_number": 2},
        {"uwb_estimator": "/KF_pos_estimator_pos_0"},
        {"rng_sensor_topic": "/range_sensor_positioning/estimated_pos"}
        ]
    )

    ld.add_action(drone_controller_node)
    
    return ld