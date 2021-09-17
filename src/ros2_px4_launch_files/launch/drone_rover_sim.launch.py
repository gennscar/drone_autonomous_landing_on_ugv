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
        {"uwb_estimator": "/LS_uwb_estimator"}
        ]
    )
    
    rover_controller_node = Node(
        package = "ros2_px4_control",
        executable = "rover_controller",
        name = "rover_controller",
        parameters = [
        {"vehicle_namespace": "/rover"}
        ]
    )

    ld.add_action(drone_controller_node)
    #ld.add_action(rover_controller_node)
    
    return ld