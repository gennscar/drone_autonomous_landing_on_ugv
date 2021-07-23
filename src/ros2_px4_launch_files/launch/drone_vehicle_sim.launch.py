from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    ld = LaunchDescription()

    drone_controller_node = Node(
        package = "ros2_px4_control",
        executable = "drone_controller_old",
        name = "DroneController",
        parameters = [
        {"control_mode": 2},
        {"vehicle_namespace": "/drone"},
        {"vehicle_number": 2},
        {"x0": 0.0},
        {"y0": 3.0},
        ]
    )
    vehicle_controller_node = Node(
        package = "ros2_px4_control",
        executable = "vehicle_controller",
        name = "VehicleController"
    )

    ld.add_action(drone_controller_node)
    ld.add_action(vehicle_controller_node)
    return ld