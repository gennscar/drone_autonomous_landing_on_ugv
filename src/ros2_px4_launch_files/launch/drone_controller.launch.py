from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():

    drone_namespace_arg = DeclareLaunchArgument(
        "drone_namespace", default_value="/drone"
    )

    drone_controller_node = Node(
        executable="drone_controller",
        package="ros2_px4_control",
        name="DroneController",
        namespace=LaunchConfiguration("drone_namespace")
    )

    """
    drone_controller_node = Node(
        package = "ros2_px4_control",
        executable = "drone_controller_old",
        name = "drone_controller_old",
        parameters = [
        {"control_mode": 1},
        {"vehicle_namespace": "/drone"},
        {"vehicle_number": 1},
        {"uwb_estimator": "/KF_pos_estimator_0/estimated_pos"}
        ]
    )"""

    return LaunchDescription([
        drone_namespace_arg,
        drone_controller_node
    ])