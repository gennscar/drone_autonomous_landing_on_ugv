from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():

    drone_namespace_arg = DeclareLaunchArgument(
        "drone_namespace", default_value="/X500_0"
    )

    drone_controller_node = Node(
        executable="drone_controller",
        package="ros2_px4_control",
        name="DroneController",
        namespace=LaunchConfiguration("drone_namespace"),
        parameters=[{"vehicle_number": 1}]
    )


    return LaunchDescription([
        drone_namespace_arg,
        drone_controller_node,
    ])