from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    drone_namespace_arg = DeclareLaunchArgument(
        "drone_namespace", default_value=""
    )

    drone_controller_node = Node(
        executable="drone_controller",
        package="ros2_px4_control",
        name="drone_controller",
        namespace=LaunchConfiguration("drone_namespace")
    )

    odometry_sender_node = Node(
        executable="odometry_sender",
        package="ros2_px4_control",
        name="odometry_sender",
        namespace=LaunchConfiguration("drone_namespace")
    )

    return LaunchDescription([
        drone_namespace_arg,
        drone_controller_node,
        odometry_sender_node
    ])
