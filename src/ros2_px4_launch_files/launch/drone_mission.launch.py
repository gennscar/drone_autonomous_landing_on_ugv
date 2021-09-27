from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    odometry_sender_sub_arg = DeclareLaunchArgument(
        "odometry_sub", default_value=""
    )

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
        namespace=LaunchConfiguration("drone_namespace"),
        parameters=[
            {"odometry_sub": LaunchConfiguration("odometry_sub")}
        ]
    )

    gps_positioning_node = Node(
        executable="gps_positioning",
        package="ros2_px4_estimation",
        name="gps_positioning",
        namespace=LaunchConfiguration("drone_namespace")
    )

    return LaunchDescription([
        drone_namespace_arg,
        odometry_sender_sub_arg,
        drone_controller_node,
        odometry_sender_node,
        gps_positioning_node
    ])
