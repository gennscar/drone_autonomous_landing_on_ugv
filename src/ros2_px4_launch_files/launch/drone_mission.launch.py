from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.actions import Shutdown
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    odometry_sender_sub_arg = DeclareLaunchArgument(
        "odometry_sub", default_value=""
    )

    drone_namespace_arg = DeclareLaunchArgument(
        "drone_namespace", default_value=""
    )

    uwb_driver_node = Node(
        package='ros2_px4_estimation',
        executable='uwb_driver',
        namespace='uwb_driver',
        parameters=[
            {"topic_name": "tag_0"},
            {"uwbPort": '/dev/ttyACM0'},
            {"anchors_pos_file_path": '/home/ubuntu/ros2_px4_ws/json/anchors_covivio_3.json'},
        ]
    )

    drone_controller_node = Node(
        executable="drone_controller",
        package="ros2_px4_control",
        name="DroneController",
        namespace=LaunchConfiguration("drone_namespace"),
	parameters=[
	    {"vehicle_number": 3}
	]
    )

    odometry_sender_node = Node(
        executable="odometry_sender",
        package="ros2_px4_control",
        name="OdometrySender",
        namespace=LaunchConfiguration("drone_namespace"),
        parameters=[
            {"odometry_sub": LaunchConfiguration("odometry_sub")}
        ]
    )

    gps_positioning_node = Node(
        executable="gps_positioning",
        package="ros2_px4_estimation",
        name="GpsPositioning",
        namespace=LaunchConfiguration("drone_namespace")
    )

    ukf_positioning_node = Node(
        executable="ukf_positioning",
        package="ros2_px4_estimation",
        name="UkfPositioning",
        namespace=LaunchConfiguration("drone_namespace"),
        parameters=[{"delta_t": 0.05}, {"q": 1e-3},
                    {"r_uwb": 0.05}, {"r_gps": 1e-5}],
	on_exit=Shutdown()
    )

    return LaunchDescription([
        uwb_driver_node,
        drone_namespace_arg,
        odometry_sender_sub_arg,
        drone_controller_node,
        odometry_sender_node,
        gps_positioning_node,
        ukf_positioning_node,
    ])
