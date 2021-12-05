from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
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
            {"anchors_pos_file_path": '/home/ubuntu/ros2_px4_ws/json/anchors.json'},
        ]
    )

    drone_controller_node = Node(
        executable="drone_controller",
        package="ros2_px4_control",
        name="DroneController",
        namespace=LaunchConfiguration("drone_namespace"),
        parameters=[
            {"vehicle_number": 1}
        ]
    )

    setpoints_flight = Node(
        executable="setpoints_flight",
        package="ros2_px4_testing",
        name="SetpointsFlight",
        namespace=LaunchConfiguration("drone_namespace"),
        on_exit=Shutdown()
    )

    recorder = ExecuteProcess(
        cmd=['ros2', 'bag', 'record', '-a'],
        output='screen'
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

    uwb_positioning_node = Node(
        executable="uwb_positioning",
        package="ros2_px4_estimation",
        name="UwbPositioning",
        namespace=LaunchConfiguration("drone_namespace"),
        parameters=[{"sensor_id": "tag_0"}, {"allowed_delay_ns": 2.5e7}],
        on_exit=Shutdown()
    )

    gps_positioning_node = Node(
        executable="gps_positioning",
        package="ros2_px4_estimation",
        name="GpsPositioning",
        namespace=LaunchConfiguration("drone_namespace")
    )

    kf_positioning_node = Node(
        executable="kf_loose_positioning",
        package="ros2_px4_estimation",
        name="KfPositioning",
        namespace=LaunchConfiguration("drone_namespace"),
        parameters=[{"delta_t": 0.02}, {"q": 0.1},
                    {"r_uwb": 0.0025}, {"r_laser": 0.01}],
    )

    ukf_positioning_node = Node(
        executable="ukf_positioning",
        package="ros2_px4_estimation",
        name="UkfPositioning",
        namespace=LaunchConfiguration("drone_namespace"),
        parameters=[{"delta_t": 0.02}, {"q": 0.1},
                    {"r_uwb": 0.0025}, {"r_laser": 0.01}],
        on_exit=Shutdown()
    )

    return LaunchDescription([
        # Arguments
        drone_namespace_arg,
        odometry_sender_sub_arg,

        # Nodes
        drone_controller_node,
        uwb_driver_node,
        setpoints_flight,
        recorder,
        odometry_sender_node,
        uwb_positioning_node,
        gps_positioning_node,
        # kf_positioning_node,
        ukf_positioning_node,
    ])
