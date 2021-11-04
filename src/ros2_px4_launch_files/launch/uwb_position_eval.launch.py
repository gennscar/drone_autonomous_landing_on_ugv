from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.actions import Shutdown
from launch.substitutions import LaunchConfiguration


ukf_params = {
    "Ukf0": [{"delta_t": 0.02}, {"q": 0.10}, {"r_uwb": 0.0100}, {"r_laser": 0.01}],
    "Ukf1": [{"delta_t": 0.02}, {"q": 0.10}, {"r_uwb": 0.0050}, {"r_laser": 0.01}],
    "Ukf2": [{"delta_t": 0.02}, {"q": 0.10}, {"r_uwb": 0.0020}, {"r_laser": 0.01}],
    "Ukf3": [{"delta_t": 0.02}, {"q": 0.10}, {"r_uwb": 0.0010}, {"r_laser": 0.01}],
    "Ukf4": [{"delta_t": 0.02}, {"q": 0.10}, {"r_uwb": 0.0005}, {"r_laser": 0.01}],
}


def generate_launch_description():
    launch_descriptor = LaunchDescription()

    launch_descriptor.add_entity(DeclareLaunchArgument(
        "drone_namespace", default_value=""
    ))

    launch_descriptor.add_entity(Node(
        package='ros2_px4_estimation',
        executable='uwb_driver',
        namespace='uwb_driver',
        parameters=[
            {"topic_name": "tag_0"},
            {"uwbPort": '/dev/ttyACM0'},
            {"anchors_pos_file_path": '/home/ubuntu/ros2_px4_ws/json/anchors_gabbia.json'},
        ]
    ))

    launch_descriptor.add_entity(Node(
        executable="drone_controller",
        package="ros2_px4_control",
        name="DroneController",
        namespace=LaunchConfiguration("drone_namespace"),
        parameters=[
            {"vehicle_number": 1}
        ]
    ))

    launch_descriptor.add_entity(Node(
        executable="uwb_positioning",
        package="ros2_px4_estimation",
        name="UwbPositioning",
        namespace=LaunchConfiguration("drone_namespace"),
        parameters=[{"sensor_id": "tag_0"}, {"allowed_delay_ns": 1e8}],
        on_exit=Shutdown()
    ))

    launch_descriptor.add_entity(Node(
        executable="gps_positioning",
        package="ros2_px4_estimation",
        name="GpsPositioning",
        namespace=LaunchConfiguration("drone_namespace")
    ))

    for ukf_name in ukf_params:
        launch_descriptor.add_entity(Node(
            executable="ukf_positioning",
            package="ros2_px4_estimation",
            name=ukf_name,
            namespace=LaunchConfiguration("drone_namespace"),
            parameters=ukf_params[ukf_name]
        ))

    for ukf_name in ukf_params:
        launch_descriptor.add_entity(Node(
            executable="odometry_error",
            package="ros2_px4_testing",
            name="OdometryError_" + ukf_name,
            namespace=LaunchConfiguration("drone_namespace"),
            parameters=[{"odometry_topic_name": ukf_name}]
        ))

    return launch_descriptor
