from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():

    drone_namespace_arg = DeclareLaunchArgument(
        "drone_namespace", default_value="/X500_1"
    )

    drone_controller_node = Node(
        executable="drone_controller",
        package="ros2_px4_control",
        name="DroneController",
        namespace=LaunchConfiguration("drone_namespace"),
        parameters=[{"vehicle_number": 2}]
    )
    
    
    fake_gps_node = Node(
        executable="GPSSimulator",
        package="ros2_px4_swarming",
        name="GPSSimulator",
        namespace=LaunchConfiguration("drone_namespace"),
        parameters=[{"QUEUE_SIZE": 1}]
    )


    return LaunchDescription([
        drone_namespace_arg,
        drone_controller_node,
        #fake_gps_node
    ])