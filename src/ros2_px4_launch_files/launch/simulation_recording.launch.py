from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="ros2_px4_control",
            executable="drone_controller",
            name="drone_controller"
        ),
        Node(
            package="ros2_px4_testing",
            executable="setpoints_flight",
            name="setpoints_flight",
        ),
        ExecuteProcess(
            cmd=['ros2', 'bag', 'record', '-o',
                 'data_records/simulated_uwb_gazebo', '--all'],
            output='screen'
        )
    ])
