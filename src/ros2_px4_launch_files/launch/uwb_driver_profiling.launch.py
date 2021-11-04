from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    uwb_driver_node = Node(
        package='ros2_px4_estimation',
        executable='uwb_driver',
        namespace='uwb_driver',
        parameters=[
            {"topic_name": "tag_0"},
            {"uwbPort": '/dev/ttyACM0'},
            {"anchors_pos_file_path": '/home/cosimocon/Dev/ros2_px4_ws/json/anchors.json'},
        ],
        prefix=['valgrind --tool=callgrind --dump-instr=yes -v --instr-atstart=no']
    )

    return LaunchDescription([
        uwb_driver_node
    ])
