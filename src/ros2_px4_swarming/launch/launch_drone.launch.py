import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory, get_package_prefix


def generate_launch_description():
    ld = LaunchDescription()

    # Environment variables
    ID = int(os.environ['DRONE_ID'])

    # Parameters file path
    params = os.path.join(
        get_package_share_directory('ros2_px4_swarming'),
        'parameters',
        'drone_params.yaml'
    )

    # Launch drone
    anchorDrone = Node(
        package='ros2_px4_swarming',
        namespace='X500_' + str(ID),
        executable='anchorDrone',
        name='X500_' + str(ID),
        parameters=[
            params,
            {'ID': ID}
        ]
    )
    ld.add_action(anchorDrone)

    return ld
